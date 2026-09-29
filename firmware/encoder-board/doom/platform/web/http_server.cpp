// http_server.cpp
//
// A small HTTP/1.1 server on lwIP's raw TCP API. Every response closes its
// connection, except /stream, which stays open and gets one multipart part
// per frame.

#include "http_server.hpp"

#include <strings.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>

#include "assets.hpp"
#include "lwip/tcp.h"

namespace
{
    constexpr uint16_t kPort = 80;
    constexpr size_t kMaxConnections = 6;
    constexpr size_t kRequestBytes = 1536;  // browsers send roughly 500-800 bytes of headers
    constexpr uint8_t kPollInterval = 4;    // in lwIP's 500 ms coarse ticks
    constexpr uint8_t kRequestTimeoutPolls = 5;

    constexpr char kPageHeader[] =
        "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: %u\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
    constexpr char kStreamHeader[] =
        "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
    constexpr char kPartHeader[] = "--frame\r\nContent-Type: image/png\r\nContent-Length: %u\r\n\r\n";
    constexpr char kPartTrailer[] = "\r\n";
    constexpr char kNoContent[] = "HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n";
    constexpr char kNotFound[] = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    constexpr char kBadRequest[] = "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";

    enum class State
    {
        kFree,
        kReading,    // collecting the request
        kReplying,   // one response queued; closes once it is acknowledged
        kStreaming,  // /stream: one part per frame until the viewer goes away
    };

    struct Connection
    {
        State state;
        tcp_pcb* pcb;
        uint32_t unacked;  // bytes queued in lwIP but not yet acknowledged
        uint8_t idle_polls;
        size_t received;
        char request[kRequestBytes];

        // Current frame, kStreaming only.
        char part_header[80];
        size_t part_header_len;
        bool part_header_queued;
        const uint8_t* body;
        size_t body_left;
        bool trailer_queued;
    };

    Connection connections[kMaxConnections];
    Connection* stream = nullptr;
    http::KeyHandler key_handler = nullptr;
    uint32_t frames_sent = 0;

    // Returns ERR_ABRT if the connection had to be aborted, which a callback
    // for that pcb must then return.
    err_t close(Connection& c)
    {
        tcp_pcb* pcb = c.pcb;
        if (&c == stream)
            stream = nullptr;
        c.state = State::kFree;
        c.pcb = nullptr;
        if (pcb == nullptr)
            return ERR_OK;

        tcp_arg(pcb, nullptr);
        tcp_recv(pcb, nullptr);
        tcp_sent(pcb, nullptr);
        tcp_err(pcb, nullptr);
        tcp_poll(pcb, nullptr, 0);
        if (tcp_close(pcb) == ERR_OK)
            return ERR_OK;
        tcp_abort(pcb);
        return ERR_ABRT;
    }

    bool queue(Connection& c, const void* data, size_t len, uint8_t flags)
    {
        if (tcp_write(c.pcb, data, static_cast<u16_t>(len), flags) != ERR_OK)
            return false;  // send buffer or segment queue full: try again after the next ack
        c.unacked += len;
        return true;
    }

    err_t reply(Connection& c, const char* head, size_t head_len, const void* body = nullptr, size_t body_len = 0)
    {
        c.state = State::kReplying;
        const uint8_t head_flags = TCP_WRITE_FLAG_COPY | (body_len ? TCP_WRITE_FLAG_MORE : 0);
        // Replies are small, so an empty send buffer always takes them whole.
        if (!queue(c, head, head_len, head_flags) || (body_len && !queue(c, body, body_len, 0)))
            return close(c);
        tcp_output(c.pcb);
        return ERR_OK;
    }

    template <size_t N>
    err_t reply(Connection& c, const char (&text)[N])
    {
        return reply(c, text, N - 1);
    }

    void pump(Connection& c)
    {
        bool queued = false;
        if (!c.part_header_queued)
        {
            if (!queue(c, c.part_header, c.part_header_len, TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE))
                return;
            c.part_header_queued = queued = true;
        }
        while (c.body_left > 0)
        {
            const size_t space = tcp_sndbuf(c.pcb);
            if (space == 0)
                break;
            const size_t chunk = std::min({c.body_left, space, size_t{0xFFFF}});
            if (!queue(c, c.body, chunk, TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE))
                break;
            c.body += chunk;
            c.body_left -= chunk;
            queued = true;
        }
        if (c.body_left == 0 && !c.trailer_queued && queue(c, kPartTrailer, sizeof(kPartTrailer) - 1, TCP_WRITE_FLAG_COPY))
        {
            c.trailer_queued = queued = true;
            ++frames_sent;
        }
        if (queued)
            tcp_output(c.pcb);
    }

    err_t start_stream(Connection& c)
    {
        if (stream)
            close(*stream);  // one viewer at a time: the newest wins

        c.state = State::kStreaming;
        c.part_header_queued = true;
        c.body_left = 0;
        c.trailer_queued = true;
        if (!queue(c, kStreamHeader, sizeof(kStreamHeader) - 1, TCP_WRITE_FLAG_COPY))
            return close(c);
        tcp_output(c.pcb);
        stream = &c;
        return ERR_OK;
    }

    bool starts_with(const char* text, const char* prefix) { return std::strncmp(text, prefix, std::strlen(prefix)) == 0; }

    size_t content_length(const char* request, const char* headers_end)
    {
        for (const char* line = std::strstr(request, "\r\n"); line && line < headers_end;
             line = std::strstr(line + 2, "\r\n"))
        {
            if (strncasecmp(line + 2, "Content-Length:", 15) == 0)
                return std::strtoul(line + 2 + 15, nullptr, 10);
        }
        return 0;
    }

    err_t handle_request(Connection& c)
    {
        const bool full = c.received == kRequestBytes - 1;
        const char* headers_end = std::strstr(c.request, "\r\n\r\n");
        if (headers_end == nullptr)
            return full ? reply(c, kBadRequest) : ERR_OK;  // wait for the rest of the headers
        const size_t head_len = static_cast<size_t>(headers_end + 4 - c.request);

        if (starts_with(c.request, "GET / ") || starts_with(c.request, "GET /index.html "))
        {
            char head[sizeof(kPageHeader) + 8];
            const int len = std::snprintf(head, sizeof(head), kPageHeader, index_html_size);
            return reply(c, head, static_cast<size_t>(len), index_html, index_html_size);
        }
        if (starts_with(c.request, "GET /stream "))
            return start_stream(c);
        if (starts_with(c.request, "POST /key "))
        {
            const size_t body_len = content_length(c.request, headers_end);
            if (c.received < head_len + body_len)
                return full ? reply(c, kBadRequest) : ERR_OK;  // wait for the body
            key_handler(c.request + head_len, body_len);
            return reply(c, kNoContent);
        }
        return reply(c, kNotFound);
    }

    err_t on_recv(void* arg, tcp_pcb* pcb, pbuf* p, err_t err)
    {
        auto* c = static_cast<Connection*>(arg);
        if (p == nullptr)
            return close(*c);  // the browser closed its end
        if (err != ERR_OK)
        {
            pbuf_free(p);
            return err;
        }
        tcp_recved(pcb, p->tot_len);

        if (c->state != State::kReading)
        {
            pbuf_free(p);  // nothing more is expected once a request is answered
            return ERR_OK;
        }
        const size_t room = kRequestBytes - 1 - c->received;
        c->received += pbuf_copy_partial(p, c->request + c->received, static_cast<u16_t>(std::min<size_t>(room, p->tot_len)), 0);
        c->request[c->received] = '\0';
        c->idle_polls = 0;
        pbuf_free(p);
        return handle_request(*c);
    }

    err_t on_sent(void* arg, tcp_pcb*, u16_t len)
    {
        auto* c = static_cast<Connection*>(arg);
        c->unacked -= len;
        if (c->state == State::kReplying && c->unacked == 0)
            return close(*c);
        if (c->state == State::kStreaming)
            pump(*c);
        return ERR_OK;
    }

    err_t on_poll(void* arg, tcp_pcb*)
    {
        auto* c = static_cast<Connection*>(arg);
        if (c->state == State::kReading && ++c->idle_polls >= kRequestTimeoutPolls)
            return close(*c);
        if (c->state == State::kStreaming)
            pump(*c);
        return ERR_OK;
    }

    void on_error(void* arg, err_t)
    {
        // lwIP has already freed the pcb.
        auto* c = static_cast<Connection*>(arg);
        if (c == nullptr)
            return;
        c->pcb = nullptr;
        close(*c);
    }

    err_t on_accept(void*, tcp_pcb* pcb, err_t err)
    {
        if (err != ERR_OK || pcb == nullptr)
            return ERR_VAL;

        Connection* c = std::find_if(std::begin(connections), std::end(connections),
                                     [](const Connection& x) { return x.state == State::kFree; });
        if (c == std::end(connections))
        {
            tcp_abort(pcb);
            return ERR_ABRT;
        }

        c->state = State::kReading;
        c->pcb = pcb;
        c->unacked = 0;
        c->idle_polls = 0;
        c->received = 0;
        c->request[0] = '\0';

        tcp_arg(pcb, c);
        tcp_recv(pcb, on_recv);
        tcp_sent(pcb, on_sent);
        tcp_err(pcb, on_error);
        tcp_poll(pcb, on_poll, kPollInterval);
        tcp_nagle_disable(pcb);  // key replies and frame tails go out immediately
        return ERR_OK;
    }
}  // namespace

void http::init(KeyHandler on_key)
{
    key_handler = on_key;
    tcp_pcb* pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, kPort);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, on_accept);
}

bool http::stream_ready()
{
    return stream != nullptr && stream->part_header_queued && stream->body_left == 0 && stream->trailer_queued;
}

void http::stream_send(const uint8_t* png, size_t size)
{
    if (!stream_ready())
        return;
    Connection& c = *stream;
    c.part_header_len = static_cast<size_t>(
        std::snprintf(c.part_header, sizeof(c.part_header), kPartHeader, static_cast<unsigned>(size)));
    c.part_header_queued = false;
    c.body = png;
    c.body_left = size;
    c.trailer_queued = false;
    pump(c);
}

uint32_t http::take_frames_sent()
{
    const uint32_t n = frames_sent;
    frames_sent = 0;
    return n;
}
