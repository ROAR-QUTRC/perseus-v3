// png.cpp
//
// One zlib stream per frame, one fixed-Huffman deflate block. Matches come
// from a 3-byte hash plus two candidates that suit rendered frames: the
// previous byte (runs) and the byte one row up (vertical repeats).

#include "png.hpp"

#include <algorithm>
#include <cstring>
#include <iterator>

namespace
{
    constexpr uint16_t kLengthBase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27,
                                          31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
    constexpr uint8_t kLengthExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
    constexpr uint16_t kDistanceBase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129,
                                            193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
    constexpr uint8_t kDistanceExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

    constexpr size_t kMinMatch = 3;
    constexpr size_t kMaxMatch = 258;
    constexpr size_t kWindow = 32768;
    constexpr int kHashBits = 12;
    constexpr uint16_t kNoPosition = 0xFFFF;
    constexpr int kEndOfBlock = 256;

    // Deflate writes Huffman codes most-significant bit first into an
    // LSB-first stream, so the codes are stored pre-reversed.
    struct Tables
    {
        uint16_t literal_code[288];
        uint8_t literal_bits[288];
        uint8_t distance_code[30];
        uint32_t crc[256];
    };

    uint32_t reverse_bits(uint32_t code, int bits)
    {
        uint32_t reversed = 0;
        for (int i = 0; i < bits; ++i, code >>= 1)
            reversed = (reversed << 1) | (code & 1);
        return reversed;
    }

    const Tables& tables()
    {
        static Tables t;
        static bool ready = false;
        if (ready)
            return t;

        for (int symbol = 0; symbol < 288; ++symbol)
        {
            uint32_t code;
            int bits;
            if (symbol < 144)
                code = 0x30 + symbol, bits = 8;
            else if (symbol < 256)
                code = 0x190 + (symbol - 144), bits = 9;
            else if (symbol < 280)
                code = symbol - 256, bits = 7;
            else
                code = 0xC0 + (symbol - 280), bits = 8;
            t.literal_code[symbol] = static_cast<uint16_t>(reverse_bits(code, bits));
            t.literal_bits[symbol] = static_cast<uint8_t>(bits);
        }
        for (int symbol = 0; symbol < 30; ++symbol)
            t.distance_code[symbol] = static_cast<uint8_t>(reverse_bits(symbol, 5));
        for (uint32_t n = 0; n < 256; ++n)
        {
            uint32_t c = n;
            for (int k = 0; k < 8; ++k)
                c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t.crc[n] = c;
        }
        ready = true;
        return t;
    }

    uint16_t hash_table[1u << kHashBits];

    struct Output
    {
        uint8_t* data;
        size_t size;
        size_t pos = 0;
        bool overflow = false;

        void u8(uint8_t v)
        {
            if (pos < size)
                data[pos++] = v;
            else
                overflow = true;
        }
        void u32(uint32_t v)
        {
            u8(static_cast<uint8_t>(v >> 24));
            u8(static_cast<uint8_t>(v >> 16));
            u8(static_cast<uint8_t>(v >> 8));
            u8(static_cast<uint8_t>(v));
        }
        void bytes(const void* src, size_t len)
        {
            if (size - pos < len)
            {
                overflow = true;
                return;
            }
            std::memcpy(data + pos, src, len);
            pos += len;
        }
    };

    class BitWriter
    {
    public:
        explicit BitWriter(Output& out)
            : out_(out)
        {
        }

        void put(uint32_t value, int bits)
        {
            buffer_ |= value << count_;
            count_ += bits;
            while (count_ >= 8)
            {
                out_.u8(static_cast<uint8_t>(buffer_));
                buffer_ >>= 8;
                count_ -= 8;
            }
        }
        void flush()
        {
            if (count_ > 0)
                out_.u8(static_cast<uint8_t>(buffer_));
            buffer_ = 0;
            count_ = 0;
        }

    private:
        Output& out_;
        uint32_t buffer_ = 0;
        int count_ = 0;
    };

    uint32_t hash3(const uint8_t* p)
    {
        const uint32_t v = (static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | p[2];
        return (v * 2654435761u) >> (32 - kHashBits);
    }

    size_t match_length(const uint8_t* data, size_t from, size_t at, size_t limit)
    {
        size_t n = 0;
        while (n < limit && data[from + n] == data[at + n])
            ++n;
        return n;
    }

    void put_literal(BitWriter& w, const Tables& t, int symbol) { w.put(t.literal_code[symbol], t.literal_bits[symbol]); }

    void put_match(BitWriter& w, const Tables& t, size_t length, size_t distance)
    {
        int code = 28;
        while (kLengthBase[code] > length)
            --code;
        put_literal(w, t, 257 + code);
        if (kLengthExtra[code])
            w.put(static_cast<uint32_t>(length - kLengthBase[code]), kLengthExtra[code]);

        int dcode = 29;
        while (kDistanceBase[dcode] > distance)
            --dcode;
        w.put(t.distance_code[dcode], 5);
        if (kDistanceExtra[dcode])
            w.put(static_cast<uint32_t>(distance - kDistanceBase[dcode]), kDistanceExtra[dcode]);
    }

    void deflate_fixed(const uint8_t* data, size_t n, size_t row_bytes, Output& out, const Tables& t)
    {
        BitWriter w(out);
        w.put(1, 1);  // BFINAL: the only block
        w.put(1, 2);  // BTYPE 01: fixed Huffman

        std::fill(std::begin(hash_table), std::end(hash_table), kNoPosition);

        size_t i = 0;
        while (i < n)
        {
            size_t best_length = 0;
            size_t best_distance = 0;

            if (i + kMinMatch <= n)
            {
                const size_t limit = std::min(kMaxMatch, n - i);
                auto consider = [&](size_t distance)
                {
                    if (distance == 0 || distance > i || distance > kWindow || best_length >= limit)
                        return;
                    const size_t from = i - distance;
                    // Cheap reject: to beat the best so far it must match at best_length too.
                    if (data[from + best_length] != data[i + best_length] || data[from] != data[i])
                        return;
                    const size_t length = match_length(data, from, i, limit);
                    if (length > best_length)
                    {
                        best_length = length;
                        best_distance = distance;
                    }
                };

                uint16_t& slot = hash_table[hash3(data + i)];
                if (slot != kNoPosition)
                    consider(i - slot);
                slot = static_cast<uint16_t>(i);
                consider(1);
                consider(row_bytes);
            }

            if (best_length >= kMinMatch)
            {
                put_match(w, t, best_length, best_distance);
                for (size_t k = i + 1; k < i + best_length && k + kMinMatch <= n; ++k)
                    hash_table[hash3(data + k)] = static_cast<uint16_t>(k);
                i += best_length;
            }
            else
            {
                put_literal(w, t, data[i]);
                ++i;
            }
        }

        put_literal(w, t, kEndOfBlock);
        w.flush();
    }

    uint32_t adler32(const uint8_t* data, size_t n)
    {
        constexpr uint32_t kModulus = 65521;
        constexpr size_t kBlock = 5552;  // largest block before the sums can overflow
        uint32_t a = 1;
        uint32_t b = 0;
        while (n > 0)
        {
            const size_t block = std::min(n, kBlock);
            for (size_t k = 0; k < block; ++k)
            {
                a += data[k];
                b += a;
            }
            a %= kModulus;
            b %= kModulus;
            data += block;
            n -= block;
        }
        return (b << 16) | a;
    }

    uint32_t crc32(const Tables& t, const uint8_t* data, size_t n)
    {
        uint32_t c = 0xFFFFFFFFu;
        for (size_t k = 0; k < n; ++k)
            c = t.crc[(c ^ data[k]) & 0xFF] ^ (c >> 8);
        return c ^ 0xFFFFFFFFu;
    }

    size_t begin_chunk(Output& out, const char* type)
    {
        const size_t start = out.pos;
        out.u32(0);  // length, patched by end_chunk()
        out.bytes(type, 4);
        return start;
    }

    void end_chunk(Output& out, size_t start, const Tables& t)
    {
        if (out.overflow)
            return;
        const uint32_t length = static_cast<uint32_t>(out.pos - start - 8);
        out.data[start + 0] = static_cast<uint8_t>(length >> 24);
        out.data[start + 1] = static_cast<uint8_t>(length >> 16);
        out.data[start + 2] = static_cast<uint8_t>(length >> 8);
        out.data[start + 3] = static_cast<uint8_t>(length);
        out.u32(crc32(t, out.data + start + 4, length + 4));  // over type and data
    }
}  // namespace

size_t png::encode(const uint8_t* scanlines, int width, int height, const uint8_t* palette_rgb, uint8_t* out_buf,
                   size_t out_size)
{
    const size_t row_bytes = static_cast<size_t>(width) + 1;
    const size_t n = scanlines_size(width, height);
    if (n >= kNoPosition)
        return 0;  // positions are stored as uint16_t

    const Tables& t = tables();
    Output out{out_buf, out_size};

    static constexpr uint8_t kSignature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    out.bytes(kSignature, sizeof(kSignature));

    size_t chunk = begin_chunk(out, "IHDR");
    out.u32(static_cast<uint32_t>(width));
    out.u32(static_cast<uint32_t>(height));
    out.u8(8);  // bits per index
    out.u8(3);  // colour type: palette
    out.u8(0);  // deflate
    out.u8(0);  // adaptive filtering (every row uses filter 0)
    out.u8(0);  // no interlace
    end_chunk(out, chunk, t);

    chunk = begin_chunk(out, "PLTE");
    out.bytes(palette_rgb, 256 * 3);
    end_chunk(out, chunk, t);

    chunk = begin_chunk(out, "IDAT");
    out.u8(0x78);  // zlib: deflate, 32 KiB window
    out.u8(0x01);  // fastest level; makes the header a multiple of 31
    deflate_fixed(scanlines, n, row_bytes, out, t);
    out.u32(adler32(scanlines, n));
    end_chunk(out, chunk, t);

    chunk = begin_chunk(out, "IEND");
    end_chunk(out, chunk, t);

    return out.overflow ? 0 : out.pos;
}
