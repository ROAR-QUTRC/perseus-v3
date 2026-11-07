#include "panes/sniffer.hpp"

int SnifferPane::draw_content(WINDOW* window)
{
    using namespace hi_can;
    // group by address with newest packets at the top
    std::map<addressing::flagged_address_t, std::vector<std::pair<std::chrono::milliseconds, Packet>>> grouped_packets;

    for (const auto& [timestamp, packet] : _packets)
    {
        if (grouped_packets.find(packet.get_address()) == grouped_packets.end())
        {
            grouped_packets[packet.get_address()] = std::vector<std::pair<std::chrono::milliseconds, Packet>>();
        }
        grouped_packets[packet.get_address()].push_back({timestamp, packet});
    }

    int x = 1;
    int y = 1;
    auto now = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch());
    for (const auto& [address, packets] : grouped_packets)
    {
        if (y >= getmaxy(window) - 1)
            break;
        // assume first packet is the newest
        const auto& newest_packet = packets.back();
        const auto& data = newest_packet.second.get_data();

        std::string line = "";
        // access address book with a string of the address
        if (this->_address_book.contains(std::format("{:X}", address.address)))
        {
            const auto& entry = this->_address_book[std::format("{:X}", address.address)];
            if (entry.contains("name"))
            {
                line = std::format("{:10X} ({}) -> ", address.address, entry["name"].get<std::string>());
            }
            else
            {
                line = std::format("{:10X} -> ", address.address);
            }
        }
        else
        {
            line = std::format("{:10X} -> ", address.address);
        }

        // print address and data
        for (const auto byte : data)
        {
            line += std::format("{:02X} ", byte);
        }
        // more than 3 seconds ago, print the time
        if (now.count() - newest_packet.first.count() > 3.0)
        {
            // seconds.milliseconds
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(newest_packet.first);
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(newest_packet.first - seconds);
            line += std::format("({}.{:03})", seconds.count(), milliseconds.count());
        }
        mvprintw(y, x, "%s", line.c_str());
        y++;
    }

    return 0;
}

void SnifferPane::add_packet(const hi_can::Packet& packet)
{
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    _packets.push_back({now, packet});
}

size_t SnifferPane::get_packet_count() const
{
    return _packets.size();
}