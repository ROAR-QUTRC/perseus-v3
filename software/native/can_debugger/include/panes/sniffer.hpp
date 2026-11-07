#pragma once
#include <chrono>
#include <hi_can_packet.hpp>
#include <map>
#include <nlohmann/json.hpp>
#include <vector>

#include "pane.hpp"

class SnifferPane : public Pane
{
public:
    SnifferPane(int x, int y, int width, int height, const nlohmann::json& address_book = {})
        : Pane(x, y, width, height),
          _address_book(address_book)
    {
    }

    int draw_content(WINDOW* window) override;

    void add_packet(const hi_can::Packet& packet);

    size_t get_packet_count() const;

private:
    nlohmann::json _address_book;
    std::vector<std::pair<std::chrono::milliseconds, hi_can::Packet>> _packets;
};