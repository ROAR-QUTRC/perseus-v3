#include <ncurses.h>
#include <signal.h>

#include <cmath>
#include <fstream>
#include <hi_can_raw.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "pane.hpp"
#include "panes/sniffer.hpp"

#define COLOR_ORANGE 8

using namespace hi_can;

void signal_handler(int signal);
nlohmann::json get_config_from_file(const std::string& filename);

static volatile bool running = true;

int main(int argc, const char* argv[])
{
    nlohmann::json address_book;
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config.json> [interface]" << std::endl;
        return 1;
    }
    else
    {
        // Read configuration from file
        address_book = get_config_from_file(argv[1]);
    }
    std::string interface_id = "vcan0";
    if (argc > 2)
    {
        interface_id = argv[2];
    }

    // handle CTRL+C cleanly:
    // https://stackoverflow.com/questions/1641182/how-can-i-catch-a-ctrl-c-event
    struct sigaction sigint_handler;
    sigint_handler.sa_handler = signal_handler;
    sigemptyset(&sigint_handler.sa_mask);
    sigint_handler.sa_flags = 0;
    sigaction(SIGINT, &sigint_handler, NULL);

    initscr();
    if (!has_colors() || start_color() != OK)
    {
        endwin();
        return 1;
    }

    // define colours
    init_color(COLOR_BLACK, 0, 0, 0);
    init_color(COLOR_ORANGE, 1000, 647, 0);
    init_pair(1, COLOR_ORANGE, COLOR_BLACK);
    // hide cursor
    curs_set(0);
    // configure input mode
    nodelay(stdscr, TRUE);
    noecho();

    SnifferPane sniffer_pane(0, 0, COLS / 2, LINES, address_book);
    sniffer_pane.set_border_color_pair(1);

    try
    {
        // initialize CAN interface
        RawCanInterface can_interface(interface_id);
        std::cout << "Opened CAN interface: " << interface_id << std::endl;
        PacketManager packet_manager(can_interface);

        addressing::filter_t filter{
            .address = 0x00000000,
            .mask = 0x00000000,
        };
        PacketManager::callback_config_t config{
            .data_callback = [&sniffer_pane](const Packet& packet)
            {
                sniffer_pane.add_packet(packet);
            },
        };
        packet_manager.set_callback(filter, config);

        int ch;
        while (running && (ch = getch()) != 'q')
        {
            if (ch == KEY_RESIZE)
            {
                sniffer_pane.resize(COLS / 2, LINES);
                sniffer_pane.clear();
            }

            packet_manager.handle();

            sniffer_pane.draw();
            sniffer_pane.refresh();

            // write to the bottom of the screen
            mvprintw(LINES - 1, 0, "Press 'q' to quit. Received %zu packets.", sniffer_pane.get_packet_count());
            refresh();

            napms(100);
        }
        endwin();
    }
    catch (const std::exception& e)
    {
        endwin();
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

void signal_handler(int signal)
{
    (void)signal;
    running = false;
}

nlohmann::json get_config_from_file(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open config file: " + filename);
    }

    nlohmann::json config;
    try
    {
        file >> config;
    }
    catch (const nlohmann::json::parse_error& e)
    {
        throw std::runtime_error("Error parsing JSON config file: " + std::string(e.what()));
    }

    return config;
}