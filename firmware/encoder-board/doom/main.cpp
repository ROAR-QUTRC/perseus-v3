// main.cpp
//
// Core 0 runs the game; the display/input backend runs on core 1.

#include "engine/engine.hpp"
#include "platform.hpp"

int main()
{
    platform::init();
    engine_run();
}
