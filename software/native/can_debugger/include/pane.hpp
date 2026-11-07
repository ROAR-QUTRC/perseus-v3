#pragma once
#include <ncurses.h>

class Pane
{
public:
    Pane(int x, int y, int width, int height);

    int draw();
    int resize(int width, int height);
    int refresh();
    int clear();
    virtual int draw_content(WINDOW* window) = 0;

    void set_border_color_pair(int color_pair) { _border_color_pair = color_pair; }

    ~Pane();

private:
    WINDOW* _window;
    int _x;
    int _y;
    int _width;
    int _height;

    // cosmetics
    int _border_color_pair = 1;
};