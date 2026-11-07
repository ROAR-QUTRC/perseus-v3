#include "pane.hpp"

Pane::Pane(int x, int y, int width, int height)
    : _x(x),
      _y(y),
      _width(width),
      _height(height)
{
    this->_window = newwin(this->_height, this->_width, this->_y, this->_x);
}

int Pane::resize(int width, int height)
{
    this->_width = width;
    this->_height = height;
    return wresize(this->_window, this->_height, this->_width);
}

int Pane::draw()
{
    int result = 0;
    wattron(this->_window, COLOR_PAIR(this->_border_color_pair));
    box(this->_window, ACS_VLINE, ACS_HLINE);
    wattroff(this->_window, COLOR_PAIR(this->_border_color_pair));
    this->draw_content(this->_window);

    return result;
}

int Pane::refresh()
{
    return wrefresh(this->_window);
}

int Pane::clear()
{
    return wclear(this->_window);
}

Pane::~Pane()
{
    delwin(this->_window);
}