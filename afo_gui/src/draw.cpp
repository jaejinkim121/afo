
#include "draw.h"

Draw::Draw(QWidget *parent) : QWidget(parent)
{
    r = 255;
    g = 255;
    b = 255;
    x = 0;
    y = 0;
    w = 0;
    h = 0;
}

void Draw::paintEvent(QPaintEvent*){
    QPainter p(this);
    QPen pen;
    pen.setWidth(0);
    p.setPen(pen);
    p.setBrush(Qt::SolidPattern);
    p.setBrush(QColor::fromRgbF(r, g, b));
    p.drawEllipse(QPoint(150, 200), 50, 50);
}

void Draw::redraw(float r, float g, float b){
    this->r = r;
    this->g = g;
    this->b = b;

    repaint();
}

void Draw::setShape(int x, int y, int w, int h){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
}
