
#ifndef DRAW_H
#define DRAW_H

#include <QObject>
#include <QWidget>
#include <QPainter>

class Draw : public QWidget
{
    Q_OBJECT
public:
    Draw(QWidget *parent = 0);
    void setShape(int x, int y, int w, int h);
    void redraw(float r, float g, float b);

private:
    float r,g,b;
    int x, y, w, h;
    void paintEvent(QPaintEvent* );

};

#endif // DRAW_H
