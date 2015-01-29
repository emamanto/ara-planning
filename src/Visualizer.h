#pragma once

#include <QWidget>
#include <QPainter>

class Visualizer : public QWidget
{
Q_OBJECT

public:
    Visualizer(QWidget* parent = 0);

private:
    void paintEvent(QPaintEvent*);
};
