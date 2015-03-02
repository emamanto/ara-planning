#pragma once

#include "Search.h"

#include <QWidget>
#include <QPainter>
#include <QPen>

class MazeWidget : public QWidget
{
    Q_OBJECT;

public:
    MazeWidget(QWidget* parent = 0);

private:
    void paintEvent(QPaintEvent*);
    maze_boxes obstacles;
    maze_solution solution;
};
