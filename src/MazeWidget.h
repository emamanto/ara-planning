#pragma once

#include "Search.h"

#include <QWidget>
#include <QPainter>

#include <vector>
#include <utility>

class MazeWidget : public QWidget
{
    Q_OBJECT;

public:
    MazeWidget(QWidget* parent = 0);

private:
    void paintEvent(QPaintEvent*);
    std::vector<std::pair<int, int> > obstacles;
};
