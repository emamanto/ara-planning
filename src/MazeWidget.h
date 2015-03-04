#pragma once

#include "Search.h"
#include "MazeStates.h"

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
    std::vector<search_result<box, primitive> > solutions;
    std::vector<float> desired_epsilons;
};
