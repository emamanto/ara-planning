#pragma once

#include "MazeSearch.h"

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
    search_result<box> solution;
    std::vector<float> desired_epsilons;
};
