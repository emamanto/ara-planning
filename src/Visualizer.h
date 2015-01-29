#pragma once

#include <QWidget>
#include <QPainter>
#include "Arm.h"

class Visualizer : public QWidget
{
Q_OBJECT

public:
    Visualizer(QWidget* parent = 0);

private:
    void paintEvent(QPaintEvent*);

    Arm arm;
};
