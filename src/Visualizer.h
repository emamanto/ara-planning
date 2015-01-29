#pragma once

#include <QWidget>
#include <QPainter>
#include "Arm.h"

//#define RAD_TO_DEG 180.f/PI
//#define DEG_TO_RAD PI/180.f

class Visualizer : public QWidget
{
Q_OBJECT

public:
    Visualizer(QWidget* parent = 0);

private:
    void paintEvent(QPaintEvent*);

    Arm arm;
};
