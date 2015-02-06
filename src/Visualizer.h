#pragma once

#include <QWidget>
#include <QPainter>
#include "Arm.h"
#include "Search.h"

class Visualizer : public QWidget
{
Q_OBJECT

public:
    Visualizer(Arm& arm, target_t& goal, QWidget* parent = 0);

public slots:
    void heuristicOn(bool on);
    void newPlan();

private:
    void paintEvent(QPaintEvent*);

    void drawArm(Arm& a, QPainter* p, bool main);
    void drawTarget(QPainter* p);
    void drawHeuristic(QPainter* p);

    Arm& arm;
    target_t& goal;
    plan latest_plan;
    bool draw_heuristic;
    bool draw_plan;
    QTransform original, arm_base;
};
