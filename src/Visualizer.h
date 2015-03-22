#pragma once

#include <QWidget>
#include <QPainter>
#include "Arm.h"
#include "ArmStates.h"
#include "Search.h"

class Visualizer : public QWidget
{
Q_OBJECT

public:
    Visualizer(Arm* arm, target* goal,
               obstacles* obs,
               Search<arm_state, action>& search,
               QWidget* parent = 0);

signals:
    void synchronizeArmControls();

public slots:
    void heuristicOn(bool on);
    void newPlan();
    void clearPlan();

private:
    void paintEvent(QPaintEvent*);

    void drawArm(Arm* a, QPainter* p, bool main);
    void drawTarget(QPainter* p);
    void drawHeuristic(QPainter* p);
    void drawPlan(QPainter* p);
    void drawObstacles(QPainter* p);
    void drawGrid(QPainter* p);

    Arm* arm;
    target* goal;
    obstacles* obs;
    Search<arm_state, action>& search;
    pose latest_plan_start;
    plan latest_plan;
    bool draw_heuristic;
    bool draw_plan;
    QTransform original, arm_base;
};
