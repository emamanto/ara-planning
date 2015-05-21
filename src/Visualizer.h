#pragma once

#include <pthread.h>
#include <QWidget>
#include <QPainter>
#include "Arm.h"
#include "ArmStates.h"
#include "Search.h"
#include "Shortcut.h"

typedef std::vector<search_result<arm_state, action> > arastar_solution;

class Visualizer : public QWidget
{
Q_OBJECT

public:
    Visualizer(Arm* arm, target* goal,
               obstacles* obs,
               QWidget* parent = 0);

signals:
    void searchFinished();

public slots:
    void heuristicOn(bool on);
    void eePath(bool on);
    void newPlan();
    void clearPlan();
    void stopSearch();
    void shortcutPlan();

private:
    void paintEvent(QPaintEvent*);

    void drawArm(Arm* a, QPainter* p, bool main);
    void drawTarget(QPainter* p);
    void drawHeuristic(QPainter* p);
    void drawPlan(QPainter* p);
    void drawObstacles(QPainter* p);
    void drawGrid(QPainter* p);

    static void* searchThread(void*);
    void planCompleted();

    Arm* arm;
    target* goal;
    obstacles* obs;
    pose latest_plan_start;
    arastar_solution latest_search;
    plan latest_plan;
    bool draw_heuristic;
    bool draw_plan;
    bool ee_only;
    bool kill_search;
    QTransform original, arm_base;

    static pthread_t search_thread;
};
