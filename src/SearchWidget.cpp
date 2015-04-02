#include "SearchWidget.h"

//#define TABLETOP

SearchWidget::SearchWidget(QWidget* parent) :
    QWidget(parent),
    arm(Arm::the_instance()),
    goal(target::the_instance()),
    obs(obstacles::the_instance()),
    armControls(arm, this),
    searchControls(goal, this),
    vis(arm, goal, obs, this)
{
    std::vector<obstacle> the_obstacles;
#ifdef TABLETOP
    the_obstacles.push_back(obstacle(-10, 280, 20, 180));
#else
    the_obstacles.push_back(obstacle(-50, 250, 30, 150));
    the_obstacles.push_back(obstacle(100, 100, 50, 20));
    the_obstacles.push_back(obstacle(160, 190, 90, 10));
    the_obstacles.push_back(obstacle(-200, 60, 10, 40));
#endif

// Nonuniform dimensionality demo
// the_obstacles.push_back(obstacle(150, 150, 30, 30));
    obs->init(the_obstacles);

    arm_state::new_goal(target::the_instance()->x,
                        target::the_instance()->y);

    QGridLayout* layout = new QGridLayout(this);
    layout->addWidget(&vis, 0, 0);
    layout->addWidget(&armControls, 0, 1);
    layout->addWidget(&searchControls, 0, 2);
    setLayout(layout);
    connect(&armControls, SIGNAL(redrawArm()),
            &vis, SLOT(repaint()));

    connect(&searchControls, SIGNAL(redrawTargetInfo()),
            &vis, SLOT(repaint()));
    connect(&searchControls, SIGNAL(drawHeuristic(bool)),
            &vis, SLOT(heuristicOn(bool)));
    connect(&searchControls, SIGNAL(drawOnlyEEPath(bool)),
            &vis, SLOT(eePath(bool)));
    connect(&searchControls, SIGNAL(initiateSearch()),
            &vis, SLOT(newPlan()));
    connect(&searchControls, SIGNAL(initiateSearch()),
            &armControls, SLOT(disable()));
    connect(&searchControls, SIGNAL(clearSearchVis()),
            &vis, SLOT(clearPlan()));
    connect(&searchControls, SIGNAL(clearSearchVis()),
            &armControls, SLOT(enable()));
    connect(&searchControls, SIGNAL(killSearch()),
            &vis, SLOT(stopSearch()));

    connect(&vis, SIGNAL(searchFinished()),
            &armControls, SLOT(synchronize()));
    connect(&vis, SIGNAL(searchFinished()),
            &searchControls, SLOT(searchOver()));
}

