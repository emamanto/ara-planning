#include "SearchWidget.h"

SearchWidget::SearchWidget(QWidget* parent) : QWidget(parent),
                                              arm(3),
                                              goal(arm.get_ee_x(),
                                                   arm.get_ee_y(),
                                                   0, 0),
                                              armControls(arm, this),
                                              searchControls(goal, this),
                                              vis(arm, goal, this)
{
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
    connect(&searchControls, SIGNAL(initiateSearch()),
            &vis, SLOT(newPlan()));
    connect(&searchControls, SIGNAL(clearSearchVis()),
            &vis, SLOT(clearPlan()));
    connect(&vis, SIGNAL(synchronizeArmControls()),
            &armControls, SLOT(synchronize()));
}

