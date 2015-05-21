#include "SearchControlPanel.h"

SearchControlPanel::SearchControlPanel(target* goal,
                                       QWidget* parent) :
    QWidget(parent),
    goal(goal)
{
    QGridLayout* layout = new QGridLayout(this);

    QLabel* xlabel =  new QLabel(tr("target x"), this);
    xbox = new QSpinBox(this);
    xbox->setRange(-ARM_LENGTH, ARM_LENGTH);
    xbox->setSingleStep(10);
    xbox->setValue(goal->x);
    layout->addWidget(xlabel, 0, 0);
    layout->addWidget(xbox, 0, 1);

    connect(xbox, SIGNAL(valueChanged(int)), this,
            SLOT(updateTarget()));

    QLabel* ylabel =  new QLabel(tr("target y"), this);
    ybox = new QSpinBox(this);
    ybox->setRange(0, ARM_LENGTH);
    ybox->setSingleStep(10);
    ybox->setValue(goal->y);
    layout->addWidget(ylabel, 1, 0);
    layout->addWidget(ybox, 1, 1);

    connect(ybox, SIGNAL(valueChanged(int)), this,
            SLOT(updateTarget()));

    start = new QPushButton(tr("search"), this);
    layout->addWidget(start, 4, 0);
    connect(start, SIGNAL(clicked()), this,
            SLOT(startSearch()));

    stop = new QPushButton(tr("stop"), this);
    layout->addWidget(stop, 4, 1);
    connect(stop, SIGNAL(clicked()), this,
            SLOT(stopSearch()));

    clear = new QPushButton(tr("clear"), this);
    layout->addWidget(clear, 5, 0);
    connect(clear, SIGNAL(clicked()), this,
            SLOT(clearSearch()));

    shorten = new QPushButton(tr("shortcut"), this);
    layout->addWidget(shorten, 5, 1);
    connect(shorten, SIGNAL(clicked()), this,
            SLOT(shortcutPlan()));

    eu = new QRadioButton(tr("euclidean"), this);
    bfs = new QRadioButton(tr("bfs"), this);
    eu->setChecked(false);
    bfs->setChecked(true);
    layout->addWidget(eu, 6, 0);
    layout->addWidget(bfs, 6, 1);
    connect(eu, SIGNAL(toggled(bool)), this,
            SLOT(heuristicEuclidean(bool)));

    heur = new QCheckBox(tr("heuristic"), this);
    layout->addWidget(heur, 7, 0);
    connect(heur, SIGNAL(stateChanged(int)), this,
            SLOT(heuristicDebug(int)));
    ee = new QCheckBox(tr("ee path"), this);
    layout->addWidget(ee, 7, 1);
    connect(ee, SIGNAL(stateChanged(int)), this,
            SLOT(eePathOnly(int)));
}

void SearchControlPanel::updateTarget()
{
    goal->x = xbox->value();
    goal->y = ybox->value();
    arm_state::new_goal(goal->x, goal->y);
    emit(redrawTargetInfo());
}

void SearchControlPanel::heuristicDebug(int check)
{
    arm_state::debug(bool(check));
    emit(drawHeuristic(bool(check)));
    emit(redrawTargetInfo());
}

void SearchControlPanel::eePathOnly(int check)
{
    emit(drawOnlyEEPath(bool(check)));
    emit(redrawTargetInfo());
}

void SearchControlPanel::heuristicEuclidean(bool check)
{
    arm_state::use_euclidean(check);
    emit(redrawTargetInfo());
}

void SearchControlPanel::shortcutPlan()
{
    emit(shortcutCurrentPlan());
    emit(redrawTargetInfo());
}

void SearchControlPanel::startSearch()
{
    xbox->setEnabled(false);
    ybox->setEnabled(false);
    start->setEnabled(false);
    stop->setEnabled(true);
    clear->setEnabled(false);
    shorten->setEnabled(false);
    eu->setEnabled(false);
    bfs->setEnabled(false);
    heur->setEnabled(false);
    ee->setEnabled(false);

    emit(initiateSearch());
}

void SearchControlPanel::clearSearch()
{
    xbox->setEnabled(true);
    ybox->setEnabled(true);
    start->setEnabled(true);
    stop->setEnabled(false);
    clear->setEnabled(false);
    shorten->setEnabled(false);
    eu->setEnabled(true);
    bfs->setEnabled(true);
    heur->setEnabled(true);
    ee->setEnabled(true);

    emit(clearSearchVis());
}

void SearchControlPanel::stopSearch()
{
    xbox->setEnabled(false);
    ybox->setEnabled(false);
    start->setEnabled(false);
    stop->setEnabled(false);
    clear->setEnabled(true);
    shorten->setEnabled(true);
    eu->setEnabled(false);
    bfs->setEnabled(false);
    heur->setEnabled(true);
    ee->setEnabled(true);

    emit(killSearch());
    emit(redrawTargetInfo());
}

void SearchControlPanel::searchOver()
{
    xbox->setEnabled(false);
    ybox->setEnabled(false);
    start->setEnabled(false);
    stop->setEnabled(false);
    clear->setEnabled(true);
    shorten->setEnabled(true);
    eu->setEnabled(false);
    bfs->setEnabled(false);
    heur->setEnabled(true);
    ee->setEnabled(true);
}
