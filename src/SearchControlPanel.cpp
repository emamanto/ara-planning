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

    QPushButton* start = new QPushButton(tr("search"), this);
    layout->addWidget(start, 4, 0);
    connect(start, SIGNAL(clicked()), this,
            SLOT(startSearch()));

    QPushButton* clear = new QPushButton(tr("clear"), this);
    layout->addWidget(clear, 4, 1);
    connect(clear, SIGNAL(clicked()), this,
            SLOT(clearSearch()));

    QRadioButton* eu = new QRadioButton(tr("euclidean"), this);
    QRadioButton* bfs = new QRadioButton(tr("bfs"), this);
    eu->setChecked(false);
    bfs->setChecked(true);
    layout->addWidget(eu, 5, 0);
    layout->addWidget(bfs, 5, 1);
    connect(eu, SIGNAL(toggled(bool)), this,
            SLOT(heuristicEuclidean(bool)));

    QCheckBox* heur = new QCheckBox(tr("heuristic"), this);
    layout->addWidget(heur, 6, 0);
    connect(heur, SIGNAL(stateChanged(int)), this,
            SLOT(heuristicDebug(int)));
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

void SearchControlPanel::heuristicEuclidean(bool check)
{
    arm_state::use_euclidean(check);
    emit(redrawTargetInfo());
}

void SearchControlPanel::startSearch()
{
    emit(initiateSearch());
}

void SearchControlPanel::clearSearch()
{
    emit(clearSearchVis());
}
