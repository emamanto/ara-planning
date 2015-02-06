#include "SearchControlPanel.h"

SearchControlPanel::SearchControlPanel(target_t& goal,
                                       QWidget* parent) :
    QWidget(parent),
    goal(goal)
{
    QGridLayout* layout = new QGridLayout(this);

    QLabel* xlabel =  new QLabel(tr("target x"), this);
    xbox = new QSpinBox(this);
    xbox->setRange(-ARM_LENGTH, ARM_LENGTH);
    xbox->setSingleStep(10);
    xbox->setValue(goal.x);
    layout->addWidget(xlabel, 0, 0);
    layout->addWidget(xbox, 0, 1);

    connect(xbox, SIGNAL(valueChanged(int)), this,
            SLOT(updateTarget()));

    QLabel* ylabel =  new QLabel(tr("target y"), this);
    ybox = new QSpinBox(this);
    ybox->setRange(0, ARM_LENGTH);
    ybox->setSingleStep(10);
    ybox->setValue(goal.y);
    layout->addWidget(ylabel, 1, 0);
    layout->addWidget(ybox, 1, 1);

    connect(ybox, SIGNAL(valueChanged(int)), this,
            SLOT(updateTarget()));

    QPushButton* start = new QPushButton(tr("search"), this);
    layout->addWidget(start, 2, 0);
    connect(start, SIGNAL(clicked()), this,
            SLOT(startSearch()));

    QPushButton* clear = new QPushButton(tr("clear"), this);
    layout->addWidget(clear, 2, 1);
    connect(clear, SIGNAL(clicked()), this,
            SLOT(clearSearch()));

    QCheckBox* heur = new QCheckBox(tr("heuristic"), this);
    layout->addWidget(heur, 3, 0);

    connect(heur, SIGNAL(stateChanged(int)), this,
            SLOT(heuristic(int)));
}

void SearchControlPanel::updateTarget()
{
    goal.x = xbox->value();
    goal.y = ybox->value();
    emit(redrawTargetInfo());
}

void SearchControlPanel::heuristic(int check)
{
    emit(drawHeuristic(bool(check)));
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
