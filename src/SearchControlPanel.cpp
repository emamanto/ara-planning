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

    QLabel* xerrlabel =  new QLabel(tr("x+/-"), this);
    xerrbox = new QSpinBox(this);
    xerrbox->setRange(0, 50);
    xerrbox->setSingleStep(5);
    xerrbox->setValue(0);
    layout->addWidget(xerrlabel, 1, 0);
    layout->addWidget(xerrbox, 1, 1);

    connect(xerrbox, SIGNAL(valueChanged(int)), this,
            SLOT(updateTarget()));

    QLabel* ylabel =  new QLabel(tr("target y"), this);
    ybox = new QSpinBox(this);
    ybox->setRange(0, ARM_LENGTH);
    ybox->setSingleStep(10);
    ybox->setValue(goal->y);
    layout->addWidget(ylabel, 2, 0);
    layout->addWidget(ybox, 2, 1);

    connect(ybox, SIGNAL(valueChanged(int)), this,
            SLOT(updateTarget()));

    QLabel* yerrlabel =  new QLabel(tr("y+/-"), this);
    yerrbox = new QSpinBox(this);
    yerrbox->setRange(0, 50);
    yerrbox->setSingleStep(5);
    yerrbox->setValue(0);
    layout->addWidget(yerrlabel, 3, 0);
    layout->addWidget(yerrbox, 3, 1);

    connect(yerrbox, SIGNAL(valueChanged(int)), this,
            SLOT(updateTarget()));

    QPushButton* start = new QPushButton(tr("search"), this);
    layout->addWidget(start, 4, 0);
    connect(start, SIGNAL(clicked()), this,
            SLOT(startSearch()));

    QPushButton* clear = new QPushButton(tr("clear"), this);
    layout->addWidget(clear, 4, 1);
    connect(clear, SIGNAL(clicked()), this,
            SLOT(clearSearch()));

    QCheckBox* heur = new QCheckBox(tr("heuristic"), this);
    layout->addWidget(heur, 5, 0);

    connect(heur, SIGNAL(stateChanged(int)), this,
            SLOT(heuristic(int)));
}

void SearchControlPanel::updateTarget()
{
    goal->x = xbox->value();
    goal->y = ybox->value();
    goal->err_x = xerrbox->value();
    goal->err_y = yerrbox->value();
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
