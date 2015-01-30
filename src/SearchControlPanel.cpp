#include "SearchControlPanel.h"

SearchControlPanel::SearchControlPanel(Search& search,
                                       QWidget* parent) :
    QWidget(parent),
    search(search)
{
    QGridLayout* layout = new QGridLayout(this);

    QLabel* xlabel =  new QLabel(tr("target x"), this);
    xbox = new QSpinBox(this);
    xbox->setRange(-ARM_LENGTH, ARM_LENGTH);
    xbox->setSingleStep(10);
    xbox->setValue(search.get_target_x());
    layout->addWidget(xlabel, 0, 0);
    layout->addWidget(xbox, 0, 1);

    connect(xbox, SIGNAL(valueChanged(int)), this,
            SLOT(updateSearch()));

    QLabel* ylabel =  new QLabel(tr("target y"), this);
    ybox = new QSpinBox(this);
    ybox->setRange(0, ARM_LENGTH);
    ybox->setSingleStep(10);
    ybox->setValue(search.get_target_y());
    layout->addWidget(ylabel, 1, 0);
    layout->addWidget(ybox, 1, 1);

    connect(ybox, SIGNAL(valueChanged(int)), this,
            SLOT(updateSearch()));

    QPushButton* start = new QPushButton(tr("start"), this);
    layout->addWidget(start, 2, 0);

    QCheckBox* heur = new QCheckBox(tr("heuristic"), this);
    layout->addWidget(heur, 3, 0);

    connect(heur, SIGNAL(stateChanged(int)), this,
            SLOT(heuristic(int)));
}

void SearchControlPanel::updateSearch()
{
    search.set_target(xbox->value(), ybox->value(), 0, 0);
    emit(redrawSearchInfo());
}

void SearchControlPanel::heuristic(int check)
{
    emit(drawHeuristic(bool(check)));
    emit(redrawSearchInfo());
}
