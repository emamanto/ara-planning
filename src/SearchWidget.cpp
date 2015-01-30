#include "SearchWidget.h"

SearchWidget::SearchWidget(QWidget* parent) : QWidget(parent),
                                              arm(3),
                                              vis(arm),
                                              controls(arm)
{
    QGridLayout* layout = new QGridLayout(this);
    layout->addWidget(&vis, 0, 0);
    layout->addWidget(&controls, 0, 1);
    setLayout(layout);
    connect(&controls, SIGNAL(redrawArm()), &vis, SLOT(repaint()));
}
