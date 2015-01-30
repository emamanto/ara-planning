#include "SearchWidget.h"

SearchWidget::SearchWidget(QWidget* parent) : QWidget(parent),
                                              arm(3),
                                              armControls(arm, this),
                                              search(arm),
                                              searchControls(search,
                                                             this),
                                              vis(arm, search, this)
{
    QGridLayout* layout = new QGridLayout(this);
    layout->addWidget(&vis, 0, 0);
    layout->addWidget(&armControls, 0, 1);
    layout->addWidget(&searchControls, 0, 2);
    setLayout(layout);
    connect(&armControls, SIGNAL(redrawArm()),
            &vis, SLOT(repaint()));
    connect(&searchControls, SIGNAL(redrawSearchInfo()),
            &vis, SLOT(repaint()));
}
