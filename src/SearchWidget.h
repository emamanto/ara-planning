#pragma once

#include "ArmControlPanel.h"
#include "SearchControlPanel.h"
#include "Visualizer.h"

class SearchWidget : public QWidget
{
Q_OBJECT

public:
    SearchWidget(QWidget* parent = 0);

private:
    Arm arm;
    ArmControlPanel armControls;
    Search search;
    SearchControlPanel searchControls;
    Visualizer vis;
};
