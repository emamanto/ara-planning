#pragma once

#include "Search.h"
#include "ArmStates.h"
#include "ArmControlPanel.h"
#include "SearchControlPanel.h"
#include "Visualizer.h"

class SearchWidget : public QWidget
{
Q_OBJECT

public:
    SearchWidget(QWidget* parent = 0);

private:
    Arm* arm;
    target* goal;
    obstacles* obs;
    ArmControlPanel armControls;
    SearchControlPanel searchControls;
    Visualizer vis;
};
