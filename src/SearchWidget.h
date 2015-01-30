#pragma once

#include "ArmControlPanel.h"
#include "Visualizer.h"

class SearchWidget : public QWidget
{
Q_OBJECT

public:
    SearchWidget(QWidget* parent = 0);

private:
    Arm arm;
    ArmControlPanel controls;
    Visualizer vis;
};
