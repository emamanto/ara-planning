#pragma once

#include <QWidget>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <map>
#include "Arm.h"
#include "ArmStates.h"

class ArmControlPanel : public QWidget
{
Q_OBJECT

public:
    ArmControlPanel(Arm* arm, QWidget* parent = 0);

signals:
    void redrawArm();

public slots:
    void synchronize();
    void disable();
    void enable();

private slots:
    void updateArm();
    void resetArm();

private:
    Arm* arm;
    std::map<int, QDoubleSpinBox*> jointMap;
};
