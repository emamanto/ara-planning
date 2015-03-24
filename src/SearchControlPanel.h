#pragma once

#include <QWidget>
#include <QSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QRadioButton>
#include "ArmStates.h"

class SearchControlPanel : public QWidget
{
Q_OBJECT

public:
    SearchControlPanel(target* goal, QWidget* parent = 0);

signals:
    void redrawTargetInfo();
    void drawHeuristic(bool);
    void drawOnlyEEPath(bool);
    void initiateSearch();
    void clearSearchVis();

private slots:
    void updateTarget();
    void heuristicDebug(int);
    void eePathOnly(int);
    void heuristicEuclidean(bool);
    void startSearch();
    void clearSearch();

private:
    target* goal;
    QSpinBox* xbox;
    QSpinBox* ybox;
};
