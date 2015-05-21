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
    void killSearch();
    void shortcutCurrentPlan();

public slots:
    void searchOver();

private slots:
    void updateTarget();
    void heuristicDebug(int);
    void eePathOnly(int);
    void heuristicEuclidean(bool);
    void shortcutPlan();
    void startSearch();
    void clearSearch();
    void stopSearch();

private:
    target* goal;
    QSpinBox* xbox;
    QSpinBox* ybox;
    QPushButton* start;
    QPushButton* stop;
    QPushButton* clear;
    QPushButton* shorten;
    QRadioButton* eu;
    QRadioButton* bfs;
    QCheckBox* heur;
    QCheckBox* ee;
};
