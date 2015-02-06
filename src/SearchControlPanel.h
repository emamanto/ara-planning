#pragma once

#include <QWidget>
#include <QSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include "Search.h"

class SearchControlPanel : public QWidget
{
Q_OBJECT

public:
    SearchControlPanel(target_t& goal, QWidget* parent = 0);

signals:
    void redrawTargetInfo();
    void drawHeuristic(bool);
    void initiateSearch();
    void clearSearchVis();

private slots:
    void updateTarget();
    void heuristic(int);
    void startSearch();
    void clearSearch();

private:
    target_t& goal;
    QSpinBox* xbox;
    QSpinBox* ybox;
};
