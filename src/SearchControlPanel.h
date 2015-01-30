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
    SearchControlPanel(Search& search, QWidget* parent = 0);

signals:
    void redrawSearchInfo();
    void drawHeuristic(bool);

private slots:
    void updateSearch();
    void heuristic(int);

private:
    Search& search;
    QSpinBox* xbox;
    QSpinBox* ybox;
};
