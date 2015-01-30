#pragma once

#include <QWidget>
#include <QSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include "Search.h"

class SearchControlPanel : public QWidget
{
Q_OBJECT

public:
    SearchControlPanel(Search& search, QWidget* parent = 0);

signals:
    void redrawSearchInfo();

private slots:
    void updateSearch();

private:
    Search& search;
    QSpinBox* xbox;
    QSpinBox* ybox;
};
