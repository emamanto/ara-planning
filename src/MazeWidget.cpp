#include "MazeWidget.h"
#include <iostream>

using namespace std;

#define BOX_WIDTH 21
#define BOX_HEIGHT 18
#define EPSILON_START 3.5f
//#define ASTAR

MazeWidget::MazeWidget(QWidget* parent) : QWidget(parent)
{
    setWindowTitle("aMaze");

    obstacles.push_back(make_pair(0,1));
    obstacles.push_back(make_pair(0,2));
    obstacles.push_back(make_pair(0,6));

    obstacles.push_back(make_pair(1,4));
    obstacles.push_back(make_pair(1,6));

    obstacles.push_back(make_pair(2,1));
    obstacles.push_back(make_pair(2,2));
    obstacles.push_back(make_pair(2,3));
    obstacles.push_back(make_pair(2,4));
    obstacles.push_back(make_pair(2,6));

    obstacles.push_back(make_pair(3,1));
    obstacles.push_back(make_pair(3,2));
    obstacles.push_back(make_pair(3,6));

    obstacles.push_back(make_pair(4,1));
    obstacles.push_back(make_pair(4,2));
    obstacles.push_back(make_pair(4,4));
    obstacles.push_back(make_pair(4,5));
    obstacles.push_back(make_pair(4,6));

#ifdef ASTAR
    desired_epsilons.push_back(3.5);
    desired_epsilons.push_back(1.5);
    desired_epsilons.push_back(1.0);

    for (std::vector<float>::iterator e = desired_epsilons.begin();
         e != desired_epsilons.end(); e++)
    {
        solutions.push_back(Search::the_instance()->maze_astar(obstacles, *e));
    }
#else
    solutions = Search::the_instance()->maze_arastar(obstacles,
                                                     EPSILON_START);
#endif

#ifdef ASTAR
    int num_valid_iterations = solutions.size();
#else
    int num_valid_iterations = 0;
    for (arastar_solution::iterator s = solutions.begin();
         s!= solutions.end(); s++)
    {
        if (!s->expanded.empty()) num_valid_iterations++;
    }
#endif

    setFixedSize(num_valid_iterations*(6*BOX_WIDTH + 10),
                 7*BOX_HEIGHT + 50);
}

void MazeWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    float epsilon = EPSILON_START + 0.5;

    for (int s = 0; s < solutions.size(); s++)
    {
#ifndef ASTAR
        epsilon -= 0.5;
#else
        epsilon = desired_epsilons.at(s);
#endif
        if (solutions.at(s).expanded.empty()) continue;
        QPen p = QPen(Qt::black);
        p.setWidth(3);
        painter.setPen(p);

        maze_solution solution = solutions.at(s);
        for(maze_boxes::iterator i = solution.expanded.begin();
            i != solution.expanded.end(); i++)
        {
            painter.fillRect(i->first*BOX_WIDTH, i->second*BOX_HEIGHT,
                             BOX_WIDTH, BOX_HEIGHT, Qt::darkGray);
        }

#ifndef ASTAR
        for(maze_boxes::iterator i = solution.incons.begin();
            i != solution.incons.end(); i++)
        {
            painter.drawPoint(i->first*BOX_WIDTH + BOX_WIDTH/2,
                              i->second*BOX_HEIGHT + BOX_HEIGHT/2);
        }
#endif

        p.setWidth(1);
        painter.setPen(p);

        for (int i = 0; i <= 6*BOX_WIDTH; i+=BOX_WIDTH)
        {
            painter.drawLine(i, 0, i, 7*BOX_HEIGHT);
        }
        for (int i = 0; i <= 7*18; i+=18)
        {
            painter.drawLine(0, i, 6*BOX_WIDTH, i);
        }

        QString n_ex = QString::number(solution.expanded.size());
        QString e = QString::number(epsilon);
        painter.drawText(5, 8*BOX_HEIGHT + 5, QString("Expanded"));
        painter.drawText(100, 8*BOX_HEIGHT + 5, n_ex);

        painter.drawText(5, 8*BOX_HEIGHT + 20, QString("Epsilon"));
        painter.drawText(100, 8*BOX_HEIGHT + 20, e);

        for(maze_boxes::iterator i = obstacles.begin();
            i != obstacles.end(); i++)
        {
            painter.fillRect(i->first*BOX_WIDTH, i->second*BOX_HEIGHT,
                             BOX_WIDTH, BOX_HEIGHT, Qt::black);
        }

        box past = std::make_pair(0, 0);
        p = QPen(Qt::cyan);
        p.setWidth(3);
        painter.setPen(p);
        for(maze_boxes::iterator s = solution.path.begin();
            s != solution.path.end(); s++)
        {
            painter.drawLine(past.first*BOX_WIDTH + (BOX_WIDTH/2),
                             past.second*BOX_HEIGHT + (BOX_HEIGHT/2),
                             s->first*BOX_WIDTH + (BOX_WIDTH/2),
                             s->second*BOX_HEIGHT + (BOX_HEIGHT/2));
            past.first = s->first;
            past.second = s->second;
        }

        painter.translate(6*BOX_WIDTH + 10, 0);
    }
}
