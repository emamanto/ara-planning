#include "MazeWidget.h"
#include <iostream>

using namespace std;

#define BOX_WIDTH 21
#define BOX_HEIGHT 18
#define EPSILON_START 3.5f
#define ASTAR

MazeWidget::MazeWidget(QWidget* parent) : QWidget(parent)
{
    setWindowTitle("aMaze");
    desired_epsilons.push_back(3.5);
    desired_epsilons.push_back(1.5);
    desired_epsilons.push_back(1.0);

    Search<box> s;

    for (std::vector<float>::iterator e = desired_epsilons.begin();
         e != desired_epsilons.end(); e++)
    {
        solutions.push_back(s.astar(box(0, 0),
                                    box(5, 6), *e));
    }
    int num_valid_iterations = desired_epsilons.size();

    setFixedSize(num_valid_iterations*(6*BOX_WIDTH + 10),
                 7*BOX_HEIGHT + 50);
}

void MazeWidget::paintEvent(QPaintEvent*)
{
     QPainter painter(this);


     for (int s = 0; s < solutions.size(); s++)
     {
         search_result<box> solution = solutions.at(s);

         QPen p = QPen(Qt::black);
         p.setWidth(3);
         painter.setPen(p);

         for(std::set<box>::iterator i = solution.expanded.begin();
             i != solution.expanded.end(); i++)
         {
             painter.fillRect(i->x*BOX_WIDTH, i->y*BOX_HEIGHT,
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
         QString e = QString::number(desired_epsilons.at(s));
         painter.drawText(5, 8*BOX_HEIGHT + 5, QString("Expanded"));
         painter.drawText(100, 8*BOX_HEIGHT + 5, n_ex);

         painter.drawText(5, 8*BOX_HEIGHT + 20, QString("Epsilon"));
         painter.drawText(100, 8*BOX_HEIGHT + 20, e);

         for(int i = 0; i <= 5; i++)
             for (int j = 0; j <= 6; j++)
             {
                 if (box(i, j).valid()) continue;
                 painter.fillRect(i*BOX_WIDTH, j*BOX_HEIGHT,
                                  BOX_WIDTH, BOX_HEIGHT, Qt::black);
             }

         box past = box(0, 0);
         p = QPen(Qt::cyan);
         p.setWidth(3);
         painter.setPen(p);
         for(std::vector<box>::iterator s = solution.path.begin();
             s != solution.path.end(); s++)
         {
             painter.drawLine(past.x*BOX_WIDTH + (BOX_WIDTH/2),
                              past.y*BOX_HEIGHT + (BOX_HEIGHT/2),
                              s->x*BOX_WIDTH + (BOX_WIDTH/2),
                              s->y*BOX_HEIGHT + (BOX_HEIGHT/2));
             past.x = s->x;
             past.y = s->y;
         }

         painter.translate(6*BOX_WIDTH + 10, 0);
     }
}

