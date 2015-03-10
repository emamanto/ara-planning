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
    Search<box, primitive> s;

    std::vector<primitive> ps;
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            if (i == 0 && j == 0) continue;
            ps.push_back(std::make_pair(i, j));
        }
    }

#ifdef ASTAR
    desired_epsilons.push_back(3.5);
    desired_epsilons.push_back(1.5);
    desired_epsilons.push_back(1.0);

    for (std::vector<float>::iterator e = desired_epsilons.begin();
         e != desired_epsilons.end(); e++)
    {
        solutions.push_back(s.astar(box(0, 0), ps, ps, *e));
    }
#else
    float epsilon = 3.5;
    std::vector<search_result<box, primitive> > all_solutions =
        s.arastar(box(0, 0), ps, ps, epsilon);

    epsilon += 0.5;
    for (std::vector<search_result<box, primitive> >::iterator s =
             all_solutions.begin(); s != all_solutions.end(); s++)
    {
        epsilon -= 0.5;
        if (s->expanded.size() == 0) continue;
        solutions.push_back(*s);
        desired_epsilons.push_back(epsilon);
    }
#endif

    setFixedSize(desired_epsilons.size()*(6*BOX_WIDTH + 10),
                 7*BOX_HEIGHT + 50);
}

void MazeWidget::paintEvent(QPaintEvent*)
{
     QPainter painter(this);


     for (int s = 0; s < solutions.size(); s++)
     {
         search_result<box, primitive> solution = solutions.at(s);

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
         for(std::set<box>::iterator i = solution.inconsistent.begin();
             i != solution.inconsistent.end(); i++)
         {
             painter.drawPoint(i->x*BOX_WIDTH + BOX_WIDTH/2,
                               i->y*BOX_HEIGHT + BOX_HEIGHT/2);
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
         {
             for (int j = 0; j <= 6; j++)
             {
                 if (box(i, j).valid()) continue;
                 painter.fillRect(i*BOX_WIDTH, j*BOX_HEIGHT,
                                  BOX_WIDTH, BOX_HEIGHT, Qt::black);
             }
         }

         box past = box(0, 0);
         p = QPen(Qt::cyan);
         p.setWidth(3);
         painter.setPen(p);
         for(std::vector<primitive>::iterator p = solution.path.begin();
             p != solution.path.end(); p++)
         {
             box current = past.apply(*p);
             painter.drawLine(past.x*BOX_WIDTH + (BOX_WIDTH/2),
                              past.y*BOX_HEIGHT + (BOX_HEIGHT/2),
                              current.x*BOX_WIDTH + (BOX_WIDTH/2),
                              current.y*BOX_HEIGHT + (BOX_HEIGHT/2));
             past.x = current.x;
             past.y = current.y;
         }

         painter.translate(6*BOX_WIDTH + 10, 0);
     }
}

