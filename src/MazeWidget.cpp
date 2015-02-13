#include "MazeWidget.h"

using namespace std;

#define BOX_WIDTH 21
#define BOX_HEIGHT 18
#define EPSILON 1

MazeWidget::MazeWidget(QWidget* parent)
{
    setFixedSize(6*BOX_WIDTH, 7*BOX_HEIGHT + 50);
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

    solution = Search::the_instance()->maze_astar(obstacles,
                                                  EPSILON);
}

void MazeWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    for(maze_boxes::iterator i = solution.expanded.begin();
            i != solution.expanded.end(); i++)
    {
        painter.fillRect(i->first*BOX_WIDTH, i->second*BOX_HEIGHT,
                         BOX_WIDTH, BOX_HEIGHT, Qt::lightGray);
    }

    for (int i = 0; i <= 6*BOX_WIDTH; i+=BOX_WIDTH)
    {
        painter.drawLine(i, 0, i, 7*BOX_HEIGHT);
    }
    for (int i = 0; i <= 7*18; i+=18)
    {
        painter.drawLine(0, i, 6*BOX_WIDTH, i);
    }

    QString n_ex = QString::number(solution.expanded.size());
    QString e = QString::number(EPSILON);
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
    QPen p = QPen(Qt::cyan);
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
}
