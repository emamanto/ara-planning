#include "MazeWidget.h"

using namespace std;

#define BOX_WIDTH 21
#define BOX_HEIGHT 18

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
}

void MazeWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    for (int i = 0; i <= 6*BOX_WIDTH; i+=BOX_WIDTH)
    {
        painter.drawLine(i, 0, i, 7*BOX_HEIGHT);
    }
    for (int i = 0; i <= 7*18; i+=18)
    {
        painter.drawLine(0, i, 6*BOX_WIDTH, i);
    }

    for(vector<pair<int, int> >::iterator i = obstacles.begin();
            i != obstacles.end(); i++)
    {
        painter.fillRect(i->first*BOX_WIDTH, i->second*BOX_HEIGHT,
                         BOX_WIDTH, BOX_HEIGHT, Qt::black);
    }
}
