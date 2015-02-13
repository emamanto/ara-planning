#include "MazeWidget.h"

MazeWidget::MazeWidget(QWidget* parent)
{
    setFixedSize(6*21, 7*18 + 50);
    setWindowTitle("aMaze");
}

void MazeWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    for (int i = 0; i <= 6*21; i+=21)
    {
        painter.drawLine(i, 0, i, 7*18);
    }
    for (int i = 0; i <= 7*18; i+=18)
    {
        painter.drawLine(0, i, 6*21, i);
    }
}
