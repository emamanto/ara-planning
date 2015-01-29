#include "Visualizer.h"

Visualizer::Visualizer(QWidget* parent) : QWidget(parent),
                                          arm(2)
{
    setFixedSize(500,400);
}

void Visualizer::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    // Black background
    painter.fillRect(0, 0,
                     maximumWidth(),
                     maximumHeight(),
                     Qt::black);

    // Set correct origin
    painter.translate(maximumWidth()/2,
                      maximumHeight() - 0.1*maximumHeight());
    painter.rotate(180);

    // Draw axes
    QPen pen = QPen(Qt::darkGray);
    pen.setWidth(2);

    painter.setPen(pen);
    painter.drawLine(-maximumWidth()/2, 0,
                     maximumWidth()/2, 0);
    painter.drawLine(0, -0.1*maximumHeight(),
                     0, 0.9*maximumHeight());

    QTransform original = painter.worldTransform();

    // Draw arm
    pen.setWidth(3);
    pen.setColor(Qt::lightGray);
    painter.setPen(pen);

    for (int i = 0; i < arm.get_num_joints(); i++)
    {
        painter.rotate(arm.get_joint(i));
        painter.drawLine(0, 0, 0,
                         (arm.get_component(i) * 350));
        painter.translate(0, (arm.get_component(i) * 350));
    }

    painter.setWorldTransform(original);
    pen.setWidth(8);
    pen.setColor(Qt::red);
    painter.setPen(pen);
    painter.drawPoint(0,0);
    pen.setColor(Qt::blue);
    painter.setPen(pen);

    for (int i = 0; i < arm.get_num_joints(); i++)
    {
        painter.rotate(arm.get_joint(i));
        painter.translate(0, (arm.get_component(i) * 350));
        painter.drawPoint(0,0);
    }
}
