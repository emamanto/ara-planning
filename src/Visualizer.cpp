#include "Visualizer.h"

Visualizer::Visualizer(Arm& arm, target_t& goal, QWidget* parent) :
    QWidget(parent),
    arm(arm),
    goal(goal),
    draw_heuristic(false),
    draw_plan(false)
{
    setFixedSize(ARM_LENGTH*2+40,ARM_LENGTH+20);
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
    painter.scale(-1.0, 1.0);

    // Draw axes
    QPen pen = QPen(Qt::darkGray);
    pen.setWidth(2);

    painter.setPen(pen);
    painter.drawLine(0, 0,
                     maximumWidth()/2, 0);
    painter.drawLine(0, 0,
                     0, 0.9*maximumHeight());

    QTransform original = painter.worldTransform();

    painter.rotate(-90);
    QTransform arm_base = painter.worldTransform();

    // Draw arm
    pen.setWidth(3);
    pen.setColor(Qt::lightGray);
    painter.setPen(pen);

    for (int i = 0; i < arm.get_num_joints(); i++)
    {
        painter.rotate(arm.get_joint(i));
        painter.drawLine(0, 0, 0,
                         (arm.get_component(i)));
        painter.translate(0, (arm.get_component(i)));
    }

    painter.setWorldTransform(arm_base);
    pen.setWidth(8);
    pen.setColor(Qt::red);
    painter.setPen(pen);
    painter.drawPoint(0,0);
    pen.setColor(Qt::blue);
    painter.setPen(pen);

    for (int i = 0; i < arm.get_num_joints(); i++)
    {
        painter.rotate(arm.get_joint(i));
        painter.translate(0, (arm.get_component(i)));
        painter.drawPoint(0,0);
    }

    // Draw search target
    painter.setWorldTransform(original);
    pen.setWidth(5);
    pen.setColor(Qt::green);
    painter.setPen(pen);
    painter.drawPoint(goal.x, goal.y);

    if(draw_heuristic)
    {
        pen.setWidth(2);
        pen.setColor(Qt::yellow);
        pen.setStyle(Qt::DashLine);
        painter.setPen(pen);

        painter.drawLine(arm.get_ee_x(),
                         arm.get_ee_y(),
                         goal.x,
                         goal.y);

        float h = Search::the_instance()->euclidean_heuristic(arm, goal);
        QString s = QString::number(h);
        painter.scale(1.0,-1.0);
        painter.drawText(goal.x+5,
                         -goal.y, s);
    }
}

void Visualizer::heuristicOn(bool on)
{
    draw_heuristic = on;
}

void Visualizer::newPlan()
{
    latest_plan = Search::the_instance()->run_search(arm, goal);
}
