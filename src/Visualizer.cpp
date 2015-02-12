#include "Visualizer.h"

Visualizer::Visualizer(Arm& arm, target_t& goal, QWidget* parent) :
    QWidget(parent),
    arm(arm),
    goal(goal),
    latest_plan_start(arm),
    draw_heuristic(false),
    draw_plan(false)
{
    setFixedSize(ARM_LENGTH*2+40,ARM_LENGTH+20);

    original.translate(maximumWidth()/2,
                      maximumHeight() - 0.1*maximumHeight());
    original.rotate(180);
    original.scale(-1.0, 1.0);

    arm_base = original;
    arm_base.rotate(-90);
}

void Visualizer::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    // Black background
    painter.fillRect(0, 0,
                     maximumWidth(),
                     maximumHeight(),
                     Qt::black);


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

    if(draw_plan)
    {
        drawPlan(&painter);
    }

    // Draw main arm
    drawArm(arm, &painter, true);
    drawTarget(&painter);

    if(draw_heuristic)
    {
        drawHeuristic(&painter);
    }
}

void Visualizer::drawArm(Arm& a, QPainter* p, bool main)
{
    p->setWorldTransform(arm_base);

    QPen pen = QPen(Qt::lightGray);
    if (!main) pen.setColor(Qt::darkGray);
    pen.setWidth(3);
    p->setPen(pen);

    for (int i = 0; i < a.get_num_joints(); i++)
    {
        p->rotate(a.get_joint(i));
        p->drawLine(0, 0, 0,
                         (a.get_component(i)));
        p->translate(0, (a.get_component(i)));
    }

    p->setWorldTransform(arm_base);
    pen.setWidth(8);
    pen.setColor(Qt::cyan);
    p->setPen(pen);
    p->drawPoint(0,0);
    pen.setColor(Qt::darkMagenta);
    p->setPen(pen);
    QColor c(30, 150, 50);
    int interval = int((150.f - 50.f)/a.get_num_joints());

    for (int i = 0; i < a.get_num_joints(); i++)
    {
        if (!main)
        {
            c.setBlue(c.blue()+interval);
            c.setGreen(c.green()-interval);
            pen.setColor(c);
            p->setPen(pen);
        }

        p->rotate(a.get_joint(i));
        p->translate(0, (a.get_component(i)));
        p->drawPoint(0,0);
    }
}

void Visualizer::drawTarget(QPainter* p)
{
    // Draw search target
    p->setWorldTransform(original);
    QPen pen = QPen(Qt::darkGreen);
    pen.setWidth(2);
    p->setPen(pen);
    p->drawRect(goal.x-goal.err_x, goal.y-goal.err_y,
                2*goal.err_x, 2*goal.err_y);
    pen.setWidth(5);
    pen.setColor(Qt::green);
    p->setPen(pen);
    p->drawPoint(goal.x, goal.y);
}

void Visualizer::drawHeuristic(QPainter* p)
{
    QPen pen = QPen(Qt::yellow);
    pen.setWidth(2);
    pen.setStyle(Qt::DashLine);
    p->setPen(pen);

    p->drawLine(arm.get_ee_x(),
                     arm.get_ee_y(),
                     goal.x,
                     goal.y);

    float h = Search::the_instance()->euclidean_heuristic(arm, goal);
    QString s = QString::number(h);
    p->scale(1.0,-1.0);
    p->drawText(goal.x+5,
                     -goal.y, s);
}

void Visualizer::drawPlan(QPainter* p)
{
    pose joints = latest_plan_start.get_joints();
    drawArm(latest_plan_start, p, false);
    for (plan::iterator i = latest_plan.begin();
         i != latest_plan.end(); i++)
    {
        latest_plan_start.apply(*i);
        drawArm(latest_plan_start, p, false);
    }
    latest_plan_start.set_joints(joints);
}

void Visualizer::heuristicOn(bool on)
{
    draw_heuristic = on;
}

void Visualizer::newPlan()
{
    latest_plan_start = arm;
    latest_plan = Search::the_instance()->run_search(arm, goal);
    arm.apply(latest_plan);
    emit(synchronizeArmControls());
    draw_plan = true;
    repaint();
}

void Visualizer::clearPlan()
{
    latest_plan.clear();
    draw_plan = false;
    repaint();
}
