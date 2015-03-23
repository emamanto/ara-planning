#include "Visualizer.h"
#include <iostream>
//#define ASTAR

Visualizer::Visualizer(Arm* arm, target* goal, obstacles* obs,
                       Search<arm_state, action>& search,
                       QWidget* parent) :
    QWidget(parent),
    arm(arm),
    goal(goal),
    obs(obs),
    search(search),
    latest_plan_start(arm->get_joints()),
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

    drawGrid(&painter);

    painter.rotate(-90);
    QTransform arm_base = painter.worldTransform();

    if(draw_plan)
    {
        drawPlan(&painter);
    }

    // Draw main arm
    drawObstacles(&painter);
    drawArm(arm, &painter, true);
    drawTarget(&painter);

    if(draw_heuristic)
    {
        drawHeuristic(&painter);
    }
}

void Visualizer::drawArm(Arm* a, QPainter* p, bool main)
{
    p->setWorldTransform(arm_base);

    QPen pen = QPen(Qt::lightGray);
    if (!main) pen.setColor(Qt::darkGray);
    pen.setWidth(3);
    p->setPen(pen);

    for (int i = 0; i < a->get_num_joints(); i++)
    {
        p->rotate(a->get_joint(i));
        p->drawLine(0, 0, 0,
                         (a->get_component(i)));
        p->translate(0, (a->get_component(i)));
    }

    p->setWorldTransform(arm_base);
    pen.setWidth(8);
    pen.setColor(Qt::cyan);
    p->setPen(pen);
    p->drawPoint(0,0);
    pen.setColor(Qt::darkMagenta);
    p->setPen(pen);
    QColor c(30, 150, 50);
    int interval = int((150.f - 50.f)/a->get_num_joints());

    for (int i = 0; i < a->get_num_joints(); i++)
    {
        if (!main)
        {
            c.setBlue(c.blue()+interval);
            c.setGreen(c.green()-interval);
            pen.setColor(c);
            p->setPen(pen);
        }

        p->rotate(a->get_joint(i));
        p->translate(0, (a->get_component(i)));
        p->drawPoint(0,0);
    }
}

void Visualizer::drawTarget(QPainter* p)
{
    // Draw search target
    p->setWorldTransform(original);
    QPen pen = QPen(Qt::green);
    pen.setWidth(5);
    p->setPen(pen);
    p->drawPoint(goal->x, goal->y);
}

void Visualizer::drawHeuristic(QPainter* p)
{
    QPen pen = QPen(Qt::yellow);
    pen.setWidth(2);
    p->setPen(pen);
    p->setWorldTransform(original);

    arm_state curr_state(arm->get_joints());
    float h = curr_state.heuristic();

    if (arm_state::using_euclidean())
    {
        pen.setStyle(Qt::DashLine);
        p->setPen(pen);
        p->drawLine(arm->get_ee_x(),
                    arm->get_ee_y(),
                    goal->x,
                    goal->y);
    }
    else
    {
        search_path path = curr_state.heuristic_path();
        search_cell past = arm_state::make_cell(goal->x, goal->y);
        for (search_path::iterator i = path.begin();
             i != path.end(); i++)
        {
            p->drawPoint((i->first),
                         (i->second));
            past = *i;
        }
    }

    QString s = QString::number(h);
    p->scale(1.0,-1.0);
    p->drawText(goal->x+5,
                     -goal->y, s);
}

void Visualizer::drawPlan(QPainter* p)
{
    pose joints = arm->get_joints();
    arm->set_joints(latest_plan_start);
    drawArm(arm, p, false);
    for (plan::iterator i = latest_plan.begin();
         i != latest_plan.end(); i++)
    {
        arm->apply(*i);
        drawArm(arm, p, false);
    }
    arm->set_joints(joints);
}

void Visualizer::drawObstacles(QPainter* p)
{
    p->setTransform(original);
    std::vector<obstacle> o = obs->get_obstacles();

    for (std::vector<obstacle>::iterator i = o.begin();
         i != o.end(); i++)
    {
        p->fillRect(i->x, i->y, i->width, -i->height, Qt::darkGray);

        QPen pen = QPen(Qt::white);
        pen.setWidth(1);
        p->setPen(pen);
        p->drawRect(i->x, i->y, i->width, -i->height);
    }
}

void Visualizer::drawGrid(QPainter* p)
{
    QColor dark = QColor(30, 30, 30);
    QPen pen = QPen(dark);
    p->setTransform(original);

    for (int w = 0; w < maximumWidth()/2; w += grid_size)
    {
        if ((w % 50) == 0) pen.setWidth(3);
        else pen.setWidth(1);
        p->setPen(pen);
        p->drawLine(w, 0, w, 0.9*maximumHeight());
        p->drawLine(-w, 0, -w, 0.9*maximumHeight());
    }

    for (int h = 0; h < maximumHeight(); h += grid_size)
    {
        if ((h % 50) == 0) pen.setWidth(3);
        else pen.setWidth(1);
        p->setPen(pen);
        p->drawLine(-maximumWidth()/2, h, maximumWidth()/2, h);
    }
}

void Visualizer::heuristicOn(bool on)
{
    draw_heuristic = on;
}

void Visualizer::newPlan()
{
    latest_plan_start = arm->get_joints();
    arm_state::new_goal(target::the_instance()->x,
                        target::the_instance()->y);
#ifdef ASTAR
    search_result<arm_state, action> final =
        search.astar(arm_state(arm->get_joints()),
                     arm->get_big_primitives(),
                     arm->get_small_primitives(),
                     5.f);
#else
    std::vector<search_result<arm_state, action> > res =
        search.arastar(arm_state(arm->get_joints()),
                       arm->get_big_primitives(),
                       arm->get_small_primitives(),
                       5.f);

    search_result<arm_state, action> final = res.at(res.size() - 1);
#endif

    latest_plan = final.path;
    arm->set_joints(latest_plan_start);
    arm->apply(latest_plan);
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
