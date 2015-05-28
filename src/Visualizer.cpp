#include "Visualizer.h"
#include <iostream>
//#define ASTAR

pthread_t Visualizer::search_thread = pthread_t();

Visualizer::Visualizer(Arm* arm, target* goal, obstacles* obs,
                       QWidget* parent) :
    QWidget(parent),
    arm(arm),
    goal(goal),
    obs(obs),
    latest_plan_start(arm->get_joints()),
    draw_heuristic(false),
    draw_plan(false),
    ee_only(false),
    kill_search(false)
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
    if (!main) pen.setColor(QColor(100, 100, 100, 150));
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
    QColor c(30, 150, 50, 200);
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
    arm->set_joints(latest_plan_start);
    drawArm(arm, p, false);
    float past_x = arm->get_ee_x();
    float past_y = arm->get_ee_y();

    QColor step_color = QColor(80, 80, 100);
    int num_steps = latest_plan.size();
    int color_diff = int((255-100) / num_steps);

    p->setTransform(original);
    for (plan::iterator i = latest_plan.begin();
         i != latest_plan.end(); i++)
    {
        arm->apply(*i);
        if (!ee_only)
        {
            drawArm(arm, p, false);
        }
        else
        {
            QPen pen = QPen(step_color);
            pen.setWidth(3);
            p->setPen(pen);
            float x = arm->get_ee_x();
            float y = arm->get_ee_y();
            p->drawLine(past_x, past_y, x, y);
            step_color.setBlue(step_color.blue() + color_diff);
            past_x = x;
            past_y = y;
        }
    }
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

void Visualizer::eePath(bool on)
{
    ee_only = on;
}

void Visualizer::newPlan()
{
    kill_search = false;
    latest_plan_start = arm->get_joints();
    arm_state::new_goal(target::the_instance()->x,
                        target::the_instance()->y);
#ifdef ASTAR
    search_result<arm_state, action> final =
        astar<arm_state, action>(arm_state(arm->get_joints()),
                                 arm->get_big_primitives(),
                                 arm->get_small_primitives(),
                                 5.f);
#else
    latest_search.clear();
    pthread_create(&search_thread, NULL, &searchThread, this);
#endif
}

void* Visualizer::searchThread(void* arg)
{
    Visualizer* v = static_cast<Visualizer*>(arg);
    Arm* a = Arm::the_instance();
    arastar<arm_state, action>(&v->latest_search,
                               &v->kill_search,
                               arm_state(a->get_joints()),
                               a->get_big_primitives(),
                               a->get_small_primitives(),
                               10.f);
    v->planCompleted();
}

void Visualizer::planCompleted()
{
    if (latest_search.size() == 1 && latest_search.at(0).path.empty())
    {
        std::cout << "Planning failed." << std::endl;
        return;
    }
    latest_plan = latest_search.at(latest_search.size()-1).path;
    arm->set_joints(latest_plan_start);
    arm->apply(latest_plan);
    emit(searchFinished());
    draw_plan = true;
}

void Visualizer::clearPlan()
{
    latest_plan.clear();
    draw_plan = false;
    repaint();
}

void Visualizer::stopSearch()
{
    kill_search = true;
}

void Visualizer::shortcutPlan()
{
   std::cout << "Shortcutting plan! Originally: "
              << latest_plan.size() << std::endl;
   latest_plan =
       shortcut<arm_state, action>(latest_plan,
                                   arm_state(latest_plan_start));
    std::cout << "Ultimate path length: " << latest_plan.size()
              << std::endl;
}
