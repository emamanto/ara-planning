/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */
// Modified by Lizzie Mamantov 4/2/15

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/config.h>
#include <iostream>

#include "RRTStarPlanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rrtplanners{

FetchMotionValidator::FetchMotionValidator(ompl::base::SpaceInformationPtr& sip)
    : ob::MotionValidator(sip)
{}

bool FetchMotionValidator::checkMotion(const ob::State* s1,
                                       const ob::State* s2) const
{
    const ob::RealVectorStateSpace::StateType *s1vec =
        s1->as<ob::RealVectorStateSpace::StateType>();
    pose start;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        start.push_back(s1vec->values[i]);
    }

    const ob::RealVectorStateSpace::StateType *s2vec =
        s2->as<ob::RealVectorStateSpace::StateType>();
    pose end;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
       end.push_back(s2vec->values[i]);
    }

    action act = subtract(end, start);
    arm_state as(start);

    return as.action_valid(act);
}

// This fxn isn't called by RRT-Connect or RRT*
// My implementation of the last valid state was just not
// working, but it looks like it doesn't matter.
bool FetchMotionValidator::checkMotion(const ob::State* s1,
                                       const ob::State* s2,
                                       std::pair<ob::State *, double> &lastValid) const
{
    bool valid = checkMotion(s1, s2);

    return valid;
}

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const ob::RealVectorStateSpace::StateType *vecstate =
        state->as<ob::RealVectorStateSpace::StateType>();

    pose p;
    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        p.push_back(vecstate->values[i]);
    }
    return arm_state(p).valid();
}

std::vector<pose> plan(pose b, pose e, float time_limit, bool is_rrtc)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(fetch_arm::get_num_joints()));

    // set the bounds for state space
    ob::RealVectorBounds bounds(fetch_arm::get_num_joints());

    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {
        bounds.setLow(i, fetch_arm::get_joint_min(i));
        bounds.setHigh(i, fetch_arm::get_joint_max(i));
    }

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);
    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    si->setMotionValidator(ob::MotionValidatorPtr(new FetchMotionValidator(si)));
    si->setup();

    ob::PlannerPtr rrt_planner;

    if (is_rrtc)
    {
        og::RRTConnect* rrtc = new og::RRTConnect(si);
        rrt_planner = ob::PlannerPtr(rrtc);
    }
    else
    {
        og::RRTstar* rrts = new og::RRTstar(si);
        rrt_planner = ob::PlannerPtr(rrts);
    }

    ss.setPlanner(rrt_planner);

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);

    std::cout << "RRT start pose: ";
    for (pose::iterator j = b.begin();
         j != b.end(); j++)
    {
        std::cout << *j << " ";
    }
    std::cout << std::endl;

    std::cout << "RRT end pose: ";
    for (pose::iterator j = e.begin();
         j != e.end(); j++)
    {
        std::cout << *j << " ";
    }
    std::cout << std::endl;

    for (int i = 0; i < fetch_arm::get_num_joints(); i++)
    {

        start[i] = b.at(i);
        goal[i] = e.at(i);
    }

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setup();
    //ss.print();

    rrt_planner->setProblemDefinition(ss.getProblemDefinition());

    // attempt to solve the problem within time limit
    ob::PlannerStatus solved = ss.solve(time_limit);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

    std::vector<pose> plan;
    for (int i = 0; i < ss.getSolutionPath().getStateCount(); i++)
    {
        const ob::RealVectorStateSpace::StateType* s =
            ss.getSolutionPath().getState(i)->as<ob::RealVectorStateSpace::StateType>();
        pose p;
        for (int i = 0; i < fetch_arm::get_num_joints(); i++)
        {
            p.push_back(s->values[i]);
        }
        plan.push_back(p);
    }
    return plan;
}
}
