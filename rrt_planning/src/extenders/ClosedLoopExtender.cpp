/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rrt_planning/extenders/ClosedLoopExtender.h"

using namespace Eigen;
using namespace std;

namespace rrt_planning
{

ClosedLoopExtender::ClosedLoopExtender(KinematicModel& model, Controller& controller,
                                       Map& map, Distance& distance) :
    Extender(map, distance), model(model), controller(controller)
{
    deltaT = 0;
    loopN = 0;
    l2distance = new L2Distance();
    thetadistance = new ThetaDistance();
}

bool ClosedLoopExtender::compute(const VectorXd& x0, const VectorXd& xRand, VectorXd& xNew)
{
    controller.setGoal(xRand);

    VectorXd xStart = x0;

    bool valid = false;

    for(unsigned i = 0; i < loopN; i++)
    {
        VectorXd x = model.compute(xStart, deltaT);

        if(map.isFree(x))
        {
            xNew = x;
            valid = true;
            xStart = x;
        }
        else
        {
            break;
        }
    }

    return valid;
}

bool ClosedLoopExtender::check(const VectorXd& x0, const VectorXd& xGoal, std::vector<Eigen::VectorXd>& parents, double& cost)
{
    controller.setGoal(xGoal);
    VectorXd xNew;

    VectorXd xStart = x0;

    bool valid = false;

    for(unsigned i = 0; i < loopN; i++)
    {
        VectorXd x = model.compute(xStart, deltaT);

        if(map.isFree(x))
        {
            xNew = x;
            valid = true;
            xStart = x;
        }
        else
        {
            break;
        }
    }

    if(valid && xNew != xGoal) {valid = false;}

    return valid;
}

bool ClosedLoopExtender::los(const VectorXd& x0, const VectorXd& xRand, VectorXd& xNew)
{
    controller.setGoal(xRand);

    VectorXd xStart = x0;

    bool valid = false;

    for(unsigned i = 0; i < loopN; i++)
    {
        VectorXd x = model.compute(xStart, deltaT);

        if(map.isFree(x))
        {
            xNew = x;
            valid = true;
            xStart = x;
        }
        else
        {
            break;
        }
    }

    return valid;

}

bool ClosedLoopExtender::steer(const VectorXd& xStart, const VectorXd& xCorner, VectorXd& xNew, vector<VectorXd>& parents, double& cost)
{
    //Separates the length check from the angle check
    Distance& l2dis = *l2distance;
    Distance& thetadis = *thetadistance;

    VectorXd xCurr = xStart;

    bool is_valid = true;
    set<VectorXd, CmpReached> check;

    double length = l2dis(xCurr, xCorner);

    do{
        is_valid = los(xCurr, xCorner, xNew);
        if(!check.insert(xNew).second){
            is_valid = false;
        }
        cost += l2dis(xCurr, xNew);
        xCurr = xNew;
        parents.push_back(xCurr);
     } while(is_valid && !((l2dis(xCurr, xCorner) < deltaX) && (thetadis(xCurr, xCorner) < deltaTheta)));

    return is_valid;
}

bool ClosedLoopExtender::steer_l2(const VectorXd& xStart, const VectorXd& xCorner, VectorXd& xNew, vector<VectorXd>& parents, double& cost)
{
    //Separates the length check from the angle check
    Distance& l2dis = *l2distance;
    Distance& thetadis = *thetadistance;

    VectorXd xCurr = xStart;

    bool is_valid = true;
    set<VectorXd, CmpReached> check;

    double length = l2dis(xCurr, xCorner);

    do{
        is_valid = los(xCurr, xCorner, xNew);
        if(!check.insert(xNew).second){
            is_valid = false;
        }
        cost += l2dis(xCurr, xNew);
        xCurr = xNew;
        parents.push_back(xCurr);
     } while(is_valid && !((l2dis(xCurr, xCorner) < deltaX) && (thetadis(xCurr, xCorner) < deltaTheta)));

    return is_valid;
}


bool ClosedLoopExtender::isReached(const VectorXd& x0, const VectorXd& xTarget)
{
    Distance& l2dis = *this->l2distance;
    Distance& thetadis = *this->thetadistance;
    return ((l2dis(x0, xTarget) < deltaX) && (thetadis(x0, xTarget) < deltaTheta));
}


void ClosedLoopExtender::initialize(ros::NodeHandle& nh)
{
    nh.param("deltaT", deltaT, 0.5);
    nh.param("loopN", loopN, 3);

    std::string plannerNamespace = nh.getNamespace();
    if(plannerNamespace == std::string("/move_base/NHPlanner"))
    {
        nh.param("deltaX", deltaX, 0.5);
        nh.param("deltaTheta", deltaTheta, 1.86);
    }
}


ClosedLoopExtender::~ClosedLoopExtender()
{

}


}
