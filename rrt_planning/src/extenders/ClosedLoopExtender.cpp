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
    VectorXd xCurr = x0;
    VectorXd xNew;
    bool is_valid = true;

    do{
        is_valid = los(xCurr, xGoal, xNew);
        cost += distance(xCurr, xNew);
        xCurr = xNew;
        parents.push_back(xCurr);

     } while(is_valid && !isReached(xCurr, xGoal));

    bool result = is_valid && isReached(xGoal, xNew);


    int size = parents.size();

    if(result && size!=0)
    {
        if(size == 1)
        {
            parents.pop_back();
            cost = distance(x0, xGoal);
        }
        else
        {
            cost -= distance(parents[size-1], parents[size-2]);
            parents.pop_back();
            size = parents.size();
            cost += distance(parents[size-1], xGoal);
        }
    }

    return result;

}


bool ClosedLoopExtender::los(const VectorXd& x0, const VectorXd& xRand, VectorXd& xNew)
{
    controller.setGoal(xRand);

    bool valid = false;

    VectorXd x = model.compute(x0, deltaT);
    if(map.isFree(x))
    {
       xNew = x;
       valid = true;
     }

    return valid;
}

bool ClosedLoopExtender::steer(const VectorXd& xStart, const VectorXd& xCorner, VectorXd& xNew, vector<VectorXd>& parents, double& cost)
{
    controller.setGoal(xCorner);
    VectorXd xCurr = xStart;
    int j = 0;

    bool is_valid = false;

    do {
        is_valid = los(xCurr, xCorner, xNew);
        cost += distance(xCurr, xNew);
        xCurr = xNew;
        parents.push_back(xCurr);
        j++;
    } while((loopN == -1 || j < loopN) && is_valid && !isReached(xCurr, xCorner));

    return is_valid;
}

bool ClosedLoopExtender::steer_l2(const VectorXd& xStart, const VectorXd& xCorner, VectorXd& xNew, vector<VectorXd>& parents, double& cost)
{
    Distance& l2dis = *this->l2distance;
    controller.setGoal(xCorner);
    VectorXd xCurr = xStart;
    int j = 0;

    bool is_valid = false;

    do {
        is_valid = los(xCurr, xCorner, xNew);
        cost += l2dis(xCurr, xNew);
        xCurr = xNew;
        parents.push_back(xCurr);
        j++;
    } while((loopN == -1 || j < loopN) && is_valid && !isReached(xCurr, xCorner));

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
    nh.param("K", loopN, -1);

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
