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

#include "rrt_planning/extenders/MotionPrimitivesExtender.h"

using namespace Eigen;

namespace rrt_planning
{

MotionPrimitivesExtender::MotionPrimitivesExtender(KinematicModel& model, ConstantController& controller,
        Map& map, Distance& distance)
    : Extender(map, distance), model(model), controller(controller), minU(model.getActionSize()), maxU(model.getActionSize())
{
    deltaT = 0;
    discretization = 0;
}

bool MotionPrimitivesExtender::compute(const VectorXd& x0, const VectorXd& xRand, VectorXd& xNew)
{
    double minDistance = std::numeric_limits<double>::infinity();

    for(auto& mp : motionPrimitives)
    {
        VectorXd x = model.applyTransform(x0, mp);

        if(map.isFree(x))
        {
            double currentDist = distance(xRand, x);

            if(currentDist < minDistance)
            {
                xNew = x;
                minDistance = currentDist;
            }
        }
    }

    return minDistance < std::numeric_limits<double>::infinity();
}

bool MotionPrimitivesExtender::los(const VectorXd& x0, const VectorXd& xRand, VectorXd& xNew, double length)
{
    double minDistance = std::numeric_limits<double>::infinity();
    //double threshold = minDistance;
    //bool reachable = false;

    for(auto& mp : motionPrimitives)
    {
        VectorXd x = model.applyTransform(x0, mp);

        /*double currentDist = distance(xRand, x);

        if(!reachable && map.isFree(x))
        {
            xNew = x;
            minDistance = currentDist;
            reachable = true;
        }
        else if(currentDist < minDistance)
        {
            if(map.isFree(x))
            {
                xNew = x;
                minDistance = currentDist;
                reachable = true;
            }
            else if(!reachable)
            {
                xNew = x;
                minDistance = currentDist;
            }
        }*/

      double currentDist = distance(xRand, x, length);
      if(currentDist < minDistance)
      {
          xNew = x;
          minDistance = currentDist;
      }
    }

    return ((minDistance < std::numeric_limits<double>::infinity()) && map.isFree(xNew));
}

bool MotionPrimitivesExtender::check(const VectorXd& x0, const VectorXd& xGoal)
{
    double minDistance = std::numeric_limits<double>::infinity();

    for(auto& mp : motionPrimitives)
    {
        VectorXd x = model.applyTransform(x0, mp);

        if(map.isFree(x) && x == xGoal)
        {
            return true;
        }
    }

    return false;
}

void MotionPrimitivesExtender::initialize(ros::NodeHandle& nh)
{
    nh.param("deltaT", deltaT, 0.5);
    nh.param("motion_primitives/discretization", discretization, 5);

    std::vector<double> minU_vec;
    std::vector<double> maxU_vec;

    bool test1 = nh.getParam("motion_primitives/minU", minU_vec);
    bool test2 = nh.getParam("motion_primitives/maxU", maxU_vec);

    if(minU_vec.size() != model.getActionSize() ||
            maxU_vec.size() != model.getActionSize())
        throw std::runtime_error("Wrong discretization parameters specified. minU and maxU should have the correct size");

    for(unsigned int i = 0; i < minU_vec.size(); i++)
    {
        minU(i) = minU_vec[i];
        maxU(i) = maxU_vec[i];
    }

    generateMotionPrimitives();
}

void MotionPrimitivesExtender::generateMotionPrimitives()
{
    VectorXd dU = (maxU - minU) / (discretization-1.0);

    generateMotionPrimitive(minU, dU, 0);
}

void MotionPrimitivesExtender::generateMotionPrimitive(const Eigen::VectorXd& u, const Eigen::VectorXd& du, unsigned int index)
{
    if(index == model.getActionSize())
    {
        controller.setControl(u);
        VectorXd&& mp = model.compute(model.getInitialState(), deltaT);
        motionPrimitives.push_back(mp);
    }
    else
    {
        Eigen::VectorXd uc = u;
        for(unsigned int n = 0; n < discretization; n++)
        {
            generateMotionPrimitive(uc, du, index+1);
            uc(index) += du(index);
        }
    }
}

MotionPrimitivesExtender::~MotionPrimitivesExtender()
{

}


}
