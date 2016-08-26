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

#include "rrt_planning/local_planner/MotionPrimitivesPlanner.h"

using namespace Eigen;

namespace rrt_planning
{

MotionPrimitivesPlanner::MotionPrimitivesPlanner(Map& map, Distance& distance, KinematicModel& model)
    : LocalPlanner(map, distance), model(model), minU(model.getActionSize()), maxU(model.getActionSize())
{
    deltaT = 0;
    discretization = 0;
}

bool MotionPrimitivesPlanner::compute(const VectorXd& x0, const VectorXd& xRand, VectorXd& xNew)
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

void MotionPrimitivesPlanner::initialize(ros::NodeHandle& nh)
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

void MotionPrimitivesPlanner::generateMotionPrimitives()
{
    VectorXd dU = (maxU - minU) / (discretization-1.0);

    generateMotionPrimitive(minU, dU, 0);
}

void MotionPrimitivesPlanner::generateMotionPrimitive(const Eigen::VectorXd& u, const Eigen::VectorXd& du, unsigned int index)
{
    if(index == model.getActionSize())
    {
        VectorXd&& mp = model.compute(model.getInitialState(), u, deltaT);
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


}
