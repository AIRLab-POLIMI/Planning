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
using namespace std;

namespace rrt_planning
{

MotionPrimitivesExtender::MotionPrimitivesExtender(KinematicModel& model, ConstantController& controller,
        Map& map, Distance& distance)
    : Extender(map, distance), model(model), controller(controller), minU(model.getActionSize()), maxU(model.getActionSize())
{
    deltaT = 0;
    discretization = 0;
    l2distance = new L2Distance();
    thetadistance = new ThetaDistance();
}

bool MotionPrimitivesExtender::compute(const VectorXd& x0, const VectorXd& xSample, VectorXd& xNew)
{
    VectorXd xRand = xSample;
    if(diffDrive)
    {
        double dis = sqrt(pow((x0(0) - xSample(0)),2) + pow((x0(1) - xSample(1)), 2));
        if(dis > 0.5)
        {
            double theta = atan2(xSample(1) - x0(1), xSample(0) - x0(0));
            xRand(2) = theta;
        }
    }

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

bool MotionPrimitivesExtender::los(const VectorXd& x0, const VectorXd& xSample, VectorXd& xNew, double length)
{
    VectorXd xRand = xSample;
    if(diffDrive)
    {
        double dis = sqrt(pow((x0(0) - xSample(0)),2) + pow((x0(1) - xSample(1)), 2));
        if(dis > 0.5)
        {
            double theta = atan2(xSample(1) - x0(1), xSample(0) - x0(0));
            xRand(2) = theta;
        }
    }

    double minDistance = std::numeric_limits<double>::infinity();

    for(auto& mp : motionPrimitives)
    {
        VectorXd x = model.applyTransform(x0, mp);

        double currentDist = distance(x, xRand, length);
        if(currentDist < minDistance)
        {
          xNew = x;
          minDistance = currentDist;
        }

    }

    return ((minDistance < std::numeric_limits<double>::infinity()) && map.isFree(xNew));
}

bool MotionPrimitivesExtender::steer(const VectorXd& xStart, const VectorXd& xCorner, VectorXd& xNew, vector<VectorXd>& parents, double& cost)
{
    //Separates the length check from the angle check
    Distance& l2dis = *this->l2distance;
    Distance& thetadis = *this->thetadistance;

    VectorXd xCurr = xStart;
    double meters = l2dis(xCurr, xCorner);
    double angles = thetadis(xCurr, xCorner);

    bool is_valid = true;
    set<VectorXd, CmpReached> check;

    double length = l2dis(xStart, xCorner);

    do{
        is_valid = los(xCurr, xCorner, xNew, length);
        if(!check.insert(xNew).second){
            is_valid = false;
        }
        cost += l2dis(xCurr, xNew);
        xCurr = xNew;
        parents.push_back(xCurr);
        meters = l2dis(xCurr, xCorner);
        angles = thetadis(xCurr, xCorner);
     } while(is_valid && !((meters < deltaX) && (angles < deltaTheta)));

    if(is_valid)
        ROS_FATAL_STREAM("Number of motion primitives: " << parents.size());
    return is_valid;
}

bool MotionPrimitivesExtender::isReached(const VectorXd& x0, const VectorXd& xTarget)
{
    Distance& l2dis = *this->l2distance;
    Distance& thetadis = *this->thetadistance;
    return ((l2dis(x0, xTarget) < deltaX) && (thetadis(x0, xTarget) < deltaTheta));
}

bool MotionPrimitivesExtender::check(const VectorXd& x0, const VectorXd& xGoal)
{
    VectorXd xNew;
    vector<VectorXd> dummy;
    double minDistance = std::numeric_limits<double>::infinity();

    return steer(x0, xGoal, xNew, dummy, minDistance);

    for(auto& mp : motionPrimitives)
    {
        VectorXd x = model.applyTransform(x0, mp);

        if(map.isFree(x) && isReached(x, xGoal))
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

    std::string planner_name;
    nh.param("planner_name", planner_name, std::string("unspecified"));


    std::string kinematicModelName;
    nh.param("kinematicModel", kinematicModelName, std::string("DifferentialDrive"));
    if(kinematicModelName == "DifferentialDrive")
    {
        distance = L2ThetaDistance();
        diffDrive = true;
    }
    else if(kinematicModelName == "Bicycle")
    {
        distance = L2ThetaDistance();
        diffDrive = false;
    }

    nh.param("deltaX", deltaX, 0.5);
    nh.param("deltaTheta", deltaTheta, 1.86);

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
