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

#ifndef INCLUDE_RRT_PLANNING_EXTENDERS_CLOSEDLOOPEXTENDER_CPP_
#define INCLUDE_RRT_PLANNING_EXTENDERS_CLOSEDLOOPEXTENDER_CPP_

#include "rrt_planning/extenders/Extender.h"
#include "rrt_planning/kinematics_models/KinematicModel.h"
#include "rrt_planning/kinematics_models/controllers/Controller.h"

namespace rrt_planning
{

class ClosedLoopExtender : public Extender
{
public:
    ClosedLoopExtender(KinematicModel& model, Controller& controller, Map& map, Distance& distance);

    virtual bool compute(const Eigen::VectorXd& x0, const Eigen::VectorXd& xRand, Eigen::VectorXd& xNew) override;
    virtual bool check(const Eigen::VectorXd& x0, const Eigen::VectorXd& xGoal, std::vector<Eigen::VectorXd>& parents, double& cost) override;
    virtual bool los(const Eigen::VectorXd& x0, const Eigen::VectorXd& xRand, Eigen::VectorXd& xNew) override;
    virtual bool steer(const Eigen::VectorXd& xCurr, const Eigen::VectorXd& xCorner, Eigen::VectorXd& xNew, std::vector<Eigen::VectorXd>& parents, double& cost) override;
    virtual bool steer_l2(const Eigen::VectorXd& xCurr, const Eigen::VectorXd& xCorner, Eigen::VectorXd& xNew, std::vector<Eigen::VectorXd>& parents, double& cost) override;
    virtual bool isReached(const Eigen::VectorXd& x0, const Eigen::VectorXd& xTarget) override;
    virtual void initialize(ros::NodeHandle& nh) override;

    virtual ~ClosedLoopExtender();

private:
    Controller& controller;
    KinematicModel& model;

    double deltaT;
    int loopN;
    
    double deltaX;
    double deltaTheta;

};

}


#endif /* INCLUDE_RRT_PLANNING_EXTENDERS_CLOSEDLOOPEXTENDER_CPP_ */
