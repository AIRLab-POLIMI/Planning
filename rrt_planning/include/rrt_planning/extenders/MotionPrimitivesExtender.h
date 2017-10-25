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

#include "rrt_planning/extenders/Extender.h"
#include "rrt_planning/kinematics_models/KinematicModel.h"
#include "rrt_planning/kinematics_models/controllers/ConstantController.h"


namespace rrt_planning
{

class MotionPrimitivesExtender : public Extender
{
public:
    MotionPrimitivesExtender(KinematicModel& model, ConstantController& controller,
                             Map& map, Distance& distance);

    virtual bool compute(const Eigen::VectorXd& x0, const Eigen::VectorXd& xRand, Eigen::VectorXd& xNew) override;
    virtual bool check(const Eigen::VectorXd& x0, const Eigen::VectorXd& xGoal) override;
    virtual void initialize(ros::NodeHandle& nh) override;

    virtual ~MotionPrimitivesExtender();


private:
    void generateMotionPrimitives();
    void generateMotionPrimitive(const Eigen::VectorXd& u, const Eigen::VectorXd& du, unsigned int index);

private:
    std::vector<Eigen::VectorXd> motionPrimitives;
    KinematicModel& model;
    ConstantController& controller;

    Eigen::VectorXd maxU;
    Eigen::VectorXd minU;
    int discretization;
    double deltaT;

};


}
