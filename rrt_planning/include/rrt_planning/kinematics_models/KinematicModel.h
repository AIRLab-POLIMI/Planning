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

#ifndef INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_
#define INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_


#include <Eigen/Dense>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>

#include "rrt_planning/map/Bounds.h"
#include "rrt_planning/kinematics_models/controllers/Controller.h"

namespace rrt_planning
{

class KinematicModel
{
public:
    /*typedef Eigen::VectorXd state_type;
    typedef boost::numeric::odeint::runge_kutta_dopri5<state_type> error_stepper_type;
    typedef boost::numeric::odeint::controlled_runge_kutta< error_stepper_type > controlled_stepper_type;*/

public:
    KinematicModel(Controller& controller) : controller(controller)
    {
        stateSize = 0;
        actionSize = 0;
    }

    virtual Eigen::VectorXd compute(const Eigen::VectorXd& x0, double delta) = 0;
    virtual Eigen::VectorXd getInitialState() = 0;

    virtual Eigen::VectorXd sampleOnBox(const Bounds& bounds) = 0;
    double sampleOnLane(std::vector<geometry_msgs::PoseStamped>& plan,
                        Eigen::VectorXd& p, double width, double deltaTheta);
    double voronoiSampleOnLane(std::vector<geometry_msgs::PoseStamped>& plan,
                        Eigen::VectorXd& p, double width, double deltaTheta);
    Eigen::VectorXd computeProjection(std::vector<geometry_msgs::PoseStamped>& plan,
                                 const Eigen::VectorXd& p);


    Eigen::VectorXd applyTransform(const Eigen::VectorXd& x0, const Eigen::VectorXd& T);

    inline unsigned int getStateSize()
    {
        return stateSize;
    }

    inline unsigned int getActionSize()
    {
        return actionSize;
    }

    virtual ~KinematicModel()
    {

    }


protected:
    Controller& controller;

    unsigned int stateSize;
    unsigned int actionSize;

    const double dt = 0.1;

};

}

#endif /* INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_ */
