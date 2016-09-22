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

#ifndef INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_BICYCLE_H_
#define INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_BICYCLE_H_

#include "rrt_planning/kinematics_models/KinematicModel.h"

namespace rrt_planning
{

class Bicycle : public KinematicModel
{
public:
	Bicycle();
    virtual Eigen::VectorXd compute(const Eigen::VectorXd& x0, const Eigen::VectorXd& u, double delta) override;
    virtual Eigen::VectorXd applyTransform(const Eigen::VectorXd& x0, const Eigen::VectorXd& T) override;
    virtual Eigen::VectorXd getInitialState() override;
    virtual Eigen::VectorXd getRandomState(const Bounds& bounds) override;


    void operator()(const state_type& x, state_type& dx,
                    const double /* t */);

private:
    Eigen::VectorXd u;

    double deltaPhi;
    double l;
};

}



#endif /* INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_BICYCLE_H_ */
