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

#include "rrt_planning/kinematics_models/controllers/POSQ.h"

#include <cmath>
#include <angles/angles.h>

#include <iostream>

using namespace Eigen;
using namespace std;

namespace rrt_planning
{

POSQ::POSQ(double Krho, double Kv, double Kalpha, double Kphi) :
    Krho(Krho), Kv(Kv), Kalpha(Kalpha), Kphi(Kphi)
{

}

VectorXd POSQ::operator()(const Eigen::VectorXd& x) const
{
    VectorXd u(2);

    double rho = (x.head(2) - goal.head(2)).norm();
    double alphaGoal = (rho==0) ? (0) : (atan2(goal(1) - x(1), goal(0) - x(0)));
    double alpha = (rho==0) ? (0) : (angles::shortest_angular_distance(alphaGoal, x(2)));
    double phi = angles::shortest_angular_distance (x(2), goal(2));

    u(0) = Krho*tanh(Kv*rho);
    u(1) = -Kalpha*alpha-Kphi*phi;

    return u;
}

POSQ::~POSQ()
{

}

}
