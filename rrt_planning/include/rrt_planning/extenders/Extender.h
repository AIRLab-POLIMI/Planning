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

#ifndef INCLUDE_RRT_PLANNING_EXTENDERS_EXTENDER_H_
#define INCLUDE_RRT_PLANNING_EXTENDERS_EXTENDER_H_

#include "rrt_planning/map/Map.h"
#include "rrt_planning/distance/Distance.h"

#include <ros/ros.h>

namespace rrt_planning
{
struct CmpReached
{
  bool operator()(const Eigen::VectorXd a, const Eigen::VectorXd b) const
  {
      return ((a(0) < b(0)) ||(a(0) == b(0) && a(1) < b(1)) ||
               (a(0) == b(0) && a(1) == b(1) && a(2) < b(2)));
  }

  bool operator()(const Eigen::Vector2d a, const Eigen::Vector2d b) const
  {
      return ((a(0) < b(0)) ||(a(0) == b(0) && a(1) < b(1)));
  }
};

class Extender
{
public:
    Extender(Map& map, Distance& distance) : map(map), distance(distance)
    {

    }

    virtual bool compute(const Eigen::VectorXd& x0, const Eigen::VectorXd& xRand, Eigen::VectorXd& xNew) = 0;
    virtual bool los(const Eigen::VectorXd& x0, const Eigen::VectorXd& xRand, Eigen::VectorXd& xNew) = 0;
    virtual bool check(const Eigen::VectorXd& x0, const Eigen::VectorXd& xGoal, std::vector<Eigen::VectorXd>& parents, double& cost) = 0;
    virtual void initialize(ros::NodeHandle& nh) = 0;
    virtual bool steer(const Eigen::VectorXd& xCurr, const Eigen::VectorXd& xCorner, Eigen::VectorXd& xNew, std::vector<Eigen::VectorXd>& parents, double& cost) = 0;
    virtual bool steer_l2(const Eigen::VectorXd& xCurr, const Eigen::VectorXd& xCorner, Eigen::VectorXd& xNew, std::vector<Eigen::VectorXd>& parents, double& cost) = 0;
    virtual bool isReached(const Eigen::VectorXd& x0, const Eigen::VectorXd& xTarget) = 0;
    virtual ~Extender()
    {

    }

protected:
    Map& map;
    Distance& distance;
    Distance* l2distance;
    Distance* thetadistance;
};


}



#endif /* INCLUDE_RRT_PLANNING_EXTENDERS_EXTENDER_H_ */
