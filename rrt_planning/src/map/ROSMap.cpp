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

#include "rrt_planning/map/ROSMap.h"

#include <costmap_2d/cost_values.h>

//#define DEBUG_CONF

namespace rrt_planning
{

ROSMap::ROSMap(costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros(costmap_ros),
    costmap(costmap_ros->getCostmap())
{
    bounds.minX = costmap->getOriginX();
    bounds.minY = costmap->getOriginY();
    bounds.maxX = bounds.minX + costmap->getSizeInMetersX();
    bounds.maxY = bounds.minY + costmap->getSizeInMetersY();
#ifdef DEBUG_CONF
    ROS_FATAL_STREAM("minX: " << bounds.minX);
    ROS_FATAL_STREAM("maxX: " << bounds.maxX);
    ROS_FATAL_STREAM("minY: " << bounds.minY);
    ROS_FATAL_STREAM("maxY: " << bounds.maxY);
#endif
    bounds.minZ = 0;
    bounds.maxZ = 0;
}

bool ROSMap::isFree(const Eigen::VectorXd& p)
{
    return getCost(p) <= costmap_2d::FREE_SPACE;
}

unsigned char ROSMap::getCost(const Eigen::VectorXd& p)
{
    double wx = p(0);
    double wy = p(1);

    unsigned int mx;
    unsigned int my;

    if(costmap->worldToMap(wx, wy, mx, my))
        return costmap->getCost(mx, my);
    else
    {
#ifdef DEBUG_CONF
        std::cerr << "out of map" << std::endl;
#endif
        return costmap_2d::NO_INFORMATION;
    }

}

bool ROSMap::insideBound(const Eigen::VectorXd& p)
{
  double wx = p(0);
  double wy = p(1);

  unsigned int mx;
  unsigned int my;
  bool result = costmap->worldToMap(wx, wy, mx, my);

  return result;
}

Eigen::VectorXd ROSMap::getOutsidePoint()
{
    Eigen::Vector3d p(bounds.maxX + 1.0, bounds.maxY + 1.0, 0);
    return p;
}

ROSMap::~ROSMap()
{

}

}
