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

#ifndef INCLUDE_RRT_PLANNING_MAP_ROSMAP_H_
#define INCLUDE_RRT_PLANNING_MAP_ROSMAP_H_

#include "rrt_planning/map/Map.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/costmap_2d.h"

namespace rrt_planning
{

class ROSMap : public Map
{
public:
    ROSMap(costmap_2d::Costmap2DROS* costmap_ros);

    virtual bool isFree(const Eigen::VectorXd& p) override;
    virtual bool isVoronoiFree(const Eigen::VectorXd& p) override;
    virtual unsigned char getCost(const Eigen::VectorXd& p) override;
    virtual bool insideBound(const Eigen::VectorXd& p) override;
    virtual Eigen::VectorXd getOutsidePoint() override;

    virtual ~ROSMap();


private:
    costmap_2d::Costmap2DROS* costmap_ros;
    costmap_2d::Costmap2D* costmap;
};

}

#endif /* INCLUDE_RRT_PLANNING_MAP_ROSMAP_H_ */
