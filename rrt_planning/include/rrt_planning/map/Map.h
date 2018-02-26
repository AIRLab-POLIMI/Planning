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

#ifndef INCLUDE_RRT_PLANNING_MAP_MAP_H_
#define INCLUDE_RRT_PLANNING_MAP_MAP_H_

#include <Eigen/Dense>
#include "rrt_planning/map/Bounds.h"

namespace rrt_planning
{

class Map
{

public:
    virtual bool isFree(const Eigen::VectorXd& p) = 0;
    virtual bool isVoronoiFree(const Eigen::VectorXd& p) = 0;
    virtual unsigned char getCost(const Eigen::VectorXd& p) = 0;
    virtual bool insideBound(const Eigen::VectorXd& p) = 0;
    virtual Eigen::VectorXd getOutsidePoint() = 0;


    inline Bounds getBounds()
    {
        return bounds;
    }

    virtual ~Map()
    {
    }

protected:
    Bounds bounds;

};

}

#endif /* INCLUDE_RRT_PLANNING_MAP_MAP_H_ */
