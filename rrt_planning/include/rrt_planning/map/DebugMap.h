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

#ifndef INCLUDE_RRT_PLANNING_MAP_DEBUGMAP_H_
#define INCLUDE_RRT_PLANNING_MAP_DEBUGMAP_H_

#include "rrt_planning/map/Map.h"

namespace rrt_planning
{

class DebugMap : public Map
{
public:
    DebugMap();

    virtual bool isFree(const Eigen::VectorXd& p) override;
    virtual unsigned char getCost(const Eigen::VectorXd& p) override;
    virtual bool collisionPoints(const Eigen::VectorXd& a, const Eigen::VectorXd& b, std::vector<Eigen::VectorXd>& actions) override;
    virtual Eigen::VectorXd exitPoint(const Eigen::VectorXd& current, const Eigen::VectorXd& middle, bool cw) override;
    virtual bool forcedUpdate(const Eigen::VectorXd& a, const Eigen::VectorXd& b, std::vector<Eigen::VectorXd>& actions) override;
    virtual Eigen::VectorXd computeMiddle(const Eigen::VectorXd& a, const Eigen::VectorXd& b) override;
    virtual bool clockwise(const Eigen::VectorXd& a, const Eigen::VectorXd& b) override;
    virtual bool insideBound(const Eigen::VectorXd& p) override;
    virtual bool isCorner(const Eigen::VectorXd& current, int discretization, double ray, double threshold, std::vector<Eigen::VectorXd>& points) override;
    virtual bool isTrueCornerWOW(const Eigen::VectorXd& current) override;
    virtual bool followObstacle(const Eigen::VectorXd& current, const Eigen::VectorXd& a, std::vector<Eigen::VectorXd>& actions) override;
    virtual ~DebugMap();

private:

};

}

#endif /* INCLUDE_RRT_PLANNING_MAP_DEBUGMAP_H_ */
