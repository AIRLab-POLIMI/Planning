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

#include "rrt_planning/rrt/RRTNode.h"

namespace rrt_planning
{

RRTNode::RRTNode()
{
    father = nullptr;
}

RRTNode::RRTNode(RRTNode* father, const Eigen::VectorXd& x)
{
    this->father = father;
    this->x = x;
    this->projectionCost = 0;
    this->cost = 0;
    std::vector<Eigen::VectorXd> empty;
    this->primitives = empty;
}

RRTNode::RRTNode(RRTNode* father, const Eigen::VectorXd& x, double cost)
{
    this->father = father;
    this->x = x;
    this->projectionCost = cost;
    this->cost = 0;
    std::vector<Eigen::VectorXd> empty;
    this->primitives = empty;
}

RRTNode::RRTNode(RRTNode* father, const Eigen::VectorXd& x, std::vector<Eigen::VectorXd> primitives, double cost)
{
    this->father = father;
    this->x = x;
    this->cost = cost;
    this->primitives = primitives;
    this->projectionCost = 0;
}

RRTNode::RRTNode(RRTNode* father, const Eigen::VectorXd& x, std::vector<Eigen::VectorXd> primitives, double cost, double projectionCost)
{
    this->father = father;
    this->x = x;
    this->cost = cost;
    this->primitives = primitives;
    this->projectionCost = projectionCost;
}

}
