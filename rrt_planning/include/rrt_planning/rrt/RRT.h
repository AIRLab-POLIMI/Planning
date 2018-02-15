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

#ifndef INCLUDE_RRT_PLANNING_RRT_RRT_H_
#define INCLUDE_RRT_PLANNING_RRT_RRT_H_

#include "rrt_planning/distance/Distance.h"
#include "rrt_planning/rrt/RRTNode.h"
#include "rrt_planning/rrt/RRTIndex.h"

namespace rrt_planning
{

class RRT
{
public:
    RRT(Distance& distance, Eigen::VectorXd& x0);

    RRTNode* searchNearestNode(Eigen::VectorXd& x);
    void addNode(RRTNode* parent, Eigen::VectorXd& xNew);
    void addNode(RRTNode* parent, Eigen::VectorXd& xNew, double cost);

    std::vector<Eigen::VectorXd> getPathToLastNode();
    std::vector<Eigen::VectorXd> getPathToLastNode(RRTNode* last);

    std::vector<RRTNode*> findNeighbors(Eigen::VectorXd& xNew, int k, double ray);
    double computeCost(RRTNode* node);
    double computeLength(RRTNode* node);
    RRTNode* getPointer();
    int getLength();

    ~RRT();


private:
    RRTNode* root;
    std::vector<RRTNode*> nodes;
    RRTIndex index;
    Distance& distance;


};

}

#endif /* INCLUDE_RRT_PLANNING_RRT_RRT_H_ */
