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

#ifndef INCLUDE_RRT_PLANNING_RRT_RRTINDEX_H_
#define INCLUDE_RRT_PLANNING_RRT_RRTINDEX_H_

#include "rrt_planning/rrt/Cover_Tree.h"

namespace rrt_planning
{

class RRTCoverWrapper
{
public:
    RRTCoverWrapper(Distance* distance, RRTNode* node) : dist(distance), node(node)
    {

    }

    inline bool operator ==(const RRTCoverWrapper& obj) const
    {
        return obj.node->x == this->node->x;
    }

    inline double distance(const RRTCoverWrapper& obj) const
    {
        auto& dist = *this->dist;
        return dist(obj.node->x, this->node->x);
    }

    inline RRTNode* getNode() const
    {
        return node;
    }

private:
    RRTNode* node;
    Distance* dist;
};

class RRTIndex : CoverTree<RRTCoverWrapper>
{
public:
    RRTIndex(Distance& dist) : dist(dist), CoverTree<RRTCoverWrapper>(2e4)
    {

    }

    inline void insert(RRTNode* p)
    {
        CoverTree<RRTCoverWrapper>::insert(RRTCoverWrapper(&dist, p));
    }

    inline void remove(RRTNode* p)
    {
        CoverTree<RRTCoverWrapper>::remove(RRTCoverWrapper(&dist, p));
    }

    inline RRTNode* getNearestNeighbour(const Eigen::VectorXd& x)
    {
        RRTNode tmp(nullptr, x);
        RRTCoverWrapper tmpWrapped(&dist, &tmp);
        auto result = CoverTree<RRTCoverWrapper>::kNearestNeighbors(tmpWrapped, 1);

        return result.back().getNode();
    }

    inline std::vector<RRTNode*> getNearestNeighbours(const Eigen::VectorXd& x, int k)
    {
        RRTNode tmp(nullptr, x);
        RRTCoverWrapper tmpWrapped(&dist, &tmp);
        auto result = CoverTree<RRTCoverWrapper>::kNearestNeighbors(tmpWrapped, k);
        
        std::vector<RRTNode*> neighbors;
        for (auto n: result)
        {
          neighbors.push_back(n.getNode());
        }

        return neighbors;
    }


private:
    Distance& dist;
};

}


#endif /* INCLUDE_RRT_PLANNING_RRT_RRTINDEX_H_ */
