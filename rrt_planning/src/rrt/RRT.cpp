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

#include "rrt_planning/rrt/RRT.h"
#include "ros/ros.h"

namespace rrt_planning
{

RRT::RRT(Distance& distance, Eigen::VectorXd& x0) : distance(distance), index(distance)
{
    root = new RRTNode(nullptr, x0);
    nodes.push_back(root);
    index.insert(root);
}

RRTNode* RRT::searchNearestNode(Eigen::VectorXd& x)
{
    /*RRTNode* nearest = root;
    double dMin = distance(root->x, x);

    for(auto* node : nodes)
    {
        double newDist = distance(node->x, x);

        if(newDist < dMin)
        {
            dMin = newDist;
            nearest = node;
        }
    }

    return nearest;*/
    return index.getNearestNeighbour(x);
}

void RRT::addNode(RRTNode* parent, Eigen::VectorXd& xNew)
{
    RRTNode* child = new RRTNode(parent, xNew);
    parent->childs.push_back(child);
    nodes.push_back(child);
    index.insert(child);
}

std::vector<Eigen::VectorXd> RRT::getPathToLastNode()
{
    std::vector<Eigen::VectorXd> path;
    RRTNode* current = nodes.back();

    while(current)
    {
        path.push_back(current->x);
        current = current->father;
    }

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<Eigen::VectorXd> RRT::getPathToLastNode(RRTNode* last)
{
    std::vector<Eigen::VectorXd> path;
    RRTNode* current = last;

    while(current)
    {
        path.push_back(current->x);
        current = current->father;
    }

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<RRTNode*> RRT::findNeighbors(Eigen::VectorXd& xNew, int k, double ray)
{
    std::vector<RRTNode*> candidates = index.getNearestNeighbours(xNew, k);
    std::vector<RRTNode*> neighbors;

    for(auto node : candidates)
    {
        //if(distance(node->x, xNew) <= ray)
        //{
            neighbors.push_back(node);
        //}
    }

    return neighbors;
}

double RRT::computeCost(RRTNode* node)
{
    RRTNode* current = node;
    double cost = 0;
    while(current != root)
    {
        RRTNode* parent = current->father;
        cost = cost + distance(current->x, parent->x);
        current = parent;
    }

    return cost;
}

double RRT::computeLength(RRTNode* node)
{
    RRTNode* current = node;
    double length = 0;
    while(current != root)
    {
        RRTNode* parent = current->father;
        length = length + (current->x.head(2)-parent->x.head(2)).norm();
        current = parent;
    }

    return length;
}

RRTNode* RRT::getPointer()
{
    return nodes.back();
}

int RRT::getLength()
{
    return nodes.size();
}

RRT::~RRT()
{
    for(auto node : nodes)
    {
        delete node;
    }
}

}
