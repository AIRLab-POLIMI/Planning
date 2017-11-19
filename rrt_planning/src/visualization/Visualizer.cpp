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

#include "rrt_planning/visualization/Visualizer.h"

#include <visualization_msgs/Marker.h>

namespace rrt_planning
{

void Visualizer::initialize(ros::NodeHandle& nh)
{
    nh.param("visualization/minPoints", minPoints, 100);
    nh.param("visualization/minSegments", minSegments, 100);
    nh.getParam("visualization/disable", disableVisualization);

    pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
}

void Visualizer::addPoint(const Eigen::VectorXd& point)
{
    if(disableVisualization)
        return;

    points.push_back(point);


    if(points.size() >= minPoints)
    {
        displayPoints();
    }
}

void Visualizer::addSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
{
    if(disableVisualization)
        return;

    segments.push_back(std::make_pair(start, end));


    if(segments.size() >= minSegments)
    {
        displaySegments();
    }

}

void Visualizer::addUpdate(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
{
    if(disableVisualization)
        return;

    updates.push_back(std::make_pair(start, end));


    if(updates.size() >= minSegments)
    {
        displayUpdates();
    }

}

void Visualizer::displayPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(disableVisualization)
        return;

    static int id = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "plan";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    for(int i = 0; i < plan.size(); i++)
    {
        geometry_msgs::Point p;

        p.x = plan[i].pose.position.x;
        p.y = plan[i].pose.position.y;
        p.z = plan[i].pose.position.z;

        marker.points.push_back(p);
    }

    pub.publish(marker);

}

void Visualizer::displayPoints()
{
    static int id = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "points";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for(auto& p_eigen : points)
    {
        geometry_msgs::Point p;

        p.x = p_eigen(0);
        p.y = p_eigen(1);
        p.z = 0;

        marker.points.push_back(p);
    }

    pub.publish(marker);

    points.clear();
}

void Visualizer::displaySegments()
{
    static int id = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "rrt";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    for(auto& segment : segments)
    {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = segment.first(0);
        p1.y = segment.first(1);
        p1.z = segment.first(2);

        p2.x = segment.second(0);
        p2.y = segment.second(1);
        p2.z = segment.second(2);

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    pub.publish(marker);

    segments.clear();
}

void Visualizer::displayUpdates()
{
    static int id = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "updates";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    for(auto& segment : updates)
    {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = segment.first(0);
        p1.y = segment.first(1);
        p1.z = segment.first(2);

        p2.x = segment.second(0);
        p2.y = segment.second(1);
        p2.z = segment.second(2);

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    pub.publish(marker);

    updates.clear();
}

void Visualizer::flush()
{
    displayPoints();
    displaySegments();
    displayUpdates();
}

void Visualizer::clean()
{
    //Prepare msg
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::DELETEALL;

    //Delete points
    marker.ns = "points";
    pub.publish(marker);

    //Delete rrt tree
    marker.ns = "rrt";
    pub.publish(marker);

    //Delete plan
    marker.ns = "plan";
    pub.publish(marker);

    marker.ns = "updates";
    pub.publish(marker);
}


}
