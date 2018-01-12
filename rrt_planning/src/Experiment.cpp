#include <fstream>
#include <iostream>
#include <string>

#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <rrt_planning/NHPlanner.h>

rrt_planning::NHPlanner* getPlanner(const std::string& name, costmap_2d::Costmap2DROS* costmap_ros);
void init(geometry_msgs::PoseStamped& start_pose,
				geometry_msgs::PoseStamped& goal_pose);


int main(int argc, char** argv)
{
	ROS_FATAL_STREAM("Starting");
	std::string planner_name, map;
	ros::init(argc, argv, "experiment");
	ros::NodeHandle private_nh("~/");

	private_nh.param("planner_name", planner_name, std::string("NHPlanner"));
	private_nh.param("map", map, std::string("map"));
	ROS_FATAL_STREAM("map name: " + map);

	//Costmap inizialization magics
    tf::TransformListener tf_(ros::Duration(10));
    costmap_2d::Costmap2DROS* costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    costmap_ros->pause();

	ROS_FATAL_STREAM("Costmap loaded");

	//Open file and start reading
	geometry_msgs::PoseStamped start_pose, goal_pose;
	std::vector<geometry_msgs::PoseStamped> plan;

	init(start_pose, goal_pose);
	//Launch planner
	rrt_planning::NHPlanner* planner = getPlanner(planner_name, costmap_ros);
	ROS_FATAL_STREAM("Started planner: " + planner_name);
	bool result = planner->makePlan(start_pose, goal_pose, plan);
	ROS_FATAL_STREAM("Plan found: " + result);

	return 0;
}

void init(geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& goal_pose)
{
	start_pose.pose.position.x = 24.6607;
	start_pose.pose.position.y = 2.52053;
	start_pose.pose.position.z = 0;
	start_pose.pose.orientation.x = 0;
	start_pose.pose.orientation.y = 0;
	start_pose.pose.orientation.z = -0.41112;
	start_pose.pose.orientation.w = 0.911581;

	goal_pose.pose.position.x = 11.9645;
	goal_pose.pose.position.y = -4.15718;
	goal_pose.pose.position.z = 0;
	goal_pose.pose.orientation.x = 0;
	goal_pose.pose.orientation.y = 0;
	goal_pose.pose.orientation.z = 0.974315;
	goal_pose.pose.orientation.w = 0.225188;


}


rrt_planning::NHPlanner* getPlanner(const std::string& name, costmap_2d::Costmap2DROS* costmap_ros)
{
	if(name == "NHPlanner")
	{
		rrt_planning::NHPlanner* planner = new rrt_planning::NHPlanner(std::string(""), costmap_ros);
		return planner;
	}
}
