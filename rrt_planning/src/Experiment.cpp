#include <fstream>
#include <iostream>
#include <string>
#include <fstream>
#include <chrono>

#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <rrt_planning/NHPlanner.h>

nav_core::BaseGlobalPlanner* getPlanner(const std::string& name, costmap_2d::Costmap2DROS* costmap_ros);
void parse(const std::string& conf, geometry_msgs::PoseStamped& start_pose,
				geometry_msgs::PoseStamped& goal_pose);
void save(const std::string& filename, const std::string& conf, double t, std::vector<geometry_msgs::PoseStamped>& plan)


int main(int argc, char** argv)
{
	ROS_FATAL_STREAM("Starting");
	std::string planner_name, map, node_name, conf, row;
	planner_name = argv[0];
	map = argv[1];
	conf = argv[2];
	row = argv[3];
	node_name = planner_name + map + row;

	ros::init(argc, argv, node_name);
	ros::NodeHandle private_nh("~/");

	//Costmap inizialization magics
    tf::TransformListener tf_(ros::Duration(10));
    costmap_2d::Costmap2DROS* costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    costmap_ros->pause();

	ROS_FATAL_STREAM("Costmap loaded");

	//Convert start and goal
	geometry_msgs::PoseStamped start_pose, goal_pose;
	std::vector<geometry_msgs::PoseStamped> plan;

	parse(conf, start_pose, goal_pose);

	//Launch planner
	nav_core::BaseGlobalPlanner* planner = getPlanner(planner_name, costmap_ros);
	ROS_FATAL_STREAM("Started planner: " + planner_name);

	std::chrono::steady_clock::time::point t0 = steady::clock().now();
	bool result = planner->makePlan(start_pose, goal_pose, plan);
	std::chrono::duration<double> ex_time = steady::clock().now() - t0;
	save(node_name, conf, t, ex_time.count());
	private_nh.deleteParam("");

	ROS_FATAL_STREAM("Plan found: " + result);
	ros::Duration(5).sleep();
	return 0;
}

void parse(const std::string& conf, geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& goal_pose)
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

void save(const std::string& filename, const std::string& conf, double t, std::vector<geometry_msgs::PoseStamped>& plan)
{
	std::ofstream f;
	f.open(filename + std::string(".exp"));

	std::string d = std::string("_");

	//Write configuration
	f << conf << "\n";

	//Write execution Time
	f << t << "\n";

	//Write path points
	for(auto p : plan)
	{
		f << p.pose.position.x << d << p.pose.position.y << d
		  << p.pose.orientation.z << d << p.pose.orientation.w << "\n";
	}

	f.close();
}


nav_core::BaseGlobalPlanner* getPlanner(const std::string& name, costmap_2d::Costmap2DROS* costmap_ros)
{
	if(name == "NHPlanner")
	{
		rrt_planning::NHPlanner* planner = new rrt_planning::NHPlanner(std::string(""), costmap_ros);
		return planner;
	}
}
