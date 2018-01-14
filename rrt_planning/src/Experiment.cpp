#include <fstream>
#include <iostream>
#include <string>
#include <fstream>
#include <chrono>

#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include "rrt_planning/NHPlanner.h"
#include "rrt_planning/AbstractPlanner.h"

using namespace rrt_planning;
using namespace std;

AbstractPlanner* getPlanner(const string& name, costmap_2d::Costmap2DROS* costmap_ros, const string& t);
void parse(const std::string& conf, geometry_msgs::PoseStamped& start_pose,
				 geometry_msgs::PoseStamped& goal_pose);
void save(const std::string& filename, const std::string& conf, double t,
				double l, std::vector<geometry_msgs::PoseStamped>& plan);


int main(int argc, char** argv)
{
	ROS_FATAL_STREAM("Starting");
	string planner_name = argv[1];
	ROS_FATAL_STREAM("1: " << planner_name);
	string map = argv[2];
	ROS_FATAL_STREAM("2: " << map);
	string conf = argv[3];
	ROS_FATAL_STREAM("3: " << conf);
	string row = argv[4];
	ROS_FATAL_STREAM("4: " << row);
	string deadline = argv[5];
	ROS_FATAL_STREAM("5: " << deadline);
	string dir = argv[6];
	ROS_FATAL_STREAM("6: " << dir);
	string node_name = planner_name + "_" + map + "_" + row;
	ROS_FATAL_STREAM("node_name: " << node_name);

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
	AbstractPlanner* planner = getPlanner(planner_name, costmap_ros, deadline);
	ROS_FATAL_STREAM("Started planner: " + planner_name);
	double test = planner->getPathLength();
	ROS_FATAL_STREAM("test: " << test);
	bool result = planner->makePlan(start_pose, goal_pose, plan);
	save(dir + node_name, conf, planner->getElapsedTime(), planner->getPathLength(), plan);
	private_nh.deleteParam("");

	ROS_FATAL_STREAM("Plan found: " << result);

	return 0;
}

void parse(const string& conf, geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& goal_pose)
{
	ROS_FATAL("start string split");
	stringstream s(conf);
	string segment;
	vector<string> seglist;
	while(getline(s, segment, '_'))
	{
	   seglist.push_back(segment);
	}
	ROS_FATAL("end string split");

	start_pose.pose.position.x = stod(seglist[0]);
	start_pose.pose.position.y = stod(seglist[1]);
	start_pose.pose.position.z = 0;
	start_pose.pose.orientation.x = 0;
	start_pose.pose.orientation.y = 0;
	start_pose.pose.orientation.z = stod(seglist[2]);
	start_pose.pose.orientation.w = stod(seglist[3]);

	goal_pose.pose.position.x = stod(seglist[4]);
	goal_pose.pose.position.y = stod(seglist[5]);
	goal_pose.pose.position.z = 0;
	goal_pose.pose.orientation.x = 0;
	goal_pose.pose.orientation.y = 0;
	goal_pose.pose.orientation.z = stod(seglist[6]);
	goal_pose.pose.orientation.w = stod(seglist[7]);

	/*start_pose.pose.position.x = 24.6607;
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
	goal_pose.pose.orientation.w = 0.225188;*/

}

void save(const std::string& filename, const std::string& conf, double t, double l, std::vector<geometry_msgs::PoseStamped>& plan)
{
	std::ofstream f;
	f.open(filename + std::string(".log"));

	std::string d = std::string("_");

	//Write configuration
	f << conf << "\n";

	//Write execution Time
	f << t << "\n";

	//Write path length
	f << l << "\n";

	//Write path points
	for(auto p : plan)
	{
		f << p.pose.position.x << d << p.pose.position.y << d
		  << p.pose.orientation.z << d << p.pose.orientation.w << "\n";
	}

	f.close();
}


AbstractPlanner* getPlanner(const string& name, costmap_2d::Costmap2DROS* costmap_ros, const string& t)
{
	chrono::duration<double> Tmax(stod(t));
	if(name == "nh")
	{
		NHPlanner* planner = new NHPlanner(string(""), costmap_ros);
		return planner;
	}
}
