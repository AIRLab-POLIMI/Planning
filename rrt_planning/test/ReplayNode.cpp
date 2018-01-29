#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>

#define CONF 10

geometry_msgs::PoseWithCovarianceStamped start;
geometry_msgs::PoseStamped goal;

ros::Publisher startPub;
ros::Publisher goalPub;
std::ofstream f;
int count;

void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	start.header.seq = msg->header.seq;
	start.header.frame_id = msg->header.frame_id;
	start.pose = msg->pose;
	}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	goal.header.frame_id = msg->header.frame_id;
	goal.pose = msg->pose;

	f << start.pose.pose.position.x << "_" << start.pose.pose.position.y << "_" << start.pose.pose.orientation.z << "_" << start.pose.pose.orientation.w << "_"
	  << goal.pose.position.x << "_" << goal.pose.position.y << "_" << goal.pose.orientation.z << "_" << goal.pose.orientation.w << "\n";

	count++;
	std::cout << count << '\n';
	if(count == CONF)
	{
		f.close();
		exit(0);
	}


}

void saveCallback(const std_msgs::String::ConstPtr& msg)
{
	start.header.stamp = ros::Time::now();
	goal.header.stamp = ros::Time::now();
	startPub.publish(start);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	goalPub.publish(goal);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "replay_node");

	ros::NodeHandle n;
	ros::Subscriber s_sub = n.subscribe("/initialpose", 10, startCallback);
	ros::Subscriber g_sub = n.subscribe("/move_base_simple/goal", 10, goalCallback);
	ros::Subscriber save_sub = n.subscribe("/nomura", 10, saveCallback);

	startPub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
	goalPub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

	ros::Rate rate(10);
	f.open("/home/reb/Scrivania/office.exp");
	count = 0;

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
}
