#include <fstream>
#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include <stdlib.h>

#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include "rrt_planning/AbstractPlanner.h"
#include "rrt_planning/NHPlanner.h"
#include "rrt_planning/NHPlannerL2.h"
#include "rrt_planning/RRTPlanner.h"
#include "rrt_planning/RRTStarPlanner.h"
#include "rrt_planning/ThetaStarRRTPlanner.h"
#include "rrt_planning/VoronoiRRTPlanner.h"


using namespace rrt_planning;
using namespace std;

AbstractPlanner* getPlanner(const string& name, costmap_2d::Costmap2DROS* costmap_ros, const string& t);
bool parse(const std::string& conf, geometry_msgs::PoseStamped& start_pose,
                         geometry_msgs::PoseStamped& goal_pose);
void save(const std::string& filename, const std::string& conf, double t, double l, double r,
                        std::vector<Eigen::VectorXd> plan);

void saveRRTStar(const std::string& filename, const std::string& conf, rrt_planning::AbstractPlanner* planner, double tmax);

int main(int argc, char** argv)
{
    string planner_name = argv[1];
    string map = argv[2];
    string conf = argv[3];
    string row = argv[4];
    string deadline = argv[5];
    string dir = argv[6];
    string node_name = planner_name + "_" + map + "_" + row;
	ROS_FATAL_STREAM("Node name: " << node_name);

    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~/");

    //Costmap inizialization magics
    tf::TransformListener tf_(ros::Duration(10));
    costmap_2d::Costmap2DROS* costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    costmap_ros->pause();

    //Convert start and goal
    geometry_msgs::PoseStamped start_pose, goal_pose;
    std::vector<geometry_msgs::PoseStamped> plan;
    bool valid = parse(conf, start_pose, goal_pose);

    if(!valid)
    {
        std::ofstream f;
        f.open(dir+node_name + string(".log"));
        f << "INVALID CONFIGURATION: " << conf << "\n";
        f.close();
        return 0;
    }

    //Launch planner
    AbstractPlanner* planner = getPlanner(planner_name, costmap_ros, deadline);
    bool result = planner->makePlan(start_pose, goal_pose, plan);

    double tmax = atof(deadline.c_str());
    if(result)
    {
        if(planner_name == "rrt_star")
        {
            saveRRTStar(dir + node_name, conf, planner, tmax);
        }
        else
        {
            save(dir + node_name, conf, planner->getElapsedTime(), planner->getPathLength(), planner->getRoughness(), planner->getPath());
        }
    }
     else
    {
        if(planner_name == "rrt_star")
        {
            std::ofstream ff;
            ff.open(dir+node_name + "_first" + string(".log"));
            ff << "configuration " << conf << "\n";

            ff << "NO_PATH_FOUND_WITHIN_DEADLINE" << "\n";

            ff.close();

            std::ofstream fl;
            fl.open(dir+node_name + "_last" + string(".log"));
            fl << "configuration " << conf << "\n";

            fl << "NO_PATH_FOUND_WITHIN_DEADLINE" << "\n";

            fl.close();
        }
        else
        {
            std::ofstream f;
            f.open(dir+node_name + string(".log"));
            f << "configuration " << conf << "\n";

            if(planner->getElapsedTime() < tmax)
                f << "FAILED_TO_FIND_PLAN" << "\n";
            else
                f << "NO_PATH_FOUND_WITHIN_DEADLINE" << "\n";

            f.close();
        }
    }

    private_nh.deleteParam("");
	ROS_FATAL_STREAM(node_name + " plan found? " << result);

   //Clean up
    if(planner)
        delete planner;
    if(costmap_ros)
        delete costmap_ros;

    return 0;
}

bool parse(const string& conf, geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& goal_pose)
{
    stringstream s(conf);
    string segment;
    vector<string> seglist;
    while(getline(s, segment, '_'))
    {
       seglist.push_back(segment);
    }

    if(seglist.size() != 8)
    return false;

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

    start_pose.header.frame_id = std::string("/map");
    goal_pose.header.frame_id = std::string("/map");

    return true;
}

void save(const std::string& filename, const std::string& conf, double t, double l, double r, std::vector<Eigen::VectorXd> plan)
{
    std::ofstream f;
    f.open(filename + string(".log"));
    std::string d = string("_");

    f << "configuration "<< conf << "\n";
    f << "length " << l << "\n";
    f << "time "<< t << "\n";
    f << "roughness " << r << "\n";

    for(auto p : plan)
    {
        f << p(0) << d << p(1) << d
          << p(2) << "\n";
    }

    f.close();
}

void saveRRTStar(const std::string& filename, const std::string& conf, AbstractPlanner* planner, double tmax)
{
    std::ofstream ff;
    ff.open(filename + "_first" + string(".log"));
    std::string d = string("_");

    double l = planner->getFirstLength();
    double t = planner->getElapsedTime();
    double r = planner->getFirstRoughness();
    std::vector<Eigen::VectorXd> plan = planner->getFirstPath();

    ff << "configuration "<< conf << "\n";
    ff << "length " << l << "\n";
    ff << "time "<< t << "\n";
    ff << "roughness " << r << "\n";

    for(auto p : plan)
    {
        ff << p(0) << d << p(1) << d
          << p(2) << "\n";
    }

    ff.close();

    std::ofstream fl;
    fl.open(filename + "_last" + string(".log"));

    l = planner->getPathLength();
    r = planner->getRoughness();
    plan = planner->getPath();

    fl << "configuration "<< conf << "\n";
    fl << "length " << l << "\n";
    fl << "time "<< tmax << "\n";
    fl << "roughness " << r << "\n";

    for(auto p : plan)
    {
        fl << p(0) << d << p(1) << d
          << p(2) << "\n";
    }

    fl.close();
}

AbstractPlanner* getPlanner(const string& name, costmap_2d::Costmap2DROS* costmap_ros, const string& t)
{
    chrono::duration<double> Tmax(stod(t));
    if(name == "nh")
    {
        NHPlanner* planner = new NHPlanner(string(""), costmap_ros, Tmax);
        return planner;
    }
    else if(name == "nh_l2")
    {
        NHPlannerL2* planner = new NHPlannerL2(string(""), costmap_ros, Tmax);
        return planner;
    }
    else if(name == "rrt")
    {
        RRTPlanner* planner = new RRTPlanner(string(""), costmap_ros, Tmax);
        return planner;
    }
    else if(name == "rrt_star")
    {
        RRTStarPlanner* planner = new RRTStarPlanner(string(""), costmap_ros, Tmax);
        return planner;
    }
    else if(name == "theta_star_rrt")
    {
        ThetaStarRRTPlanner* planner = new ThetaStarRRTPlanner(string(""), costmap_ros, Tmax);
        return planner;
    }
    else if(name == "voronoi_rrt")
    {
        VoronoiRRTPlanner* planner = new VoronoiRRTPlanner(string(""), costmap_ros, Tmax);
        return planner;
    }
    else
    {
        ROS_FATAL("Planner not impleted, aborting");
        exit(0);
    }
}
