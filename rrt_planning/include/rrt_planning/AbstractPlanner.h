#ifndef INCLUDE_ABSTRACTPLANNER_H_
#define INCLUDE_ABSTRACTPLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <chrono>
#include <Eigen/Dense>

//#define DEBUG_CONF
//#define PRINT_CONF
//#define VIS_CONF

namespace rrt_planning
{

class AbstractPlanner : public nav_core::BaseGlobalPlanner
{
public:
    AbstractPlanner(){}
    AbstractPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){}

    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) = 0;
    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) = 0;

	double getElapsedTime();
	double getPathLength();
    double getRoughness();
    double getFirstLength();
    double getFirstRoughness();

    std::vector<Eigen::VectorXd> getPath();
    std::vector<Eigen::VectorXd> getFirstPath();


    virtual ~AbstractPlanner();

protected:
	bool timeOut();
	void computeFirstRoughness(std::vector<Eigen::VectorXd>& path);
	void computeRoughness(std::vector<Eigen::VectorXd>& path);
	void computeFirstLength(std::vector<Eigen::VectorXd>& path);
	void computeLength(std::vector<Eigen::VectorXd>& path);


protected:
	std::chrono::steady_clock::time_point t0;
	std::chrono::duration<double> Tmax;
	std::chrono::duration<double> Tcurrent;
	double length;
    double roughness;
    double first_length;
    double first_roughness;

    std::vector<Eigen::VectorXd> first_path;
    std::vector<Eigen::VectorXd> final_path;

};

}



#endif /* INCLUDE_ABSTRACTPLANNER_H_ */
