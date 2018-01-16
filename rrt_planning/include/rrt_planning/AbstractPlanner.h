#ifndef INCLUDE_ABSTRACTPLANNER_H_
#define INCLUDE_ABSTRACTPLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <chrono>

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
    int getDeadActions();

    virtual ~AbstractPlanner();

protected:
	bool timeOut();


protected:
	std::chrono::steady_clock::time_point t0;
	std::chrono::duration<double> Tmax;
	std::chrono::duration<double> Tcurrent;
	double length;
    int dead;

};

}



#endif /* INCLUDE_ABSTRACTPLANNER_H_ */
