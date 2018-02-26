#ifndef INCLUDE_RRT_PLANNING_VORONOI_H_
#define INCLUDE_RRT_PLANNING_VORONOI_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#include "dynamicvoronoi.h"

#include "rrt_planning/map/ROSMap.h"
#include "rrt_planning/grid/Grid.h"
#include "rrt_planning/theta_star/PriorityQueue.h"
#include "rrt_planning/visualization/Visualizer.h"


namespace rrt_planning
{

class Voronoi : public nav_core::BaseGlobalPlanner
{

public:
    Voronoi();
    Voronoi(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;
	bool initVoronoi();
	bool findPath(std::vector<Cell>&, int start_x, int start_y, int goal_x, int goal_y,
				  bool voronoi_cell, bool stop_at_voronoi);

    ~Voronoi();

private:
    void publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
                     const ros::Time& stamp, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
	void displayVoronoiGrid();
	std::vector<Cell> smoothPath(std::vector<Cell>* path);

private:

    ros::Publisher pub;
    static const Cell S_NULL;
	double discretization;

    ROSMap* map;
    Grid* grid;

	DynamicVoronoi voronoi_;

    Cell s_start;
    Cell s_goal;

    int sizeX;
	int sizeY;

    Visualizer visualizer;
};

}

#endif /* INCLUDE_RRT_PLANNING_VORONOI_H_ */
