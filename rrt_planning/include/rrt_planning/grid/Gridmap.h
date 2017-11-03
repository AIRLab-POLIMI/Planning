#ifndef INCLUDE_RRT_PLANNING_GRID_GRIDMAP_H_
#define INCLUDE_RRT_PLANNING_GRID_GRIDMAP_H_

#include <vector>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include "rrt_planning/map/Map.h"
#include "rrt_planning/grid/Cell.h"

namespace rrt_planning
{

enum Modes {collision_points, exit_point, is_los};

class Gridmap
{

public:
    Gridmap(Map& map, double gridResolution);
    Cell computeMiddle(const Cell& a, const Cell& b, const Cell& action);
    Cell findSubgoal(const Cell& a, const Cell& b);
    std::vector<Cell> getPoints(const Cell& a, const Cell& b, bool clockwise);
    std::vector<Cell> findSubgoals(const Cell& entry_point, const Cell& exit_point);
    bool los(const Cell& a, const Cell& b, std::vector<Cell>& collision, Modes mode);
    bool isCorner(const Cell& cell);
    double distance(const Cell& a, const Cell& b);
    Cell followObstacle(const Cell& action, const Cell& subgoal);


private:
    void computeBounds();
    bool checkFree(const Cell& cell);
    bool clockwise(const Cell& start, const Cell& middle, int x, int y);
    bool isSameDirection(const Cell& a, const Cell& b, const Cell& s);
    bool isInside(const Eigen::Vector3d& point);
    Eigen::Vector3d findLine(const Cell& a, const Cell& b);
    std::vector<Cell> findIntersection(const Eigen::Vector3d& line);
    Cell convertPose(const geometry_msgs::PoseStamped& msg);
    bool isFree(const Cell& s);
    Eigen::VectorXd toMapPose(int X, int Y);


private:

    Map& map;
    double gridResolution;	
    int minY, minX, maxX, maxY;
    std::vector<Eigen::Vector3d> lines;

};

}
#endif /* INCLUDE_RRT_PLANNING_GRID_GRIDMAP_H_ */
