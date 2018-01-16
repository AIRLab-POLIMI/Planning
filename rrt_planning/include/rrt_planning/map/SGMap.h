#ifndef INCLUDE_RRT_PLANNING_MAP_SGMAP_H_
#define INCLUDE_RRT_PLANNING_MAP_SGMAP_H_

#include "rrt_planning/map/Map.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/costmap_2d.h"
#include "rrt_planning/nh/Triangle.h"

namespace rrt_planning
{

class SGMap
{
public:
    SGMap(Map& map);
    void initialize(ros::NodeHandle& nh);

    bool collisionPoints(const Eigen::VectorXd& a, const Eigen::VectorXd& b, std::vector<Eigen::VectorXd>& actions);
    Eigen::VectorXd exitPoint(const Eigen::VectorXd& current, const Eigen::VectorXd& middle, bool cw);
    std::vector<Eigen::VectorXd> infiniteExitPoint(const Eigen::VectorXd& current, const Eigen::VectorXd& middle, bool cw);
    void forcedUpdate(const Eigen::VectorXd& a, const Eigen::VectorXd& b, std::vector<Eigen::VectorXd>& actions);

    Eigen::VectorXd computeMiddle(const Eigen::VectorXd& a, const Eigen::VectorXd& b);
    bool clockwise(const Eigen::VectorXd& a, const Eigen::VectorXd& b);
    bool isCorner(const Eigen::VectorXd& current);
    bool isTrueCornerWOW(const Eigen::VectorXd& current);
    bool followObstacle(const Eigen::VectorXd& current, const Eigen::VectorXd& a, std::vector<Eigen::VectorXd>& actions);

    virtual ~SGMap();


private:
  Map& map;

  //corner parameters
  int discretization;
  int k_los;
  double ray;
  double threshold;
  double corner_step;

  //los parameters
  double macro;
  double micro;
  double step;

};

}

#endif /* INCLUDE_RRT_PLANNING_MAP_SGMAP_H_ */
