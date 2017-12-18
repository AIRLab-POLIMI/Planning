#ifndef INCLUDE_NHPLANNER_H_
#define INCLUDE_NHPLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

#include "rrt_planning/distance/Distance.h"
#include "rrt_planning/extenders/ExtenderFactory.h"
#include "rrt_planning/visualization/Visualizer.h"
#include "rrt_planning/nh/CornerIndex.h"
#include "rrt_planning/nh/OpenList.h"
#include "rrt_planning/map/SGMap.h"


namespace rrt_planning
{

struct CmpReached
{
  bool operator()(const Eigen::VectorXd a, const Eigen::VectorXd b) const
  {
      return ((a(0) < b(0)) ||(a(0) == b(0) && a(1) < b(1)) ||
               (a(0) == b(0) && a(1) == b(1) && a(2) < b(2)));
  }
};

class NHPlanner : public nav_core::BaseGlobalPlanner
{
public:

    NHPlanner();
    NHPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

    virtual ~NHPlanner();

private:
    bool newState(const Eigen::VectorXd& xSample, const Eigen::VectorXd& xNear,
                  Eigen::VectorXd& xNew, double length, Distance& distance);
    Node* steer(Node* current, const Eigen::VectorXd& xCorner, Distance& distance);
    Node* park(Node* current, Distance& distance);

    Eigen::VectorXd convertPose(const geometry_msgs::PoseStamped& pose);

    void publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
                     const ros::Time& stamp);

    void addOpen(Node* node, const Action& action, Distance& distance);

    void addSubgoal(Node* node, const Action& action, Distance& distance);

    std::vector<Action> findAction(Node* node, const Action& action, Distance& distance, std::vector<Triangle>& triangles);

    std::vector<Eigen::VectorXd> retrievePath(Node* node);
    Eigen::VectorXd sampleCorner(const Eigen::VectorXd& corner, bool cw);
    void addGlobal(const Eigen::VectorXd& node, const Eigen::VectorXd& action, const Eigen::VectorXd& parent);
    bool insideGlobal(const Eigen::VectorXd& p, bool subgoal);

private:
    Map* rosmap;
    SGMap* map;
    Distance* l2dis;
    Distance* l2thetadis;
    Distance* thetadis;
    OpenList open;
    Action target;
    std::map<Eigen::VectorXd, Node*, rrt_planning::CmpReached> reached;
    std::vector<Triangle> global_closed;

    double deltaX;
    double deltaTheta;
    int count;
    int k;


    ExtenderFactory extenderFactory;
    Visualizer visualizer;

};

}



#endif /* INCLUDE_NHPLANNER_H_ */
