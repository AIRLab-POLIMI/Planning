#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

#include "rrt_planning/distance/Distance.h"
#include "rrt_planning/extenders/ExtenderFactory.h"
#include "rrt_planning/visualization/Visualizer.h"
#include "rrt_planning/AbstractPlanner.h"

namespace rrt_planning
{

class RRTStarPlanner : public AbstractPlanner
{
public:

    RRTStarPlanner();
    RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::chrono::duration<double> t);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

    virtual ~RRTStarPlanner();

private:
    bool newState(const Eigen::VectorXd& xRand,
                  const Eigen::VectorXd& xNear,
                  Eigen::VectorXd& xNew,
                  std::vector<Eigen::VectorXd>& primitives,
                  double& cost);

    bool collisionFree(const Eigen::VectorXd& x0, 
                       const Eigen::VectorXd& xGoal,
                       std::vector<Eigen::VectorXd>& primitives,
                       double& cost);

    Eigen::VectorXd convertPose(const geometry_msgs::PoseStamped& pose);

    void publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
                     const ros::Time& stamp);


private:
    Map* map;
    Distance* distance;

    int K;
    double deltaX;
    double greedy;
    double gamma;
    int dimension;
    int knn;

    ExtenderFactory extenderFactory;

    Visualizer visualizer;

};

}
