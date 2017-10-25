
#ifndef INCLUDE_RRT_PLANNING_VORONOIRRTPLANNER_H_
#define INCLUDE_RRT_PLANNING_VORONOIRRTPLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

#include "rrt_planning/distance/Distance.h"
#include "rrt_planning/extenders/ExtenderFactory.h"
#include "rrt_planning/visualization/Visualizer.h"

#include <voronoi_planner/planner_core.h>

namespace rrt_planning{

    class VoronoiRRTPlanner : public nav_core::BaseGlobalPlanner{
        public:

            VoronoiRRTPlanner();
            VoronoiRRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            virtual ~VoronoiRRTPlanner();

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
            bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) override;

        private:

            bool newState(const Eigen::VectorXd& xRand,
                          const Eigen::VectorXd& xNear,
                          Eigen::VectorXd& xNew);
            Eigen::VectorXd convertPose(const geometry_msgs::PoseStamped& pose);
            void publishPlan(std::vector<Eigen::VectorXd>& path,
                             std::vector<geometry_msgs::PoseStamped>& plan,
                             const ros::Time& stamp);

            Map* map;
            Distance* distance;

            int K;
            double deltaX;
            double laneWidth;
            double greedy;
            double deltaTheta;

            voronoi_planner::VoronoiPlanner* voronoiPlanner;

            ExtenderFactory extenderFactory;

            Visualizer visualizer;
    };
}

#endif /* INCLUDE_RRT_PLANNING_VORONOIRRTPLANNER_H_ */
