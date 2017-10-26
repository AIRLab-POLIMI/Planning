#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

#include "rrt_planning/RRTStarPlanner.h"

#include "rrt_planning/map/ROSMap.h"
#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/extenders/MotionPrimitivesExtender.h"
#include "rrt_planning/utils/RandomGenerator.h"
#include "rrt_planning/rrt/RRT.h"

using namespace Eigen;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::RRTStarPlanner, nav_core::BaseGlobalPlanner);

using namespace std;

//Default Constructor
namespace rrt_planning
{
RRTStarPlanner::RRTStarPlanner ()
{
    K = 0;
    deltaX = 0;
    greedy = 0;
    gamma = 0;
    dimension = 0;
    knn = 0;

    map = nullptr;
    distance = nullptr;
}

RRTStarPlanner::RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    map = new ROSMap(costmap_ros);
    distance = new L2ThetaDistance();

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("iterations", K, 30000);
    private_nh.param("deltaX", deltaX, 0.5);
    private_nh.param("greedy", greedy, 0.1);
    private_nh.param("gamma", gamma, 1.5);
    private_nh.param("dimension", dimension, 3);
    private_nh.param("knn", knn, 10);

    extenderFactory.initialize(private_nh, *map, *distance);
    visualizer.initialize(private_nh);
}

bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan)
{
    Distance& distance = *this->distance;

    VectorXd&& x0 = convertPose(start);
    VectorXd&& xGoal = convertPose(goal);
    RRTNode* last;
    bool plan_found = false;

    RRT rrt(distance, x0);

    ROS_INFO("Planner started");

    visualizer.clean();


    for(unsigned int i = 0; i < K || (i >= K && !plan_found); i++)
    {
        ROS_WARN_STREAM("K: " << i);
        VectorXd xRand;

        if(RandomGenerator::sampleEvent(greedy))
            xRand = xGoal;
        else
            xRand = extenderFactory.getKinematicModel().sampleOnBox(map->getBounds());

        visualizer.addPoint(xRand);

        auto* node = rrt.searchNearestNode(xRand);
        VectorXd xNew;

        if(newState(xRand, node->x, xNew))
        {
            //rrt.addNode(node, xNew);
            std::vector<RRTNode*> neighbors;
            double maxCost, newCost;
            RRTNode* father = node;
            int cardinality = rrt.getLength();
            double radius = gamma*pow(log(cardinality)/cardinality, 1/dimension);

            //Find all samples inside ray
            neighbors = rrt.findNeighbors(xNew, knn, radius);
           //neighbors.push_back(node->father);

            //Compute cost of getting there
            maxCost = rrt.computeCost(node) + distance(node->x, xNew);

            for(auto n : neighbors)
            {
                if(collisionFree(n->x, xNew))
                {
                    newCost = rrt.computeCost(n) + distance(n->x, xNew);
                    if(newCost < maxCost)
                    {
                        maxCost = newCost;
                        father = n;
                    }
                }
             }

            rrt.addNode(father, xNew);
            visualizer.addSegment(father->x, xNew);

            //Rewire tree
            RRTNode* newNode = rrt.getPointer();
            double cost = rrt.computeCost(newNode);

            for(auto n : neighbors)
            {
                if(collisionFree(xNew, n->x))
                {
                    newCost = cost + distance(xNew, n->x);
                    if(newCost < rrt.computeCost(n))
                    {
                        n->father = newNode;
                        visualizer.addSegment(xNew, n->x);
                        ROS_INFO("rewiring");
                    }
                }
            }

            if(distance(xNew, xGoal) < deltaX)
            {
                last = rrt.getPointer();
                plan_found = true;
                ROS_INFO("AT LAST");
            }
        }

    }

    if(plan_found)
    {

        auto&& path = rrt.getPathToLastNode(last);
        publishPlan(path, plan, start.header.stamp);

        visualizer.displayPlan(plan);
        visualizer.flush();

        ROS_INFO("Plan found");

        return true;
    }

    visualizer.flush();

    ROS_WARN_STREAM("Failed to find a plan in " << K << " RRT Star iterations");
    return false;

}


bool RRTStarPlanner::newState(const VectorXd& xRand,
                          const VectorXd& xNear,
                          VectorXd& xNew)
{
    return extenderFactory.getExtender().compute(xNear, xRand, xNew);
}

bool RRTStarPlanner::collisionFree(const VectorXd& x0, const VectorXd& xGoal)
{
    return extenderFactory.getExtender().check(x0, xGoal);
}

VectorXd RRTStarPlanner::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& q_ros = msg.pose.orientation;
    auto& t_ros = msg.pose.position;

    Quaterniond q(q_ros.w, q_ros.x, q_ros.y, q_ros.z);

    Vector3d theta = q.matrix().eulerAngles(0, 1, 2);

    VectorXd x(3);
    x << t_ros.x, t_ros.y, theta(2);

    return x;
}

void RRTStarPlanner::publishPlan(std::vector<VectorXd>& path,
                             std::vector<geometry_msgs::PoseStamped>& plan, const ros::Time& stamp)
{
    for(auto x : path)
    {
        geometry_msgs::PoseStamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        msg.pose.position.x = x(0);
        msg.pose.position.y = x(1);
        msg.pose.position.z = 0;

        Matrix3d m;
        m = AngleAxisd(x(2), Vector3d::UnitZ())
            * AngleAxisd(0, Vector3d::UnitY())
            * AngleAxisd(0, Vector3d::UnitX());

        Quaterniond q(m);

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        plan.push_back(msg);
    }
}


RRTStarPlanner::~RRTStarPlanner()
{
    if(distance)
        delete distance;

    if(map)
        delete map;

}


};
