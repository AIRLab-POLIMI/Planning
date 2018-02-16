#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

#include "rrt_planning/RRTStarPlanner.h"

#include "rrt_planning/map/ROSMap.h"
#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/extenders/MotionPrimitivesExtender.h"
#include "rrt_planning/utils/RandomGenerator.h"
#include "rrt_planning/rrt/RRT.h"

#include <thread>
#include <chrono>


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

RRTStarPlanner::RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::chrono::duration<double> t)
{
    initialize(name, costmap_ros);
    Tmax = t;
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
    //private_nh.param("gamma", gamma, 1.5);
    private_nh.param("dimension", dimension, 3);
    private_nh.param("knn", knn, 10);

    extenderFactory.initialize(private_nh, *map, *distance);
    visualizer.initialize(private_nh);
    //gamma = pow(2.0,4.0)*exp(1.0 + 1.0/3.0);
    double t;
    private_nh.param("Tmax", t, 300.0);
    Tmax = std::chrono::duration<double>(t);
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

    double gamma_knn = 6.0;
    double gamma_r = 23.0;
    double min_radius = 1.0;

    RRT rrt(distance, x0);
#ifdef PRINT_CONF
    ROS_INFO("Planner started");
#endif
#ifdef VIS_CONF
    visualizer.clean();
#endif
    t0 = chrono::steady_clock::now();

    while(!timeOut())
    {
        VectorXd xRand;

        if(RandomGenerator::sampleEvent(greedy))
        {
            xRand = xGoal;
            //ROS_FATAL_STREAM("sampled goal " << i << " times");
            //ROS_FATAL_STREAM("number of samples so far: " << s);
        }
        else
        {
            do
            {
                xRand = extenderFactory.getKinematicModel().sampleOnBox(map->getBounds());
            } while(!map->isFree(xRand));
        }
#ifdef VIS_CONF
        visualizer.addPoint(xRand);
#endif

        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        auto* node = rrt.searchNearestNode(xRand);
        VectorXd xNew;

        if(newState(xRand, node->x, xNew))
        {
            //visualizer.addUpdate(xRand, node->x);
            //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            std::vector<RRTNode*> neighbors;
            double maxCost, newCost;
            RRTNode* father = node;
            int cardinality = rrt.getLength();
            double radius = gamma_r*pow(log(cardinality)/double(cardinality), double(1)/double(dimension));
            double ray = min(min_radius, radius);
            double knearest = gamma_knn*log(cardinality);

            //Find all samples inside ray or the k-nearest neighbors
            neighbors = rrt.findNeighbors(xNew, knearest, ray);

            //Compute cost of getting there
            maxCost = rrt.computeCost(node) + distance(node->x, xNew);
            for(auto n : neighbors)
            {
                if(collisionFree(n->x, xNew))
                {
                    newCost = rrt.computeCost(n) + distance(xNew, n->x);
                    if(newCost < maxCost)
                    {
                        maxCost = newCost;
                        father = n;
                    }
                }
             }

            rrt.addNode(father, xNew);

#ifdef VIS_CONF
            visualizer.addSegment(father->x, xNew);
#endif

            //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            //Rewire tree
            RRTNode* newNode = rrt.getPointer();
            double n_cost = rrt.computeCost(newNode);
            for(auto n : neighbors)
            {
                if(collisionFree(xNew, n->x))
                {
                    newCost = n_cost + distance(n->x, xNew);
                    if(newCost < rrt.computeCost(n))
                    {
                        n->father = newNode;
                        newNode->childs.push_back(n);
#ifdef VIS_CONF
                        visualizer.addSegment(xNew, n->x);
#endif
                    }
                }
            }


            if(!plan_found && extenderFactory.getExtender().isReached(xNew, xGoal))
            {
                last = rrt.getPointer();

                length = rrt.computeLength(last);
                double cost = rrt.computeCost(last);
                ROS_FATAL_STREAM("first cost: " << cost);
                ROS_FATAL_STREAM("first length: " << length);
                auto&& path = rrt.getPathToLastNode();
                plan.clear();
                publishPlan(path, plan, start.header.stamp);
#ifdef VIS_CONF
                visualizer.displayPlan(plan);
                visualizer.flush();
#endif

                plan_found = true;
            }
        }

    }

    if(plan_found)
    {
        Tcurrent = chrono::steady_clock::now() - t0;
        length = rrt.computeLength(last);
        double cost = rrt.computeCost(last);
        ROS_FATAL_STREAM("refined cost: " << cost);
        ROS_FATAL_STREAM("refined length: " << length);
        auto&& path = rrt.getPathToLastNode(last);
        computeRoughness(path);
        plan.clear();
        publishPlan(path, plan, start.header.stamp);
#ifdef VIS_CONF
        visualizer.displayPlan(plan);
        visualizer.flush();
#endif
#ifdef PRINT_CONF
        ROS_INFO("Plan found");
#endif

        return true;
    }
#ifdef VIS_CONF
    visualizer.flush();
#endif
#ifdef PRINT_CONF
    ROS_WARN_STREAM("Failed to find a plan in " << K << " RRT Star iterations");
#endif
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
