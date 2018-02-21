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

    private_nh.param("K", K, 1);
    private_nh.param("deltaX", deltaX, 0.5);
    private_nh.param("greedy", greedy, 0.1);
    //private_nh.param("gamma", gamma, 1.5);
    private_nh.param("dimension", dimension, 3);
    private_nh.param("knn", knn, 10);

    extenderFactory.initialize(private_nh, *map, *distance);
    visualizer.initialize(private_nh);
    //gamma = pow(2.0,4.0)*exp(1.0 + 1.0/3.0);
    double t;
    private_nh.param("Tmax", t, 180.0);
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
    std::set<RRTNode*> ending_nodes;
    double gamma_knn = 6.0;
    double gamma_r = 23.0;
    double min_radius = double(K)/2.0;

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

        if(!plan_found && RandomGenerator::sampleEvent(greedy))
        {
            xRand = xGoal;
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

        auto* node = rrt.searchNearestNode(xRand);
        VectorXd xNew;
        vector<VectorXd> primitives;
        double cost = 0;

        if(newState(xRand, node->x, xNew, primitives, cost))
        {
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
            maxCost = rrt.computeCost(node) + cost;
            vector<VectorXd> new_primitives;
            vector<VectorXd> tmp_primitives;
            double c = 0;
            double c_tmp;

            for(auto n : neighbors)
            {
                tmp_primitives.clear();
                c_tmp = 0;
                if(collisionFree(n->x, xNew, tmp_primitives, c_tmp))
                {
                    newCost = rrt.computeCost(n) + c_tmp;
                    if(newCost < maxCost)
                    {
                        maxCost = newCost;
                        father = n;
                        new_primitives = tmp_primitives;
                        c = c_tmp;
                    }
                }
             }

            rrt.addNode(father, xNew, primitives, cost);

#ifdef VIS_CONF
            visualizer.addSegment(father->x, xNew);
#endif

            //Rewire tree
            RRTNode* newNode = rrt.getPointer();
            double n_cost = rrt.computeCost(newNode);
            new_primitives.clear();
            c = 0;

            for(auto n : neighbors)
            {
                tmp_primitives.clear();
                c_tmp = 0;
                if(collisionFree(xNew, n->x, tmp_primitives, c_tmp))
                {
                    newCost = n_cost + c_tmp;
                    if(newCost < rrt.computeCost(n))
                    {
                        n->father = newNode;
                        new_primitives = tmp_primitives;
                        c = c_tmp;
                        newNode->childs.push_back(n);
#ifdef VIS_CONF
                        visualizer.addSegment(xNew, n->x);
#endif
                    }
                }
            }


            if(extenderFactory.getExtender().isReached(xNew, xGoal))
            {
                last = rrt.getPointer();
                if(!plan_found)
                {
                    Tcurrent = chrono::steady_clock::now() - t0;
                    ROS_FATAL_STREAM("first plan found in " << Tcurrent.count() << " seconds");

#ifdef DEBUG_CONF

                double cost = rrt.computeCost(last);

                auto&& path = rrt.getPathToLastNode();
                computeLength(path);
                ROS_FATAL_STREAM("first cost: " << cost);
                ROS_FATAL_STREAM("first length: " << length);
                plan.clear();
                publishPlan(path, plan, start.header.stamp);
#endif
#ifdef VIS_CONF
                visualizer.displayPlan(plan);
                visualizer.flush();
#endif
                }
                plan_found = true;
                ending_nodes.insert(last);
            }
        }

    }

    if(plan_found)
    {
        Tcurrent = chrono::steady_clock::now() - t0;
        double length_tmp = -1;
        double cost = -1;
        for(auto p : ending_nodes)
        {
            auto&& path = rrt.getPathToLastNode(p);
            computeLength(path);
            double l = getPathLength();
            double c = rrt.computeCost(p);
            if((length_tmp == -1) || (length_tmp != -1 && c < cost))
            {
                length_tmp = l;
                cost = c;
                last = p;
            }
        }

        auto&& path = rrt.getPathToLastNode(last);
        length = length_tmp;
#ifdef PRINT_CONF
        ROS_FATAL_STREAM("cost: " << cost);
        ROS_FATAL_STREAM("length: " << length);
#endif
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
                          VectorXd& xNew, vector<VectorXd>& primitives, double& cost)
{
    return extenderFactory.getExtender().steer(xNear, xRand, xNew, primitives, cost);
}

bool RRTStarPlanner::collisionFree(const VectorXd& x0, const VectorXd& xGoal, vector<VectorXd>& primitives, double& cost)
{
    return extenderFactory.getExtender().check(x0, xGoal, primitives, cost);
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
