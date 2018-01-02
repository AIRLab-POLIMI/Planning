#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <chrono>
#include <thread>

#include "rrt_planning/NHPlanner.h"

#include "rrt_planning/extenders/MotionPrimitivesExtender.h"
#include "rrt_planning/map/ROSMap.h"
#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/utils/RandomGenerator.h"


using namespace Eigen;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::NHPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_planning
{

NHPlanner::NHPlanner()
{
    deltaX = 0;
    deltaTheta = 0;

    rosmap = nullptr;
    map = nullptr;
    l2dis = nullptr;
    thetadis = nullptr;
    thetadis = nullptr;

}

NHPlanner::NHPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void NHPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("deltaX", deltaX, 0.5);
    private_nh.param("deltaTheta", deltaTheta, 0.5);
    private_nh.param("k", k, 3);

    rosmap = new ROSMap(costmap_ros);
    map = new SGMap(*rosmap);
    l2dis = new L2Distance();
    thetadis = new ThetaDistance();
    l2thetadis = new L2ThetaDistance();

    map->initialize(private_nh);
    extenderFactory.initialize(private_nh, *rosmap, *l2thetadis);
    visualizer.initialize(private_nh);
    angleFactory.initialize(private_nh);
    positionFactory.initialize(private_nh);
}

bool NHPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
                                   const geometry_msgs::PoseStamped& goal_pose,
                                   std::vector<geometry_msgs::PoseStamped>& plan)
{
    visualizer.clean();
    count = 0;
    Distance& l2thetadis = *this->l2thetadis;
    Distance& l2dis = *this->l2dis;
    Distance& thetadis = *this->thetadis;
    VectorXd&& x0 = convertPose(start_pose);
    VectorXd&& xGoal = convertPose(goal_pose);

    if(!rosmap->isFree(x0))
    {
      ROS_FATAL("Invalid starting position");
      return false;
    }

    if(!rosmap->isFree(xGoal))
    {
      ROS_FATAL("Invalid goal position");
      return false;
    }

    //Initialization
    Node* start_node = new Node(x0, nullptr, 0);
    start_node->setParent(start_node);

    target = Action(xGoal, true, true, false, nullptr);
    shared_ptr<Action> goal_action = make_shared<Action>(target);
    goal_action->setParent(goal_action);
    target = *goal_action;

    addOpen(start_node, target, l2dis);
    start_node->addSubgoal(xGoal);
    reached[x0] = start_node;

    CornerIndex index(l2dis);
    index.insert(xGoal);

    ROS_FATAL("Start Search: pick a god and pray");
    ros::Time start_time = ros::Time::now();

    //Start search
    while(!open.empty())
    {
        Key key = open.pop();
        Node* current = key.first;
        Action action = key.second;
        bool improve = true;


        //Check if the goal is reached
        if(l2dis(current->getState(), xGoal) < deltaX && thetadis(current->getState(), xGoal) < deltaTheta)
        {
            auto&& path = retrievePath(current);
            publishPlan(path, plan, start_pose.header.stamp);
            open.clear();
            reached.clear();
            global_closed.clear();
            corner_samples.clear();

            visualizer.displayPlan(plan);
            visualizer.flush();
            ros::Time end = ros::Time::now();
            ros::Duration ex_time = end - start_time;

            ROS_FATAL("Plan found: simple geometry");
            ROS_FATAL_STREAM("Action count: " << count);
            ROS_FATAL_STREAM("Time: " << ex_time);

            return true;
        }

        Node* new_node = nullptr;
        vector<VectorXd> samples;
        double theta = action.getState()(2);
        if(action.getState() == xGoal)
        {
            samples.push_back(xGoal);
        }
        else if(action.isCorner())
        {
            VectorXd xCurr = current->getState();
            VectorXd xCorner = action.getState();
            Vector2d corner_key(xCorner(0), xCorner(1));

            if(corner_samples.count(corner_key))
            {
                samples = corner_samples.at(corner_key);
            } else
            {
                samples.push_back(xCorner);
            }

            theta = atan2(xCorner(1) - xCurr(1), xCorner(0) - xCurr(0));
        }

        for(auto c : samples)
        {
            VectorXd sample = c;
            if(sample != xGoal)
            {
                sample(2) = sampleAngle(theta);
            }
            new_node = reach(current, sample);
            if(new_node)
            {
                //If I can reach it, see if I already passed it or if it's the Goal
                if(!reached.count(new_node->getState()))
                {
                    reached[new_node->getState()] = new_node;
                    addOpen(new_node, target, l2dis);
                    new_node->addSubgoal(xGoal);
                }
                  else
                {
                    new_node = reached.at(new_node->getState());
                }

                visualizer.addSegment(current->getState(), new_node->getState());
                Action p = *action.getParent();
                if(!new_node->contains(p.getState()))
                {
                    Action parent(p.getState(), p.isClockwise(), true,
                                        p.isCorner(), p.getParent());
                    addOpen(new_node, parent, l2dis);
                    new_node->addSubgoal(parent.getState());
                }
                addGlobal(current->getState(), action.getState(), p.getState());
                current->addSubgoal(action.getState());
                improve = false;
            }

        }

        //Couldn't reach the corner or it is not valid, improve it
        if(improve)
        {
            vector<Triangle> triangles;
            vector<Action> new_actions = findAction(current, action, l2dis, triangles);
            for(auto a : new_actions)
            {

                if(fabs(current->getState()(2)) < 2*M_PI && a.getState() != action.getState())
                {
                    count++;
                    addOpen(current, a, l2dis);

                    if(a.isCorner())
                    {
                        Action copy = a;
                        VectorXd curr = a.getState();
                        VectorXd nearest = index.getNearestNeighbour(curr);
                        if(l2dis(nearest, curr) < deltaX)
                        {
                            copy.setState(nearest);
                        }
                        else
                        {
                            index.insert(curr);
                            VectorXd n = current->getState();
                            double theta = atan2(curr(1)- n(1), curr(0) - n(0));
                            curr(2) = theta;
                            sampleCorner(curr, a.isClockwise());
                        }

                        addSubgoal(current, copy, l2dis);
                        visualizer.addCorner(copy.getState());
                    }
                    else
                        visualizer.addPoint(a.getState());

                }
            }

            for(auto t : triangles)
            {
                current->addTriangle(t);
            }
        }
    }

    visualizer.flush();

    ROS_FATAL("Failed to find plan: omae wa mou shindeiru");

    open.clear();
    reached.clear();
    global_closed.clear();
    corner_samples.clear();
    //FIXME i did not find plan
    return true;

}

Node* NHPlanner::reach(Node* current, const VectorXd& xCorner)
{
    VectorXd xCurr = current->getState();
    VectorXd xNew = xCurr;
    vector<VectorXd> parents;
    bool is_valid = false;
    double cost = current->getCost();

    is_valid = extenderFactory.getExtender().steer(xCurr, xCorner, xNew, parents, cost);

    Node* new_node = nullptr;
    if(is_valid && map->isTrueCornerWOW(xNew))
    {
        parents.pop_back();
        new_node = new Node(xNew, current, cost, parents);
    }
    return new_node;
}

VectorXd NHPlanner::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& q_ros = msg.pose.orientation;
    auto& t_ros = msg.pose.position;

    Quaterniond q(q_ros.w, q_ros.x, q_ros.y, q_ros.z);

    Vector3d theta = q.matrix().eulerAngles(0, 1, 2);

    VectorXd x(3);
    x << t_ros.x, t_ros.y, theta(2);

    return x;
}

void NHPlanner::publishPlan(std::vector<VectorXd>& path,
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

void NHPlanner::addOpen(Node* node, const Action& action, Distance& distance)
{

    if(insideGlobal(action.getState(), action.isSubgoal()))
    {
    	return;
    }

   if(node->insideArea(action.getState()))
    {
    	return;
    }

    Key key(node, action);
    double h = distance(node->getState(), action.getState()) + distance(action.getState(), target.getState());
    open.insert(key, h + node->getCost());

}

void NHPlanner::addSubgoal(Node* node, const Action& action, Distance& distance)
{

    Action subgoal(action.getState(), action.isClockwise(), true, action.isCorner(), action.getParent());
    Node* parent = node->getParent();
    if(insideGlobal(action.getState(), action.isSubgoal()))
    {
        return;
    }

    while(!parent->contains(subgoal.getState()))
    {
        if(!parent->insideArea(subgoal.getState()))
        {
            Key key(parent, subgoal);
            double h = distance(parent->getState(), subgoal.getState()) + distance(subgoal.getState(), target.getState());
            open.insert(key, h + parent->getCost());
        }
        parent->addSubgoal(subgoal.getState());
        parent = parent->getParent();
    }

}

vector<Action> NHPlanner::findAction(Node* node, const Action& action, Distance& distance, vector<Triangle>& triangles)
{
    vector<Action> actions;
    VectorXd n = node->getState();
    VectorXd a = action.getState();
    vector<VectorXd> collision, vertices;
    //FIXME use parameters
    double step = 0.3;
    bool follow = false;

    triangles.clear();
    bool is_los = map->collisionPoints(a, n, collision);
    if(is_los)
    {
        VectorXd p = action.getParent()->getState();
        bool is_los = map->collisionPoints(n, p, collision);
        if(collision.size() == 2)
        {
            p = map->computeMiddle(collision[0], collision[1]);
        }
        Triangle t(a, n, p);
        triangles.push_back(t);
        collision.clear();
    }

    if(is_los)
    {
        is_los = map->followObstacle(n, a, collision);
        if(is_los){
            actions.push_back(Action(collision[0], action.isClockwise(), false, true, action.getParent()));
            return actions;
        }
        follow = true;
    }

    if(is_los || collision.size() < 2) {return actions;}

    double c1 = distance(collision[0], a);
    double c2 = distance(collision[1], a);
    bool sample = action.isSubgoal();
    if(!follow && c1 > step && c2 > step) {sample = true;}
    if(!is_los) {swap(collision[0], collision[1]);}

    VectorXd middle = map->computeMiddle(collision[0], collision[1]);

    if(action.isClockwise() || sample)
    {
        VectorXd new_state = map->exitPoint(n, middle, true);
        vertices.push_back(new_state);
        if(rosmap->insideBound(new_state))
        {
            shared_ptr<Action> p = action.getParent();
            bool corner = map->isCorner(new_state);
            if(sample)
            {
                p = make_shared<Action>(action);
            }

            actions.push_back(Action(new_state, true, false, corner, p));
        }
    }

    if(!action.isClockwise() || sample)
    {
        VectorXd new_state = map->exitPoint(n, middle, false);
        vertices.push_back(new_state);
        if(rosmap->insideBound(new_state))
        {
            shared_ptr<Action> p = action.getParent();
            bool corner = map->isCorner(new_state);
            if(sample)
            {
                p = make_shared<Action>(action);
            }
            actions.push_back(Action(new_state, false, false, corner, p));
        }
    }


    if(sample && vertices.size() == 2 && vertices[0] != vertices[1])
    {
        Triangle t(a, vertices[0], vertices[1]);
        triangles.push_back(t);
    }

    return actions;
  }

void NHPlanner::addGlobal(const VectorXd& node, const VectorXd& action, const VectorXd& parent)
{
    Distance& distance= *this->l2dis;
    vector<VectorXd> collision;

    //check los(node, action)
    bool is_los = map->collisionPoints(node, action, collision);
    if(!is_los && distance(collision[1], action) > deltaX)
    {
        return;
    }

    //check los(node, parent)
    is_los = map->collisionPoints(node, parent, collision);
    VectorXd p = parent;
    if(collision.size() == 2)
    {
        p = map->computeMiddle(collision[0], collision[1]);
    }

    //TODO check los(action, parent)

    Triangle t(node, action, p);
    global_closed.push_back(t);
}

bool NHPlanner::insideGlobal(const Eigen::VectorXd& p, bool subgoal)
{
    for(auto t: global_closed)
    {
        if(t.contains(p))
            return true;
  }

  return false;
}

void NHPlanner::sampleCorner(const VectorXd& corner, bool cw)
{
    vector<VectorXd> samples;
    while(samples.size() < k)
    {
        VectorXd sample = positionFactory.getSampling().sample(corner, cw);
        if(rosmap->isFree(sample))
        {
            samples.push_back(sample);
            visualizer.addCorner(sample);
        }
    }

    corner_samples[Vector2d(corner(0), corner(1))] = samples;

    return;
}

double NHPlanner::sampleAngle(double theta)
{
    return (angleFactory.getSampling().sample() + theta);
}


vector<VectorXd> NHPlanner::retrievePath(Node* node)
{
    std::vector<Eigen::VectorXd> path, mp;
    Node* current = node;

    while(current->getCost() != 0)
    {
        visualizer.addPathPoint(current->getState());
        path.push_back(current->getState());
        mp = current->getMotionPrimitives();
        std::reverse(mp.begin(), mp.end());
        for(auto m : mp)
        {
            path.push_back(m);
        }
        current = current->getParent();
    }

    path.push_back(current->getState());
    std::reverse(path.begin(), path.end());

    return path;
}

NHPlanner::~NHPlanner()
{
    if(l2dis)
        delete l2dis;

    if(thetadis)
        delete thetadis;

    if(l2thetadis)
        delete l2thetadis;

    if(map)
        delete map;

    if(rosmap)
        delete rosmap;
}


};
