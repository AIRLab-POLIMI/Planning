#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <random>

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

    rosmap = nullptr;
    map = nullptr;
    distance = nullptr;

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
    private_nh.param("ray", ray, 0.5);
    private_nh.param("threshold", threshold, 0.35);
    private_nh.param("discretization", discretization, 5);

    rosmap = new ROSMap(costmap_ros);
    map = new SGMap(*rosmap, discretization, ray, threshold);
    distance = new L2Distance();

    extenderFactory.initialize(private_nh, *rosmap, *distance);
    visualizer.initialize(private_nh);
}

bool NHPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
                                   const geometry_msgs::PoseStamped& goal_pose,
                                   std::vector<geometry_msgs::PoseStamped>& plan)
{
    visualizer.clean();

    Distance& distance = *this->distance;
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
    vector<Eigen::VectorXd> dummy;
    Node* start_node = new Node(x0, nullptr, 0, dummy);
    start_node->setParent(start_node);

    target = Action(xGoal, xGoal, xGoal, true, true, false, nullptr);
    shared_ptr<Action> goal_action = make_shared<Action>(target);
    goal_action->setParent(goal_action);
    target = *goal_action;

    addOpen(start_node, target, distance);
    start_node->addSubgoal(xGoal);
    reached[x0] = start_node;

    CornerIndex index(distance);
    index.insert(xGoal);

    ROS_FATAL("Start Search: pick a god and pray");

    //Start search
    while(!open.empty())
    {

        Key key = open.pop();
        Node* current = key.first;
        Action action = key.second;
        bool improve = true;
        //ROS_FATAL_STREAM("Closed size " << current->getSize());

        //Check if the goal is reached
        if(distance(current->getState(), xGoal) < deltaX)
        {
            auto&& path = retrievePath(current);
            publishPlan(path, plan, start_pose.header.stamp);
            open.clear();
            reached.clear();

            visualizer.displayPlan(plan);
            visualizer.flush();

            ROS_FATAL("Plan found: simple geometry");

            return true;
        }

        if(action.isCorner() || action.getState() == xGoal)
        {
            //Motion primitives
            VectorXd xCurr = current->getState();
            VectorXd xNew, xCorner;
            bool is_valid = true;
            vector<VectorXd> parents;
            double cost = current->getCost();


            //ROS_FATAL("trying to reach");
            do{
              is_valid = newState(xCurr, action.getState(), xNew);
              cost += distance(xCurr, xNew);
              xCurr = xNew;
              parents.push_back(xCurr);
            } while(is_valid && !(distance(xCurr, action.getState()) < deltaX));

            if(action.getState()!= xGoal && (!is_valid || !map->isTrueCornerWOW(xCurr))){
              //ROS_FATAL("trying to reach sample");
              vector<Action> actions;
              xCurr = current->getState();
              sampleCorner(xCurr, action, actions);
              xCorner = actions[0].getState();
              if(!rosmap->isFree(xCorner))
                continue;
              visualizer.addCorner(xCorner);
              cost = current->getCost();
              parents.clear();
              do {
                is_valid = newState(xCurr, xCorner, xNew);
                cost += distance(xCurr, xNew);
                xCurr = xNew;
                parents.push_back(xCurr);
              } while(is_valid && !(distance(xCurr, xCorner) < deltaX));
            }

            if(is_valid)
            {
                //If I can reach it, see if I already passed it or if it's the Goal
                Node* new_node;
                if(!reached.count(xCurr))
                {
                    parents.pop_back();
                    new_node = new Node(xCurr, current, cost, parents);
                    reached[xCurr] = new_node;
                    addOpen(new_node, target, distance);
                    new_node->addSubgoal(xGoal);
                }
                  else
                {
                    new_node = reached.at(xCurr);
                }

                visualizer.addSegment(current->getState(), new_node->getState());
                Action parent(action.getParent()->getState(), action.getParent()->getState(), action.getParent()->getState(),
                              action.getParent()->isClockwise(), true, action.getParent()->isCorner(), action.getParent()->getParent());
                addOpen(new_node, parent, distance);
                improve = false;

                //Check if the goal is reached
                if(distance(new_node->getState(), xGoal) < deltaX)
                {
                    auto&& path = retrievePath(new_node);
                    publishPlan(path, plan, start_pose.header.stamp);
                    open.clear();
                    reached.clear();

                    visualizer.displayPlan(plan);
                    visualizer.flush();

                    ROS_FATAL("Plan found: simple geometry");

                    return true;
                }
            }
        }

        //Couldn't reach the action and it is not valid, improve it
        if(improve)
        {
            vector<Action> new_actions = findAction(current, action, distance);
            for(auto a : new_actions)
            {
                if(fabs(current->getState()(2)) < 2*M_PI)
                {
                    addOpen(current, a, distance);

                    if(a.isCorner())
                    {
                        Action copy = a;
                        VectorXd curr = a.getState();
                        VectorXd nearest = index.getNearestNeighbour(curr);
                        if(distance(nearest, curr) < deltaX)
                            copy.setState(nearest);
                        else
                            index.insert(curr);

                        addSubgoal(current, copy, distance);
                        visualizer.addCorner(copy.getState());
                    }
                    else
                        visualizer.addPoint(a.getState());


                    ROS_WARN_STREAM( "node " << current->getState()(0) << ", " << current->getState()(1) << ", " << current->getState()(2)
                                    << " added point: " << a.getState()(0) << ", " << a.getState()(1)
                                    << " sub: " << a.getSubgoal()(0) << ", " << a.getSubgoal()(1));
                }
            }
        }
    }

    visualizer.flush();

    ROS_FATAL("Failed to find plan: omae wa mou shindeiru");
    exit(0);
    return false;

}


bool NHPlanner::newState(const VectorXd& xNear, const VectorXd& xSample, VectorXd& xNew)
{
    return extenderFactory.getExtender().los(xNear, xSample, xNew);
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
    std::set<rrt_planning::CoorPair, rrt_planning::CoorCmp> closed = node->getClosed();
    VectorXd a = action.getState();
    VectorXd s = action.getSubgoal();
    for(auto c : closed)
    {
        double bucket = 0.17;
        double dist_a = distance(c.first, a);
        double dist_s = distance(c.second, s);
        if(dist_a < bucket && dist_s < bucket)
        {
          ROS_FATAL("You can't sit with us!");
          return;
        }
    }

    Key key(node, action);
    double h = distance(node->getState(), action.getState()) + distance(action.getState(), target.getState());
    open.insert(key, h + node->getCost());
    node->addClosed(action);
}

void NHPlanner::addSubgoal(Node* node, const Action& action, Distance& distance)
{
    Action subgoal(action.getState(), action.getState(), action.getState(), action.isClockwise(), true, action.isCorner(), action.getParent());
    node->addSubgoal(subgoal.getState());
    Node* parent = node->getParent();

    while(!parent->contains(subgoal))
    {
        addOpen(parent, subgoal, distance);
        parent->addSubgoal(subgoal.getState());
        parent = parent->getParent();
    }

}

vector<Action> NHPlanner::findAction(const Node* node, const Action& action, Distance& distance)
{
    vector<Action> actions;
    VectorXd n = node->getState();
    VectorXd curr = n;
    VectorXd a = action.getState();
    vector<VectorXd> collision;
    VectorXd old = a;
    VectorXd new_state;
    Vector3d NULL_VEC(-1, -1, -1);
    double step = 0.3;
    std::vector<Eigen::VectorXd> points;

    bool is_los = map->collisionPoints(a, n, collision);

    if(is_los)
    {
        old = action.getOld();
        is_los = map->followObstacle(n, a, collision);
        if(is_los){
          actions.push_back(Action(collision[0], action.getSubgoal(), action.getOld(), action.isClockwise(), false, true, action.getParent()));
          ROS_FATAL("is true corner wow... ?");
          if(collision[0](0) == a(0) && collision[0](1) == a(1))
          {
            ROS_FATAL("you thought it was a corner, but it was me THE ACTION");
          }
          return actions;
        }
        //curr = a;
    }

    if(is_los || collision.size() < 2) {
      ROS_FATAL("LOL");
      return actions;
    }

    double c1 = distance(collision[0], a);
    double c2 = distance(collision[1], a);
    bool sample = action.isSubgoal();
    if( c1 > step && c2 > step) {sample = true;}
    if(!is_los) {swap(collision[0], collision[1]);}

    VectorXd middle = map->computeMiddle(collision[0], collision[1]);

    if(action.isClockwise() || sample)
    {
      new_state = map->exitPoint(curr, middle, true);
      if(new_state != NULL_VEC)
      {
          shared_ptr<Action> p = action.getParent();
          VectorXd sub = action.getSubgoal();
          bool corner = map->isCorner(new_state, points);
          if(sample)
          {
              p = make_shared<Action>(action);
              if(action.getSubgoal() == target.getState() && action.getState() != target.getState()){
                  sub = action.getState();
              }
          }
          actions.push_back(Action(new_state, sub, old, true, false, corner, p));
          /*if(corner)
          {
            for(auto p : points)
                visualizer.addPoint(p);
          }*/
       }
    }

    if(!action.isClockwise() || sample)
    {
      new_state = map->exitPoint(curr, middle, false);
      if(new_state != NULL_VEC)
      {
          shared_ptr<Action> p = action.getParent();
          VectorXd sub = action.getSubgoal();
          bool corner = map->isCorner(new_state, points);
          if(sample)
          {
              p = make_shared<Action>(action);
              if(action.getSubgoal() == target.getState() && action.getState() != target.getState()){
                  sub = action.getState();
              }
          }
          actions.push_back(Action(new_state, sub, old, false, false, corner, p));
          /*if(corner)
          {
            for(auto p : points)
                visualizer.addPoint(p);
          }*/
       }
    }

    if(actions.empty())
      ROS_FATAL("LOL2");

    ROS_FATAL("Finished update");
    return actions;
  }

void NHPlanner::sampleCorner(const VectorXd& current, const Action& action, vector<Action>& actions)
{
    double lambda = 1/0.25;
    int samples = 1;
    VectorXd a = action.getState();
    VectorXd vec = a - current;

    double theta = atan2(vec(1), vec(0));
    double theta_new = action.isClockwise() ? theta - M_PI/2 : theta + M_PI/2;


    for(int i = 0; i < samples; i++)
    {
      double sample = RandomGenerator::sampleExponential(lambda);
      VectorXd new_state = Vector3d(a(0) + sample*cos(theta_new), a(1) + sample*sin(theta_new), a(2));
      actions.push_back(Action(new_state, action.getSubgoal(), a, false, action.isClockwise(), true, action.getParent()));
    }
}


vector<Action> NHPlanner::followObstacle(const Cell& node, const Action& action)
{
    /*Cell a = action.getCell();
    Cell corner;
    int dx = a.first - node.first;
    int dy = a.second - node.second;*/
    std::vector<Action> actions;

    /*bool no_action = false;
    Cell prev = a;

    while(!no_action)
    {
        vector<Cell> subgoals = gridmap->getPoints(node, prev, action.isClockwise());

        corner = gridmap->followObstacle(prev, subgoals[0]);
        if(corner == prev)
        {
            corner = gridmap->followObstacle(prev, subgoals[1]);
            if(corner == prev)
                no_action = true;

            else
                prev = corner;
        }
        else
            prev = corner;

    }
    double theta = atan2(corner.second - node.second, corner.first - node.first);
    VectorXd state = gridmap->toMapPose(corner.first, corner.second, theta);
    bool is_corner = gridmap->isCorner(corner);

    actions.push_back(Action(corner, state, action.getSubgoal(), action.isClockwise(), false, is_corner, action.getParent()));
    if(is_corner)
      sampleCorner(node, actions.back(), actions);*/
    return actions;
}

vector<VectorXd> NHPlanner::retrievePath(Node* node)
{
    std::vector<Eigen::VectorXd> path, mp;
    Node* current = node;

    while(current->getCost() != 0)
    {
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
    if(distance)
        delete distance;

    if(map)
        delete map;

    if(rosmap)
        delete rosmap;
}


};
