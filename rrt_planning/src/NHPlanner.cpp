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

    map = nullptr;
    gridmap = nullptr;
    distance = nullptr;
}

NHPlanner::NHPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void NHPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    double discretization;

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("deltaX", deltaX, 0.5);
    private_nh.param("discretization", discretization, 0.2);

    map = new ROSMap(costmap_ros);
    gridmap = new Gridmap(*map, discretization);
    distance = new L2Distance();

    extenderFactory.initialize(private_nh, *map, *distance);
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

    //Initialization
    Cell start = gridmap->convertPose(start_pose);
    Cell goal = gridmap->convertPose(goal_pose);
    Node* start_node = new Node(start, x0, nullptr, 0);
    start_node->setParent(start_node);
    reached[x0] = start_node;

    target = Action(goal, xGoal, goal, true, true, false, nullptr);
    shared_ptr<Action> goal_action = make_shared<Action>(target);
    goal_action->setParent(goal_action);
    target = *goal_action;

    addOpen(start_node, target, distance);
    //reached[x0] = start_node;

    ROS_INFO("Pick a god and pray");

    //Start search
    while(!open.empty())
    {
        Key key = open.pop();
        Node* current = key.first;
        Action action = key.second;
        bool improve = true;

        //Check if the goal is reached
        if(distance(current->getState(), xGoal) < deltaX)
        {
            ROS_INFO("Retrieving plan");
            auto&& path = retrievePath(current);
            publishPlan(path, plan, start_pose.header.stamp);
            open.clear();
            reached.clear();

            visualizer.displayPlan(plan);
            visualizer.flush();

            ROS_INFO("Plan found");

            return true;
        }

        //Add new subgoal reached to the list
        if(action.getCell() == goal || action.isCorner())
        {
            //Motion primitives
            VectorXd xCurr = current->getState();
            VectorXd xNew;
            int loop = 0;
            bool is_valid = true;
            vector<VectorXd> parents;

            ROS_INFO("trying to reach");
            do{
              is_valid = newState(xCurr, action.getState(), xNew);
              xCurr = xNew;
              parents.push_back(xCurr);
              loop++;
            } while(is_valid && !(distance(xCurr, action.getState()) < deltaX) && loop < 20);

            if(loop >= 20) is_valid = false;


            if(is_valid)
            {
                Node* new_node;
                if(!reached.count(xCurr))
                {
                    parents.pop_back();
                    Node* curr = current;
                    for(auto p : parents)
                    {
                      double c = curr->getCost() + distance(curr->getState(), p);
                      Cell cell = gridmap->convertPose(p);
                      Node* ancestor = new Node(cell, p, curr, c);
                      curr = ancestor;
                    }
                    Cell cell_node = gridmap->convertPose(xCurr);
                    double cost = curr->getCost() + distance(curr->getState(), xCurr);
                    new_node = new Node(cell_node, xCurr, curr, cost);
                    reached[xCurr] = new_node;
                    addOpen(new_node, target, distance);
                }
                  else
                {
                    new_node = reached.at(xCurr);
                }

                visualizer.addSegment(current->getState(), new_node->getState());
                Action parent(action.getParent()->getCell(), action.getParent()->getState(), action.getParent()->getCell(),
                              action.getParent()->isClockwise(), true, action.getParent()->isCorner(), action.getParent()->getParent());
                addOpen(new_node, parent, distance);
                improve = false;

                if(distance(new_node->getState(), xGoal) < deltaX)
                {
                    ROS_INFO("Retrieving plan");
                    auto&& path = retrievePath(new_node);
                    publishPlan(path, plan, start_pose.header.stamp);
                    open.clear();
                    reached.clear();

                    visualizer.displayPlan(plan);
                    visualizer.flush();

                    ROS_INFO("Plan found");

                    return true;
                }
            }

        }
        if(improve)
        {
            vector<Action> new_actions = findAction(current, action);
            for(auto a : new_actions)
            {
                visualizer.addPoint(a.getState());
                ROS_WARN_STREAM("added point: " << a.getCell().first << ", " << a.getCell().second );
                if(!current->contains(a))
                {
                    addOpen(current, a, distance);
                    if(a.isCorner() && a.getState() != xGoal)
                      addSubgoal(current, a, distance);
                }
            }
        }
    }

    visualizer.flush();

    ROS_WARN_STREAM("Picked the wrong God. Try Dio Brando next time.");
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
    if(!node->contains(action))
    {
        Key key(node, action);
        double h = distance(node->getState(), action.getState()) + distance(action.getState(), target.getState());
        open.insert(key, h + node->getCost());
        node->addClosed(action);
    }
}

void NHPlanner::addSubgoal(Node* node, const Action& action, Distance& distance)
{
    Action subgoal(action.getCell(), action.getState(), action.getCell(), action.isClockwise(), true, action.isCorner(), action.getParent());
    Node* parent = node->getParent();

    while(!parent->contains(subgoal))
    {
        if(reached.count(parent->getState()))
        {
          addOpen(parent, subgoal, distance);
          parent->addSubgoal(subgoal.getCell());
        }
        parent = parent->getParent();
    }

}

vector<Action> NHPlanner::findAction(const Node* node, const Action& action)
{
    vector<Action> actions;
    Cell n = node->getCell();
    Cell a = action.getCell();
    vector<Cell> collision;
    double theta;

    bool is_los;

    ROS_INFO("Starting los");
    is_los = gridmap->los(a, n, collision, collision_points);
    ROS_INFO("los ended");

    if(is_los)
    {
        actions = followObstacle(n, action);
    }
    if(is_los || collision.size() < 2) {return actions;}

    bool sample = action.isSubgoal();
    if(a != collision[1] && a != collision[0]) {sample = true;}
    if(!is_los) {swap(collision[0], collision[1]);}

    //Start the search for new actions (at last)
    Cell middle = gridmap->computeMiddle(collision[0], collision[1], a);
    vector<Cell> subgoals = gridmap->findSubgoals(collision[0], collision[1]);
    Cell ccw_point = subgoals[0];
    Cell cw_point = subgoals[1];

    if(action.isClockwise() || sample)
    {
        is_los = gridmap->los(middle, cw_point, collision, exit_point);
        if(is_los)
        {
            shared_ptr<Action> p = action.getParent();
            Cell sub = action.getSubgoal();
            theta = atan2(collision[0].second - n.second, collision[0].first - n.first);
            bool corner = gridmap->isCorner(collision[0]);
            //bool corner = false;
            VectorXd state = gridmap->toMapPose(collision[0].first, collision[0].second, theta);
            if(sample)
            {
                p = make_shared<Action>(action);
                if(action.getSubgoal() == target.getCell() && action.getCell() != target.getCell()){
                    sub = action.getCell();
                }
            }
            actions.push_back(Action(collision[0], state, sub, true, false, corner, p));
            if(corner)
              sampleCorner(n, actions.back(), actions);
         }
    }

    if(!action.isClockwise() || sample)
    {
        is_los = gridmap->los(middle, ccw_point, collision, exit_point);
        if(is_los)
        {
            shared_ptr<Action> p = action.getParent();
            Cell sub = action.getSubgoal();
            theta = atan2(collision[0].second - n.second, collision[0].first - n.first);
            bool corner = gridmap->isCorner(collision[0]);
            //bool corner = false;
            VectorXd state = gridmap->toMapPose(collision[0].first, collision[0].second, theta);
            if(sample)
            {
                p = make_shared<Action>(action);
                if(action.getSubgoal() == target.getCell() && action.getCell() != target.getCell()){
                    sub = action.getCell();
                }
            }
            actions.push_back(Action(collision[0], state, sub, false, false, corner, p));
            if(corner)
              sampleCorner(n, actions.back(), actions);
         }
    }

    return actions;
}

void NHPlanner::sampleCorner(const Cell& current, const Action& action, vector<Action> actions)
{
    double lambda = 2.0;
    int samples = 10;
    Cell a = action.getCell();
    VectorXd state = action.getState();

    double theta = atan2(a.second - current.second, a.first - current.first);
    double theta_new = action.isClockwise() ? theta - M_PI/2 : theta + M_PI/2;


    for(int i = 0; i < samples; i++)
    {
      double sample = RandomGenerator::sampleExponential(lambda);
      VectorXd new_state = Vector3d(state(0) + sample*cos(theta_new), state(1) + sample*sin(theta_new), state(2));
      Cell new_corner = gridmap->convertPose(new_state);
      actions.push_back(Action(new_corner, new_state, action.getSubgoal(), false, action.isClockwise(), true, action.getParent()));
    }

}


vector<Action> NHPlanner::followObstacle(const Cell& node, const Action& action)
{
    Cell a = action.getCell();
    Cell corner;
    int dx = a.first - node.first;
    int dy = a.second - node.second;
    std::vector<Action> actions;

    bool no_action = false;
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
      sampleCorner(node, actions.back(), actions);
    return actions;
}

vector<VectorXd> NHPlanner::retrievePath(Node* node)
{
    std::vector<Eigen::VectorXd> path;
    Node* current = node;

    while(current->getCost() != 0)
    {
        path.push_back(current->getState());
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
}


};
