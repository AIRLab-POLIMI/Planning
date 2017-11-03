#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

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
    distance = new L2ThetaDistance();

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
    Cell start(x0(0), x0(1));
    Node* start_node = new Node(start, x0, nullptr, 0);
    start_node->setParent(start_node);

    Cell goal(xGoal(0), xGoal(1));
    Action target(goal, xGoal, xGoal, true, true, nullptr);
    shared_ptr<Action> goal_action = make_shared<Action>(target);
    goal_action->setParent(goal_action);
    target = *goal_action;

    addOpen(start_node, target, distance, xGoal);
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

            visualizer.displayPlan(plan);
            visualizer.flush();

            ROS_INFO("Plan found");

            return true;
        }

        //Add new subgoal reached to the list
        if(action.getCell() == goal || gridmap->isCorner(action.getCell()))
        {
            //Motion primitives
            VectorXd xCurr = current->getState();
            VectorXd xNew;
            bool is_valid = true;
            ROS_INFO("Trying to reach action ");
            while(is_valid && !(distance(xCurr, action.getState()) < deltaX))
            {
                double x = xCurr(0);
                double y = xCurr(1);

                /*ROS_WARN_STREAM("Trying to reach action from (" << x << ", " << y << ", " << z << ") ");*/
                is_valid = newState(xCurr, action.getState(), xNew);
                double z = xNew(0);
                double w = xNew(1);
                if(x == z && y == w)
                    is_valid = false;
                xCurr = xNew;
            }
            ROS_INFO("Checking if action was reached");
            if(is_valid)
            {
                double cost = current->getCost() + distance(current->getState(), xCurr);
                Node* new_node = new Node(Cell(xCurr(0),xCurr(1)), xCurr, current, cost);
                addOpen(new_node, target, distance, xGoal);
                Action parent(action.getParent()->getCell(), action.getParent()->getState(), action.getParent()->getState(), action.getParent()->isClockwise(), true, action.getParent()->getParent());
                addOpen(new_node, parent, distance, xGoal);
                improve = false;
            }

        }
        if(improve)
        {
            ROS_INFO("Improving the action");
            vector<Action> new_actions = findAction(current, action, xGoal);
            for(auto a : new_actions)
            {
                visualizer.addPoint(a.getState());
                addOpen(current, a, distance, xGoal);
                if(gridmap->isCorner(a.getCell()))
                    addSubgoal(current, a, distance, xGoal);
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

void NHPlanner::addOpen(Node* node, const Action& action, Distance& distance, VectorXd& xGoal)
{
    //if(!node->contains(action))
    //{
        Key key(node, action);
        double h = distance(node->getState(), action.getState()) + distance(action.getState(), xGoal);
        open.insert(key, h + node->getCost());
        //node->addClosed(action);
    //}
}

void NHPlanner::addSubgoal(Node* node, const Action& action, Distance& distance, VectorXd& xGoal)
{
    Action subgoal(action.getCell(), action.getState(), action.getState(), action.isClockwise(), true, action.getParent());
    Node* parent = node->getParent();

    while(parent->getCost() != 0)
    {
        addOpen(parent, subgoal, distance, xGoal);
        parent = parent->getParent();
    }
    addOpen(parent, subgoal, distance, xGoal);
}

vector<Action> NHPlanner::findAction(const Node* node, const Action& action, VectorXd& xGoal)
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
            VectorXd sub = action.getSubgoal();
            theta = atan2(collision[0].first - n.first, collision[0].second - n.second);
            VectorXd state = Vector3d(collision[0].first, collision[0].second, theta);
            if(sample)
            {
                p = make_shared<Action>(action);
                if(action.getSubgoal() == xGoal && action.getSubgoal() != xGoal){
                    sub = action.getState();
                }
            }
            actions.push_back(Action(collision[0], state, sub, true, false, p));
         }
    }

    if(!action.isClockwise() || sample)
    {
        is_los = gridmap->los(middle, ccw_point, collision, exit_point);
        if(is_los)
        {
            shared_ptr<Action> p = action.getParent();
            VectorXd sub = action.getSubgoal();
            theta = atan2(collision[0].first - n.first, collision[0].second - n.second);
            VectorXd state = Vector3d(collision[0].first, collision[0].second, theta);
            if(sample)
            {
                p = make_shared<Action>(action);
                if(action.getSubgoal() == xGoal && action.getSubgoal() != xGoal){
                    sub = action.getState();
                }
            }
            actions.push_back(Action(collision[0], state, sub, false, false, p));
         }
    }

    return actions;
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
    double theta = atan2(corner.first - node.first, corner.second - node.second);
    VectorXd state = Vector3d(corner.first, corner.second, theta);

    actions.push_back(Action(corner, state, action.getSubgoal(), action.isClockwise(), false, action.getParent()));

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
