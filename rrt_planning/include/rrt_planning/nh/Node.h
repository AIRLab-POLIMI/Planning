#ifndef INCLUDE_RRT_PLANNING_NH_NODE_H
#define INCLUDE_RRT_PLANNING_NH_NODE_H

#include <cassert>
#include <set>
#include <Eigen/Dense>
#include "rrt_planning/nh/Action.h"
#include "rrt_planning/nh/Triangle.h"


namespace rrt_planning
{

typedef std::pair<double, double> Sub;

class Node
{
public:
    Node();
    Node(const Eigen::VectorXd& state, Node* parent, double cost);
    Node(const Eigen::VectorXd& state, Node* parent, double cost, std::vector<Eigen::VectorXd> mp);

    void addSubgoal(const Eigen::VectorXd& subgoal);
    void addTriangle(const Triangle& t);
    bool insideArea(const Eigen::VectorXd& p);
    bool contains(const Eigen::VectorXd& subgoal);

    Node* setParent(Node* p);

    Node* getParent();
    double getCost();
    Eigen::VectorXd getState();
    std::vector<Eigen::VectorXd> getMotionPrimitives();


private:
    Eigen::VectorXd state;
    Node* parent;
    double cost;
    std::vector<Eigen::VectorXd> mp; //motion primitives to reach parent
    std::set<Sub> subgoals;
    std::vector<Triangle> closed_area;
};

}
#endif // INCLUDE_RRT_PLANNING_NH_NODE_H
