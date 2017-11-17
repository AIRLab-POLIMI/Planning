#ifndef NODE_H
#define NODE_H

#include <cassert>
#include <set>
#include <Eigen/Dense>
#include "rrt_planning/nh/Action.h"



namespace rrt_planning
{

typedef std::pair<const Eigen::VectorXd, const Eigen::VectorXd> CoorPair;

struct CoorCmp
{
    bool operator()(const rrt_planning::CoorPair& a, const rrt_planning::CoorPair& b) const
    {
        return (eigenOrdering(a.first, b.first) || a.first == b.first && eigenOrdering(a.second, b.second));
    }

    bool eigenOrdering(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const
    {
      return (a(0) < b(0) || a(0) == b(0) && a(1) < b(1));
    }

};

class Node
{
public:
    inline Node();
    inline Node(const Eigen::VectorXd& state, Node* parent, double cost):
                state(state), parent(parent), cost(cost) {}

    void addSubgoal(const Eigen::VectorXd& subgoal)
    {
        rrt_planning::CoorPair pair(subgoal, subgoal);
        closed.insert(pair);
    }

    void addClosed(const Action& a)
    {
      rrt_planning::CoorPair pair(a.getState(), a.getSubgoal());
      closed.insert(pair);
    }

    bool contains(const Action& action)
    {
        rrt_planning::CoorPair pair(action.getState(), action.getSubgoal());
        return (closed.count(pair) == 1);
    }

    Node* setParent(Node* p){parent = p;}

    Eigen::VectorXd getState() const {return state;}
    Node* getParent() {return parent;}
    double getCost() const {return cost;}

private:
    Eigen::VectorXd state;
    Node* parent;
    double cost;
    std::set<rrt_planning::CoorPair, rrt_planning::CoorCmp> closed;
};

}
#endif // NODE_H
