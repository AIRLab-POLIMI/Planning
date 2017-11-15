#ifndef NODE_H
#define NODE_H

#include <map>
#include <Eigen/Dense>
#include "rrt_planning/nh/Action.h"

namespace rrt_planning
{

typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> CoorPair;

class Node
{
public:
    inline Node();
    inline Node(const Eigen::VectorXd& state, Node* parent, double cost):
                state(state), parent(parent), cost(cost) {}

    void addSubgoal(const Eigen::VectorXd& subgoal)
    {
        CoorPair pair(subgoal, subgoal);
        closed.insert(pair);
    }


    bool contains(const Action& action)
    {
        CellPair pair(action.getState(), action.getSubgoal());
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
    std::set<CellPair> closed;
};

}
#endif // NODE_H
