#ifndef NODE_H
#define NODE_H

#include <map>
#include <Eigen/Dense>
#include "rrt_planning/nh/Action.h"

namespace rrt_planning
{

typedef std::pair<Cell, Cell> CellPair;

class Node
{
public:
    inline Node();
    inline Node(const Cell& cell, const Eigen::VectorXd state, Node* parent, double cost):
        cell(cell), state(state), parent(parent), cost(cost) {}

    void addSubgoal(const Cell& subgoal)
    {
        CellPair pair(subgoal, subgoal);
        closed.insert(pair);
    }

    void addClosed(const Action& action)
    {
        CellPair pair(action.getCell(), action.getSubgoal());
        closed.insert(pair);
    }

    bool contains(const Action& action)
    {
        CellPair pair(action.getCell(), action.getSubgoal());
        return closed.count(pair);
    }

    Node* setParent(Node* p){parent = p;}

    Cell getCell() const {return cell;}
    Eigen::VectorXd getState() const {return state;}
    Node* getParent() {return parent;}
    double getCost() const {return cost;}

private:
    Cell cell;
    Eigen::VectorXd state;
    Node* parent;
    double cost;
    std::set<CellPair> closed;
};

}
#endif // NODE_H
