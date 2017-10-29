#ifndef NODE_H
#define NODE_H

#include <map>
#include "rrt_planning/nh/action.h"

typedef std::pair<Cell, Cell> CellPair;

class Node
{
public:
    inline Node();
    inline Node(const Cell& cell, Node* parent, double cost):
        cell(cell), parent(parent), cost(cost) {}

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
    Node* getParent() {return parent;}
    double getCost() const {return cost;}

private:
    Cell cell;
    Node* parent;
    double cost;
    std::set<CellPair> closed;
};

#endif // NODE_H
