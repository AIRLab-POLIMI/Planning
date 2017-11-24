#ifndef NODE_H
#define NODE_H

#include <cassert>
#include <set>
#include <Eigen/Dense>
#include "rrt_planning/nh/Action.h"
#include "rrt_planning/nh/Triangle.h"



namespace rrt_planning
{

typedef std::pair<const Eigen::VectorXd, const Eigen::VectorXd> CoorPair;
typedef std::pair<double, double> Sub;

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
    inline Node(const Eigen::VectorXd& state, Node* parent, double cost, std::vector<Eigen::VectorXd> mp):
                state(state), parent(parent), cost(cost), mp(mp) {}

    void addSubgoal(const Eigen::VectorXd& subgoal)
    {
        rrt_planning::Sub pair(subgoal(0), subgoal(1));
        subgoals.insert(pair);
    }

    void addClosed(const Action& a)
    {
      rrt_planning::CoorPair pair(a.getState(), a.getSubgoal());
      closed.insert(pair);
    }

    void addTriangle(const Triangle& t)
    {
      closed_area.push_back(t);
    }

    bool insideArea(const Eigen::VectorXd& p)
    {
      for(auto t : closed_area)
      {
        if(t.contains(p))
          return true;
      }

      return false;      
    }

    bool contains(const Action& action)
    {
        rrt_planning::Sub pair(action.getState()(0), action.getState()(1));
        return (subgoals.count(pair) == 1);
    }

    int getSize()
    {
      return closed.size();
    }

    std::set<rrt_planning::CoorPair, rrt_planning::CoorCmp> getClosed()
    {
      return closed;
    }

    Node* setParent(Node* p){parent = p;}

    Eigen::VectorXd getState() const {return state;}
    Node* getParent() {return parent;}
    std::vector<Eigen::VectorXd> getMotionPrimitives() {return mp;}
    double getCost() const {return cost;}

private:
    Eigen::VectorXd state;
    Node* parent;
    double cost;
    std::vector<Eigen::VectorXd> mp; //motion primitives to reach parent
    std::set<rrt_planning::CoorPair, rrt_planning::CoorCmp> closed;
    std::set<Sub> subgoals;
    std::vector<Triangle> closed_area;
};

}
#endif // NODE_H
