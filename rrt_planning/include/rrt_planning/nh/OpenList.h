#ifndef OPENLIST_H
#define OPENLIST_H

#include "rrt_planning/nh/Action.h"
#include "rrt_planning/nh/Node.h"
#include <math.h>
#include <typeinfo>

namespace rrt_planning
{

typedef std::pair<Node*, Action> Key;
typedef std::pair<rrt_planning::Key, double> Tuple;

struct Cmp
{
    bool operator()(rrt_planning::Tuple a, rrt_planning::Tuple b)
    {
        return ((a.second < b.second)
                || (a.second == b.second && a.first.first->getCost() < b.first.first->getCost())
                || (a.second == b.second && a.first.first->getCost() == b.first.first->getCost() && eigenOrdering(a.first.first->getState(),b.first.first->getState()))
                || (a.second == b.second && a.first.first->getCost() == b.first.first->getCost() && a.first.first->getState() == b.first.first->getState()
                    && eigenOrdering(a.first.second.getState(), b.first.second.getState())));
    }

    bool eigenOrdering(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const
    {
      return ((a(0) < b(0)) || (a(0) == b(0) && a(1) < b(1))
              || (a(0) == b(0) && a(1) == b(1) && a(2) < b(2)));
    }

};

}
class OpenList
{
public:
    OpenList() {}

    void insert(const rrt_planning::Key& key, double value)
    {
        rrt_planning::Tuple new_pair(key, value);
        open.insert(new_pair);
    }

    rrt_planning::Key pop()
    {
      auto it = open.begin();
      auto ptr = *it;
      open.erase(it);

      return ptr.first;
    }

    bool empty() const {return open.empty();}
    int size() {return open.size();}
    void clear() {open.clear();}

    //~OpenList();

private:
    std::set<rrt_planning::Tuple, rrt_planning::Cmp> open;
};

#endif // OPENLIST_H
