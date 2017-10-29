#ifndef OPENLIST_H
#define OPENLIST_H

#include "rrt_planning/nh/action.h"
#include "rrt_planning/nh/node.h"
#include <math.h>
#include <typeinfo>

typedef std::pair<Node*, Action> Key;
typedef std::pair<Key, double> Tuple;

struct Cmp
{
    bool operator()(Tuple a, Tuple b)
    {
        return ((a.second < b.second) || (a.second == b.second && a.first.first->getCost() < b.first.first->getCost())
                || (a.second == b.second && a.first.first->getCost() == b.first.first->getCost() && a.first.first->getCell() < b.first.first->getCell())
                || ((a.second == b.second && a.first.first->getCost() == b.first.first->getCost() && a.first.first->getCell() == b.first.first->getCell() && a.first.second.getCell() < b.first.second.getCell())));
    }
};


class OpenList
{
public:
    OpenList();

    void insert(const Key& key, double value)
    {
        Tuple new_pair(key, value);
        open.insert(new_pair);
    }

    Key pop()
    {
      auto it = open.begin();
      auto ptr = *it;
      open.erase(it);

      return ptr.first;
    }

    bool empty() const {return open.empty();}
    int size() {return open.size();}
    void clear() {open.clear();}

    ~OpenList();

private:
    std::set<Tuple, Cmp> open;
};

#endif // OPENLIST_H
