#ifndef ACTION_H
#define ACTION_H

#include <memory>
#include "rrt_planning/grid/cell.h"

class Action
{
public:
    inline Action(){}
    inline Action(const Cell& cell, const Cell& subgoal, bool cw, bool sample, std::shared_ptr<Action> parent):
               cell(cell), subgoal(subgoal), cw(cw), sample(sample), parent(parent) {}

    Cell getCell() const {return cell;}
    Cell getSubgoal() const {return subgoal;}
    bool isClockwise() const {return cw;}
    bool isSubgoal() const {return sample;}
    std::shared_ptr<Action> getParent() const {return parent;}
    void setParent(std::shared_ptr<Action> p) {parent = p;}

private:
    Cell cell;
    Cell subgoal;
    bool cw;
    bool sample;
    std::shared_ptr<Action> parent;
};

#endif // ACTION_H
