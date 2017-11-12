#ifndef ACTION_H
#define ACTION_H

#include <memory>
#include <Eigen/Dense>
#include "rrt_planning/grid/Cell.h"

namespace rrt_planning
{
class Action
{
public:
    inline Action(){}
    inline Action(const Cell& cell, const Eigen::VectorXd state, const Cell& subgoal, bool cw, bool sample, bool corner, std::shared_ptr<Action> parent):
               cell(cell), state(state), subgoal(subgoal), cw(cw), sample(sample), corner(corner), parent(parent) {}

    Cell getCell() const {return cell;}
    Eigen::VectorXd getState() const {return state;}
    Cell getSubgoal() const {return subgoal;}
    bool isClockwise() const {return cw;}
    bool isSubgoal() const {return sample;}
    bool isCorner() const {return corner;}
    std::shared_ptr<Action> getParent() const {return parent;}
    void setParent(std::shared_ptr<Action> p) {parent = p;}

private:
    Cell cell;
    Eigen::VectorXd state;
    Cell subgoal;
    bool cw;
    bool sample;
    bool corner;
    std::shared_ptr<Action> parent;
};
}

#endif // ACTION_H
