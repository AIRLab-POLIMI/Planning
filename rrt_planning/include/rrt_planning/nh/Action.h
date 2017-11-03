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
    inline Action(const Cell& cell, const Eigen::VectorXd state, const Eigen::VectorXd& subgoal, bool cw, bool sample, std::shared_ptr<Action> parent):
               cell(cell), state(state), subgoal(subgoal), cw(cw), sample(sample), parent(parent) {}

    Cell getCell() const {return cell;}
    Eigen::VectorXd getState() const {return state;}
    Eigen::VectorXd getSubgoal() const {return subgoal;}
    bool isClockwise() const {return cw;}
    bool isSubgoal() const {return sample;}
    std::shared_ptr<Action> getParent() const {return parent;}
    void setParent(std::shared_ptr<Action> p) {parent = p;}

private:
    Cell cell;
    Eigen::VectorXd state;
    Eigen::VectorXd subgoal;
    bool cw;
    bool sample;
    std::shared_ptr<Action> parent;
};
}

#endif // ACTION_H
