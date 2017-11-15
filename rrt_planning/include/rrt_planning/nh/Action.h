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
    inline Action(const Eigen::VectorXd& state, const Eigen::VectorXd& subgoal, bool cw, bool sample, bool corner, std::shared_ptr<Action> parent):
                  state(state), subgoal(subgoal), cw(cw), sample(sample), corner(corner), parent(parent) {}

    Eigen::VectorXd getState() const {return state;}
    Eigen::VectorXd getSubgoal() const {return subgoal;}
    bool isClockwise() const {return cw;}
    bool isSubgoal() const {return sample;}
    bool isCorner() const {return corner;}
    std::shared_ptr<Action> getParent() const {return parent;}
    void setParent(std::shared_ptr<Action> p) {parent = p;}
    

private:
    Eigen::VectorXd state;
    Eigen::VectorXd subgoal;
    bool cw;
    bool sample;
    bool corner;
    std::shared_ptr<Action> parent;
};
}

#endif // ACTION_H
