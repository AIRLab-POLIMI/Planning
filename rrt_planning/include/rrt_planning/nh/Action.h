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
    inline Action(const Eigen::VectorXd& state, const Eigen::VectorXd& subgoal, const Eigen::VectorXd& old, bool cw, bool sample, bool corner, std::shared_ptr<Action> parent):
                  state(state), subgoal(subgoal), old(old), cw(cw), sample(sample), corner(corner), parent(parent) {}

    Eigen::VectorXd getState() const {return state;}
    Eigen::VectorXd getSubgoal() const {return subgoal;}
    Eigen::VectorXd getOld() const {return old;}
    bool isClockwise() const {return cw;}
    bool isSubgoal() const {return sample;}
    bool isCorner() const {return corner;}
    std::shared_ptr<Action> getParent() const {return parent;}
    void setParent(std::shared_ptr<Action> p) {parent = p;}
    void setCorner(bool isCorner) {corner = isCorner;}
    void setState(Eigen::VectorXd& new_state) {state = new_state;}


private:
    Eigen::VectorXd state;
    Eigen::VectorXd subgoal;
    Eigen::VectorXd old;
    bool cw;
    bool sample;
    bool corner;
    std::shared_ptr<Action> parent;
};
}

#endif // ACTION_H
