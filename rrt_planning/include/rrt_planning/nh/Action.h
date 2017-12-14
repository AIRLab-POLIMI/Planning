#ifndef INCLUDE_RRT_PLANNING_NH_ACTION_H
#define INCLUDE_RRT_PLANNING_NH_ACTION_H

#include <memory>
#include <Eigen/Dense>

namespace rrt_planning
{
class Action
{
public:
    inline Action(){}
    inline Action(const Eigen::VectorXd& state, bool cw, bool sample, bool corner, std::shared_ptr<Action> parent):
                  state(state), cw(cw), sample(sample), corner(corner), parent(parent) {}

    Eigen::VectorXd getState() const {return state;}
    bool isClockwise() const {return cw;}
    bool isSubgoal() const {return sample;}
    bool isCorner() const {return corner;}
    std::shared_ptr<Action> getParent() const {return parent;}
    void setParent(std::shared_ptr<Action> p) {parent = p;}
    void setCorner(bool isCorner) {corner = isCorner;}
    void setState(Eigen::VectorXd& new_state) {state = new_state;}


private:
    Eigen::VectorXd state;
    bool cw;
    bool sample;
    bool corner;
    std::shared_ptr<Action> parent;
};
}

#endif // INCLUDE_RRT_PLANNING_NH_ACTION_H
