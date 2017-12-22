#include "rrt_planning/sampling/position/FixedSampler.h"

namespace rrt_planning
{
    Eigen::VectorXd FixedSampler::sample(const Eigen::VectorXd& corner, bool cw)
    {
        double theta = cw ? corner(2) - M_PI/2 : corner(2) + M_PI/2;
        Eigen::VectorXd new_state = Eigen::Vector3d(corner(0) + deltaX*cos(theta), corner(1) + deltaX*sin(theta), corner(2));

        return new_state;
    }
}
