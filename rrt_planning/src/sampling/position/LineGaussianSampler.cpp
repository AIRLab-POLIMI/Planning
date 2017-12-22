#include "rrt_planning/sampling/position/LineGaussianSampler.h"

namespace rrt_planning
{
    std::random_device LineGaussianSampler::rd;
    std::mt19937 LineGaussianSampler::gen(rd());

    Eigen::VectorXd LineGaussianSampler::sample(const Eigen::VectorXd& corner, bool cw)
    {
        std::exponential_distribution<double> d(lambda);
        double sample = d(gen);
        double theta = cw ? corner(2) - M_PI/2 : corner(2) + M_PI/2;
        Eigen::VectorXd new_state = Eigen::Vector3d(corner(0) + sample*cos(theta), corner(1) + sample*sin(theta), corner(2));

        return new_state;
    }
}
