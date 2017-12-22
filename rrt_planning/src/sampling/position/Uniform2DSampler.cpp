#include "rrt_planning/sampling/position/Uniform2DSampler.h"

namespace rrt_planning
{
    std::random_device Uniform2DSampler::rd;
    std::mt19937 Uniform2DSampler::gen(rd());

    Eigen::VectorXd Uniform2DSampler::sample(const Eigen::VectorXd& corner, bool cw)
    {
        std::uniform_real_distribution<double> d(0.0, deltaX);
        std::uniform_real_distribution<double> g(0.0, 2*M_PI);
        double r = d(gen);
        double a = g(gen);

        Eigen::VectorXd new_state = corner;
        new_state(0) += r * cos(a);
        new_state(1) += r * sin(a);

        return new_state;
    }
}
