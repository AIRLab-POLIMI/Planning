#include "rrt_planning/sampling/position/Gaussian2DSampler.h"

namespace rrt_planning
{
    std::random_device Gaussian2DSampler::rd;
    std::mt19937 Gaussian2DSampler::gen(rd());

    Eigen::VectorXd Gaussian2DSampler::sample(const Eigen::VectorXd& corner, bool cw)
    {
        std::normal_distribution<double> d(lambda, 0.5);
        double sampleX = d(gen);
        double sampleY = d(gen);
        
        Eigen::VectorXd new_state = Eigen::Vector3d(corner(0) + sampleX, corner(1) + sampleY, corner(2));

        return new_state;
    }
}

