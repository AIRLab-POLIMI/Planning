#include "rrt_planning/sampling/position/LineGaussianSampler.h"

namespace rrt_planning
{
    std::random_device LineGaussianSampler::rd;
    std::mt19937 LineGaussianSampler::gen(rd());

    double LineGaussianSampler::sample()
    {
        std::exponential_distribution<double> d(lambda);

        return d(gen);
    }
}
