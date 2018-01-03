#include "rrt_planning/sampling/angle/AtanSampler.h"

namespace rrt_planning
{
    std::random_device AtanSampler::rd;
    std::mt19937 AtanSampler::gen(rd());

    double AtanSampler::sample()
    {
        std::normal_distribution<double> d_sample(0.0, 0.1);
        double sample = d_sample(gen);
        double magics = atan(sample);

        double direction = 2 * magics ;

        return direction;
    }
    
}

