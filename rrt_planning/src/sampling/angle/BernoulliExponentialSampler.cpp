#include "rrt_planning/sampling/angle/BernoulliExponentialSampler.h"

namespace rrt_planning
{
    std::random_device BernoulliExponentialSampler::rd;
    std::mt19937 BernoulliExponentialSampler::gen(rd());

    double BernoulliExponentialSampler::sample()
    {
        std::normal_distribution<double> d_sample(0.0, 0.1);
        double sample = d_sample(gen);

        std::bernoulli_distribution d_sign(0.5);
        bool positive = d_sign(gen);
        int sign = (positive) ? 1 : -1;
        double magics = (1 - exp(-fabs(sample)));

        double direction = sign * magics ;

        return direction;
    }
}
