#include "rrt_planning/sampling/angle/BetaSampler.h"
#include <boost/math/distributions/beta.hpp>

namespace rrt_planning
{
    std::random_device BetaSampler::rd;
    std::mt19937 BetaSampler::gen(rd());

    double BetaSampler::sample()
    {
        boost::math::beta_distribution<double> b_dist(5.0, 5.0);
        std::uniform_real_distribution<double> u_dist(0, 1);
        double u_sample = u_dist(gen);

        double sample = quantile(b_dist, u_sample);

        double direction = sample * 2 * M_PI - M_PI;

        return direction;
    }

}
