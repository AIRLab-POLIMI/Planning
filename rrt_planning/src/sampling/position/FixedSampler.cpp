#include "rrt_planning/sampling/position/FixedSampler.h"

namespace rrt_planning
{
    double FixedSampler::sample()
    {
        return deltaX;
    }
}
