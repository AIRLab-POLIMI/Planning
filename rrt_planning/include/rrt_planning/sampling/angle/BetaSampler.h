#ifndef INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_BETASAMPLER_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_BETASAMPLER_H_

#include "rrt_planning/sampling/angle/SamplingAngle.h"

namespace rrt_planning
{
class BetaSampler : public SamplingAngle
{
    public:
        BetaSampler() {}
        virtual double sample() override;
        virtual ~BetaSampler() {}
    private:
        static std::random_device rd;
        static std::mt19937 gen;
};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_BETASAMPLER_H_ */
