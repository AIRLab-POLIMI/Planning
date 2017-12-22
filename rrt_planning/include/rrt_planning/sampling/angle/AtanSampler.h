#ifndef INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_ATANSAMPLER_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_ATANSAMPLER_H_

#include "rrt_planning/sampling/angle/SamplingAngle.h"

namespace rrt_planning
{
class AtanSampler : public SamplingAngle
{
    public:
        AtanSampler() {}
        virtual double sample() override;
        virtual ~AtanSampler() {}
    private:
        static std::random_device rd;
        static std::mt19937 gen;
};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_ATANSAMPLER_H_ */
