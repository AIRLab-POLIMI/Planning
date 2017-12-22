#ifndef INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_BERNOULLIEXPONENTIALSAMPLER_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_BERNOULLIEXPONENTIALSAMPLER_H_

#include "rrt_planning/sampling/angle/SamplingAngle.h"

namespace rrt_planning
{
class BernoulliExponentialSampler : public SamplingAngle
{
    public:
        BernoulliExponentialSampler() {}
        virtual double sample() override;
        virtual ~BernoulliExponentialSampler() {}
    private:
        static std::random_device rd;
        static std::mt19937 gen;
};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_BERNOULLIEXPONENTIALSAMPLER_H_ */
