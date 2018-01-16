#ifndef INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_FIXEDANGLESAMPLER_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_FIXEDANGLESAMPLER_H_

#include "rrt_planning/sampling/angle/SamplingAngle.h"

namespace rrt_planning
{
class FixedAngleSampler : public SamplingAngle
{
    public:
        FixedAngleSampler() {}
        virtual double sample() override;
        virtual ~FixedAngleSampler() {}
};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_FIXEDANGLESAMPLER_H_ */
