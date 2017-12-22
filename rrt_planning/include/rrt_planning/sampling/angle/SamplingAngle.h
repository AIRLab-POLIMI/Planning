#ifndef INCLUDE_RRT_PLANNING_UTILS_SAMPLINGANGLE_H_
#define INCLUDE_RRT_PLANNING_UTILS_SAMPLINGANGLE_H_

#include <random>

namespace rrt_planning
{
class SamplingAngle
{
    public:
        SamplingAngle() {}
        virtual double sample() = 0;
        virtual ~SamplingAngle() {}
    
};
}
#endif /* INCLUDE_RRT_PLANNING_UTILS_SAMPLINGANGLE_H_ */
