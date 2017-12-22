#ifndef INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITION_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITION_H_

#include <random>

namespace rrt_planning
{
class SamplingPosition
{
    public:
        SamplingPosition() {}
        virtual double sample() = 0;
        virtual ~SamplingPosition() {}
    
};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITION_H_ */
