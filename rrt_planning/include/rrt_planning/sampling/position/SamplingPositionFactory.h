#ifndef INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITIONFACTORY_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITIONFACTORY_H_

#include "rrt_planning/sampling/position/SamplingPosition.h"
#include <ros/ros.h>

namespace rrt_planning
{
class SamplingPositionFactory
{
    public:
        void initialize(ros::NodeHandle& nh);
        
        inline SamplingPosition& getSampling()
        {
            return *sampling;
        }
        
        ~SamplingPositionFactory();
        
    private:
        SamplingPosition* sampling;
};
}

#endif /* INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITIONFACTORY_H_ */
