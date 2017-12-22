#ifndef INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_SAMPLINGANGLEFACTORY_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_SAMPLINGANGLEFACTORY_H_

#include "rrt_planning/sampling/angle/SamplingAngle.h"
#include <ros/ros.h>

namespace rrt_planning
{
class SamplingAngleFactory
{
    public:
        void initialize(ros::NodeHandle& nh);
        
        inline SamplingAngle& getSampling()
        {
            return *sampling;
        }
        
        ~SamplingAngleFactory();
        
    private:
        SamplingAngle* sampling;
};
}

#endif /* INCLUDE_RRT_PLANNING_SAMPLING_ANGLE_SAMPLINGANGLEFACTORY_H_ */
