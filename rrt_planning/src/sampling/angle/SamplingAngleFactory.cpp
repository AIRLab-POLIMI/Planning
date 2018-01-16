#include "rrt_planning/sampling/angle/SamplingAngleFactory.h"
#include "rrt_planning/sampling/angle/AtanSampler.h"
#include "rrt_planning/sampling/angle/BernoulliExponentialSampler.h"
#include "rrt_planning/sampling/angle/BetaSampler.h"
#include "rrt_planning/sampling/angle/FixedAngleSampler.h"

#include <stdexcept>

//#define DEBUG_CONF

namespace rrt_planning
{

void SamplingAngleFactory::initialize(ros::NodeHandle& nh)
{
    std::string samplingAngleName;
    nh.param("angle_sampler", samplingAngleName, std::string("Atan"));

    if(samplingAngleName == "Atan")
    {
        SamplingAngle* sampling = new AtanSampler();

        this->sampling = sampling;
#ifdef DEBUG_CONF
        ROS_FATAL("Atan angle sampler");
#endif
    }
    else if(samplingAngleName == "BernoulliExponential")
    {
        SamplingAngle* sampling = new BernoulliExponentialSampler();

        this->sampling = sampling;
#ifdef DEBUG_CONF
        ROS_FATAL("Bernoulli Exponential angle sampler");
#endif
    }
    else if(samplingAngleName == "Beta")
    {
        SamplingAngle* sampling = new BetaSampler();

        this->sampling = sampling;
#ifdef DEBUG_CONF
        ROS_FATAL("Beta angle sampler");
#endif
    }
    else if(samplingAngleName == "Fixed")
    {
        SamplingAngle* sampling = new FixedAngleSampler();

        this->sampling = sampling;
#ifdef DEBUG_CONF
        ROS_FATAL("Fixed angle sample");
#endif
    }
    else
    {
        throw std::runtime_error("Unknown angle sampler " + samplingAngleName);
    }

}


SamplingAngleFactory::~SamplingAngleFactory()
{
    if(sampling)
        delete sampling;
}


}
