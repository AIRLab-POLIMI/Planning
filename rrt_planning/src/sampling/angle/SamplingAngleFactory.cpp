#include "rrt_planning/sampling/angle/SamplingAngleFactory.h"
#include "rrt_planning/sampling/angle/AtanSampler.h"
#include "rrt_planning/sampling/angle/BernoulliExponentialSampler.h"
#include "rrt_planning/sampling/angle/BetaSampler.h"
#include "rrt_planning/sampling/angle/FixedAngleSampler.h"

#include <stdexcept>

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

        ROS_FATAL("Atan angle sampler");
    }
    else if(samplingAngleName == "BernoulliExponential")
    {
        SamplingAngle* sampling = new BernoulliExponentialSampler();

        this->sampling = sampling;

        ROS_FATAL("Bernoulli Exponential angle sampler");
    }
    else if(samplingAngleName == "Beta")
    {
        SamplingAngle* sampling = new BetaSampler();

        this->sampling = sampling;

        ROS_FATAL("Beta angle sampler");
    }
    else if(samplingAngleName == "Fixed")
    {
        SamplingAngle* sampling = new FixedAngleSampler();

        this->sampling = sampling;

        ROS_FATAL("Fixed angle sample");
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
