#include "rrt_planning/sampling/position/SamplingPositionFactory.h"
#include "rrt_planning/sampling/position/LineGaussianSampler.h"
#include "rrt_planning/sampling/position/FixedSampler.h"
#include "rrt_planning/sampling/position/Uniform2DSampler.h"
#include "rrt_planning/sampling/position/Gaussian2DSampler.h"

#include <stdexcept>

//#define DEBUG_CONF

namespace rrt_planning
{

void SamplingPositionFactory::initialize(ros::NodeHandle& nh)
{
    std::string samplingPositionName;
    nh.param("position_sampler", samplingPositionName, std::string("LineGaussian"));

    if(samplingPositionName == "LineGaussian")
    {
        double deltaX;
        nh.param("deltaX", deltaX, 0.3);
        SamplingPosition* sampling = new LineGaussianSampler((1.0)/deltaX);

        this->sampling = sampling;
#ifdef DEBUG_CONF
        ROS_FATAL("Gaussian on line position sampler");
#endif
    }
    else if(samplingPositionName == "Fixed")
    {
        double deltaX;
        nh.param("deltaX", deltaX, 0.3);
        SamplingPosition* sampling = new FixedSampler(deltaX);

        this->sampling = sampling;
#ifdef DEBUG_CONF
        ROS_FATAL("Fixed position sampler");
#endif
    }
    else if(samplingPositionName == "Uniform2D")
    {
        double deltaX;
        nh.param("deltaX", deltaX, 0.3);
        SamplingPosition* sampling = new Uniform2DSampler(deltaX);

        this->sampling = sampling;
#ifdef DEBUG_CONF
        ROS_FATAL("Uniform2D position sampler");
#endif
    }
    else if(samplingPositionName == "Gaussian2D")
    {
        double deltaX;
        nh.param("deltaX", deltaX, 0.3);
        SamplingPosition* sampling = new Gaussian2DSampler(deltaX);

        this->sampling = sampling;
#ifdef DEBUG_CONF
        ROS_FATAL("Gaussian 2D position sampler");
#endif
    }
    else
    {
        throw std::runtime_error("Unknown position sampler " + samplingPositionName);
    }

}


SamplingPositionFactory::~SamplingPositionFactory()
{
    if(sampling)
        delete sampling;
}


}

