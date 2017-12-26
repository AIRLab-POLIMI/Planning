#include "rrt_planning/sampling/position/SamplingPositionFactory.h"
#include "rrt_planning/sampling/position/LineGaussianSampler.h"
#include "rrt_planning/sampling/position/FixedSampler.h"
#include "rrt_planning/sampling/position/Uniform2DSampler.h"
#include "rrt_planning/sampling/position/Gaussian2DSampler.h"

#include <stdexcept>

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

        ROS_FATAL("Gaussian on line position sampler");
    }
    else if(samplingPositionName == "Fixed")
    {
        double deltaX;
        nh.param("deltaX", deltaX, 0.3);
        SamplingPosition* sampling = new FixedSampler(deltaX);

        this->sampling = sampling;

        ROS_FATAL("Fixed position sampler");
    }
    else if(samplingPositionName == "Uniform2D")
    {
        double deltaX;
        nh.param("deltaX", deltaX, 0.3);
        SamplingPosition* sampling = new Uniform2DSampler(deltaX);

        this->sampling = sampling;

        ROS_FATAL("Uniform2D position sampler");
    }
    else if(samplingPositionName == "Gaussian2D")
    {
        double deltaX;
        nh.param("deltaX", deltaX, 0.3);
        SamplingPosition* sampling = new Gaussian2DSampler(deltaX);

        this->sampling = sampling;

        ROS_FATAL("Gaussian 2D position sampler");
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

