#ifndef INCLUDE_RRT_PLANNING_SAMPLING_POSITION_LINEGAUSSIANSAMPLER_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_POSITION_LINEGAUSSIANSAMPLER_H_

#include "rrt_planning/sampling/position/SamplingPosition.h"

namespace rrt_planning
{
class LineGaussianSampler : public SamplingPosition
{
    public:
        LineGaussianSampler() {}
        LineGaussianSampler(double lambda) : lambda(lambda) {}
        virtual double sample() override;
        virtual ~LineGaussianSampler() {}
    private:
        double lambda;
        static std::random_device rd;
        static std::mt19937 gen;
    
};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_POSITION_LINEGAUSSIANSAMPLER_H_ */
