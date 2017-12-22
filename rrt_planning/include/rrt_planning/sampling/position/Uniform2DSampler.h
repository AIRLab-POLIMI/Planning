#ifndef INCLUDE_RRT_PLANNING_SAMPLING_POSITION_UNIFORM2DSAMPLER_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_POSITION_UNIFORM2DSAMPLER_H_

#include "rrt_planning/sampling/position/SamplingPosition.h"

namespace rrt_planning
{
class Uniform2DSampler : public SamplingPosition
{
    public:
        Uniform2DSampler() {}
        Uniform2DSampler(double deltaX) : deltaX(deltaX) {}
        virtual Eigen::VectorXd sample(const Eigen::VectorXd& corner, bool cw) override;
        virtual ~Uniform2DSampler() {}
    private:
        double deltaX;
        static std::random_device rd;
        static std::mt19937 gen;

};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_POSITION_UNIFORM2DSAMPLER_H_ */
