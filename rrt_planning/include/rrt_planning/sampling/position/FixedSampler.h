#ifndef INCLUDE_RRT_PLANNING_SAMPLING_POSITION_FIXEDSAMPLER_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_POSITION_FIXEDSAMPLER_H_

#include "rrt_planning/sampling/position/SamplingPosition.h"

namespace rrt_planning
{
class FixedSampler : public SamplingPosition
{
    public:
        FixedSampler() {}
        FixedSampler(double deltaX) : deltaX(deltaX) {}
        virtual Eigen::VectorXd sample(const Eigen::VectorXd& corner, bool cw) override;
        virtual ~FixedSampler() {}
    private:
        double deltaX;
};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_POSITION_FIXEDSAMPLER_H_ */
