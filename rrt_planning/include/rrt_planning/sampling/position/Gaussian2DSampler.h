#ifndef INCLUDE_RRT_PLANNING_SAMPLING_POSITION_GAUSSIAN2DSAMPLER_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_POSITION_GAUSSIAN2DSAMPLER_H_

#include "rrt_planning/sampling/position/SamplingPosition.h"

namespace rrt_planning
{
class Gaussian2DSampler : public SamplingPosition
{
    public:
        Gaussian2DSampler() {}
        Gaussian2DSampler(double lambda) : lambda(lambda) {}
        virtual Eigen::VectorXd sample(const Eigen::VectorXd& corner, bool cw) override;
        virtual ~Gaussian2DSampler() {}
    private:
        double lambda;
        static std::random_device rd;
        static std::mt19937 gen;

};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_POSITION_GAUSSIAN2DSAMPLER_H_ */
