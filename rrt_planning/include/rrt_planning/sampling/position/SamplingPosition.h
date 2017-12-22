#ifndef INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITION_H_
#define INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITION_H_

#include <random>
#include <Eigen/Dense>

namespace rrt_planning
{
class SamplingPosition
{
    public:
        SamplingPosition() {}
        virtual Eigen::VectorXd sample(const Eigen::VectorXd& corner, bool cw) = 0;
        virtual ~SamplingPosition() {}

};
}
#endif /* INCLUDE_RRT_PLANNING_SAMPLING_POSITION_SAMPLINGPOSITION_H_ */
