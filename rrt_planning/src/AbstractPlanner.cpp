#include "rrt_planning/AbstractPlanner.h"

namespace rrt_planning
{

bool AbstractPlanner::timeOut()
{
    auto deltaT = std::chrono::steady_clock::now() - t0;

    if(deltaT > Tmax)
    {
        ROS_FATAL("Computational time excedeed");
        return true;
    }
    else
    {
        return false;
    }
}

double AbstractPlanner::getPathLength()
{
	return length;
}

double AbstractPlanner::getElapsedTime()
{
    return Tcurrent.count();
}

int AbstractPlanner::getDeadActions()
{
    return dead;
}

void AbstractPlanner::computeRoughness(std::vector<Eigen::VectorXd> path)
{
    double r = 0;
    for(int i=0; i < path.size()-1; i++)
    {
        double deltaK = fabs((path[i](2)-path[i+1](2)) / length);
        double contribute = pow(deltaK,2);
        r = r + contribute;
    }
    roughness = r;

    return;
}

double AbstractPlanner::getRoughness()
{
    return roughness;
}

AbstractPlanner::~AbstractPlanner()
{

}

};
