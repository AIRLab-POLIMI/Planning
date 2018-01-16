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

AbstractPlanner::~AbstractPlanner()
{

}

};
