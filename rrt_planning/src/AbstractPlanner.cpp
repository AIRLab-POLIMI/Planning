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

void AbstractPlanner::computeLength(std::vector<Eigen::VectorXd>& path)
{
    length = 0;
    for(int i=0; i < path.size()-1; i++)
    {
        length += (path[i].head(2) - path[i+1].head(2)).norm();
    }
}

void AbstractPlanner::computeFirstLength(std::vector<Eigen::VectorXd>& path)
{
    first_length = 0;
    for(int i=0; i < path.size()-1; i++)
    {
        first_length += (path[i].head(2) - path[i+1].head(2)).norm();
    }
}

double AbstractPlanner::getElapsedTime()
{
    return Tcurrent.count();
}

double AbstractPlanner::getFirstLength()
{
    return first_length;
}

double AbstractPlanner::getFirstRoughness()
{
    return first_roughness;
}

std::vector<Eigen::VectorXd> AbstractPlanner::getPath()
{
    return final_path;
}


std::vector<Eigen::VectorXd> AbstractPlanner::getFirstPath()
{
    return first_path;
}


void AbstractPlanner::computeRoughness(std::vector<Eigen::VectorXd>& path)
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

void AbstractPlanner::computeFirstRoughness(std::vector<Eigen::VectorXd>& path)
{
    double r = 0;
    for(int i=0; i < path.size()-1; i++)
    {
        double deltaK = fabs((path[i](2)-path[i+1](2)) / first_length);
        double contribute = pow(deltaK,2);
        r = r + contribute;
    }
    first_roughness = r;

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
