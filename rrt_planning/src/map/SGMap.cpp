#include "rrt_planning/map/SGMap.h"

using namespace std;
using namespace Eigen;

namespace rrt_planning
{

SGMap::SGMap(Map& map) : map(map) {}

void SGMap::initialize(ros::NodeHandle& nh)
{
    nh.param("macro", macro, 0.5);
    nh.param("micro", micro, 0.3);
    nh.param("step", step, 0.05);
    nh.param("discretization", discretization, 360);
    nh.param("threshold", threshold, 0.4);
    nh.param("ray", ray, 0.3);
    nh.param("corner_step", corner_step, 0.1);
}


bool SGMap::collisionPoints(const VectorXd& a, const VectorXd& b, vector<VectorXd>& actions)
{
    actions.clear();
    VectorXd tmp, exit_point;
    exit_point = b;

    double DX = b(0) - a(0);
    double DY = b(1) - a(1);

    double norm = sqrt(pow(DX, 2) + pow(DY, 2));
    double dx = (DX / norm) * step;
    double dy = (DY / norm) * step;
    int x_steps = DX / dx;

    VectorXd p = a;
    VectorXd old = a;
    bool curr, prev;
    curr = prev = map.isFree(p);

    for(int i = 0; i < x_steps; i++)
    {
        old = p;
        p(0) += dx;
        p(1) += dy;
        curr = map.isFree(p);

        if(curr != prev)
        {
            if(actions.empty())
            {
                actions.push_back(old);
                tmp = old;
            }
            else
            {
                if(curr)
                {
                    exit_point = p;
                    double distance = sqrt(pow(exit_point(0) - tmp(0), 2) + pow(exit_point(1) - tmp(1), 2));
                    if(distance > micro)
                    {
                        actions.push_back(exit_point);
                        return false;
                    }
                }
                else
                {
                    tmp = old;
                    double distance = sqrt(pow(exit_point(0) - tmp(0), 2) + pow(exit_point(1) - tmp(1), 2));
                    if(distance > macro)
                    {
                        actions.push_back(exit_point);
                        return false;
                    }
                }
            }

        }
        prev = curr;
    }

    if(actions.size() == 1)
    {
        actions.push_back(exit_point);
        return false;
    }

    return true;
}

VectorXd SGMap::exitPoint(const VectorXd& current, const VectorXd& middle, bool cw)
{
    Vector3d c_point(current(0), current(1), 1);
    Vector3d m_point(middle(0), middle(1), 1);
    Vector3d line = c_point.cross(m_point);
    double c = (-middle(0) * -line(1)) + (-middle(1) * line(0));
    Vector3d normal(-line(1), line(0), c);
    normal /= normal(2);

    double DX = normal(1);
    double DY = -normal(0);

    double norm = sqrt(pow(DX, 2) + pow(DY, 2));
    double dx = (DX / norm) * step;
    double dy = (DY / norm) * step;
    VectorXd p = middle;
    p(2) = 0;
    bool curr, prev;
    prev = false;

    //Check direction
    p(0) += (5*dx);
    p(1) += (5*dy);
    VectorXd a = Vector2d(middle(0)- current(0), middle(1) - current(1));
    VectorXd b = Vector2d(p(0) - current(0), p(1) - current(1));
    bool dir = clockwise(a, b);

    if(dir != cw)
    {
        dx = -dx;
        dy = -dy;
    }

    p = middle;

    //Middle point outside obstacle
    if(map.isFree(middle))
    {
        p(0) += dx;
        p(1) += dy;
        if(map.isFree(p))
        {
            return p;
        }

        return map.getOutsidePoint();
    }

    while(map.insideBound(p))
    {
        curr = map.isFree(p);
        if(curr != prev)
        {
            return p;
        }
        prev = curr;

        p(0) += dx;
        p(1) += dy;
    }

    return p;
}

vector<VectorXd> SGMap::infiniteExitPoint(const VectorXd& current, const VectorXd& middle, bool cw)
{
    Vector3d c_point(current(0), current(1), 1);
    Vector3d m_point(middle(0), middle(1), 1);
    Vector3d line = c_point.cross(m_point);
    double c = (-middle(0) * -line(1)) + (-middle(1) * line(0));
    Vector3d normal(-line(1), line(0), c);
    normal /= normal(2);

    double DX = normal(1);
    double DY = -normal(0);

    double norm = sqrt(pow(DX, 2) + pow(DY, 2));
    double dx = (DX / norm) * step;
    double dy = (DY / norm) * step;
    VectorXd p = middle;
    p(2) = 0;
    bool curr, prev;
    prev = false;

    vector<VectorXd> samples;

    //Check direction
    p(0) += (5*dx);
    p(1) += (5*dy);
    VectorXd a = Vector2d(middle(0)- current(0), middle(1) - current(1));
    VectorXd b = Vector2d(p(0) - current(0), p(1) - current(1));
    bool dir = clockwise(a, b);

    if(dir != cw)
    {
        dx = -dx;
        dy = -dy;
    }

    p = middle;

    while(map.insideBound(p))
    {
        curr = map.isFree(p);
        if(curr != prev)
        {
            if(curr)
                samples.push_back(p);
        }
        prev = curr;

        p(0) += dx;
        p(1) += dy;
    }

    return samples;
}

void SGMap::forcedUpdate(const VectorXd& a, const VectorXd& b, vector<VectorXd>& actions)
{
    actions.clear();
    double DX = b(0) - a(0);
    double DY = b(1) - a(1);
    double norm = sqrt(pow(DX, 2) + pow(DY, 2));

    double dx = (DX / norm) * step;
    double dy = (DY / norm) * step;
    VectorXd p = b;
    bool curr, prev;
    curr = prev = map.isFree(p);

    while(map.insideBound(p))
    {
        curr = map.isFree(p);
        p(0) += dx;
        p(1) += dy;

        if(curr != prev)
        {
            actions.push_back(p);
            if(actions.size() == 2)
                return;
        }

        prev = curr;
    }

    return;
}

bool SGMap::followObstacle(const VectorXd& current, const VectorXd& a, vector<VectorXd>& actions)
{
    actions.clear();
    double DX = a(0) - current(0);
    double DY = a(1) - current(1);
    double norm = sqrt(pow(DX, 2) + pow(DY, 2));
    double dx = (DX / norm) * step;
    double dy = (DY / norm) * step;
    VectorXd p = a;
    vector<VectorXd> dummy;

    while(map.insideBound(p) && map.isFree(p))
    {
        if(isCorner(p))
        {
            actions.push_back(p);
            return true;
        }
        p(0) += dx;
        p(1) += dy;
    }

    forcedUpdate(current, a, actions);
    return false;
}

bool SGMap::isCorner(const VectorXd& current)
{
    double delta = 2*M_PI / discretization;
    double angle = current(2);

    VectorXd p = current;
    p(0) = current(0) + ray*cos(angle);
    p(1) = current(1) + ray*sin(angle);
    VectorXd old = p;
    bool curr, prev;
    curr = prev = map.isFree(p);

    vector<VectorXd> points;
    for(uint i = 0; i < discretization; i++)
    {
        old = p;
        angle += delta;
        p(0) = current(0) + ray*cos(angle);
        p(1) = current(1) + ray*sin(angle);
        curr = map.isFree(p);

        if(curr != prev)
        {
            //Get collisions as points inside the obstacle
            if(curr)
                points.push_back(old);
            else
                points.push_back(p);
        }

        if(points.size() == 2)
            break;

        prev = curr;
    }

    if(points.empty()){return true;}

    //Check if it's a corner
    //middle point and traslated point must be inside obstacle
    VectorXd middle = computeMiddle(points[0], points[1]);
    if(map.isFree(middle))
    {
        return false;
    }

    double DX = current(0) - middle(0);
    double DY = current(1) - middle(1);
    double norm = sqrt(pow(DX, 2) + pow(DY, 2));
    double dx = (DX / norm) * corner_step;
    double dy = (DY / norm) * corner_step;

    middle(0) += dx;
    middle(1) += dy;

    if(map.isFree(middle))
        return false;

    return true;
}


bool SGMap::isTrueCornerWOW(const VectorXd& current)
{
    double true_step = 0.3;
    double x = current(0) + cos(current(2)) * true_step;
    double y = current(1) + sin(current(2)) * true_step;

    return map.isFree(Vector3d(x, y, 0));
}

VectorXd SGMap::computeMiddle(const VectorXd& a, const VectorXd& b)
{
    double dx = fabs(b(0) - a(0));
    double dy = fabs(b(1) - a(1));
    VectorXd middle = a;

    middle(0) = (b(0) > a(0)) ? (a(0) + dx/2) : (b(0) + dx/2);
    middle(1) = (b(1) > a(1)) ? (a(1) + dy/2) : (b(1) + dy/2);

    return middle;
}

bool SGMap::clockwise(const VectorXd& a, const VectorXd& b)
{
    double angle = atan2(b(1), b(0)) - atan2(a(1), a(0));
    if(fabs(angle) > M_PI){
        angle = ( angle > 0 ) ? (angle - 2*M_PI) : (angle + 2*M_PI);
    }
    return (angle < 0);
}

SGMap::~SGMap()
{
}

}
