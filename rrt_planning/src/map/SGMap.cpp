#include "rrt_planning/map/SGMap.h"

using namespace std;
using namespace Eigen;

namespace rrt_planning
{

SGMap::SGMap(Map& map, int discretization, double ray, double threshold) : map(map),
      discretization(discretization), ray(ray), threshold(threshold) {}


bool SGMap::collisionPoints(const VectorXd& a, const VectorXd& b, vector<VectorXd>& actions)
{
  actions.clear();
  double DX = b(0) - a(0);
  double DY = b(1) - a(1);

  //FIXME use parameters
  double step = 0.05;

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
      //Get collisions as points outside the obstacle
      if(actions.empty())
        actions.push_back(old);
      else
        actions.push_back(p);

      if(actions.size() == 2)
        return false;
    }
    prev = curr;
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

  //FIXME use parameters
  double step = 0.05;
  double norm = sqrt(pow(DX, 2) + pow(DY, 2));


  //int k = floor(sqrt(pow(DX, 2) + pow(DY, 2)) / step);

  //FML again
  //k = (k == 0) ? 1 : k;

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
  //bool dir = clockwise(middle - current, p - current);
  if(dir != cw)
  {
      dx = -dx;
      dy = -dy;
  }

  p = middle;

  if(map.isFree(middle))
    //ROS_FATAL("FATALITY");

  if(!map.insideBound(p)){
    ROS_INFO("middle outside bounds");
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

  return Vector3d(-1, -1, -1);
}

bool SGMap::forcedUpdate(const VectorXd& a, const VectorXd& b, vector<VectorXd>& actions)
{
  actions.clear();
  double DX = b(0) - a(0);
  double DY = b(1) - a(1);

  //FIXME use parameters
  double step = 0.05;
  double norm = sqrt(pow(DX, 2) + pow(DY, 2));

  //int k = floor(sqrt(pow(DX, 2) + pow(DY, 2)) / step);
  //k = (k == 0) ? 1 : k;
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
        if(actions.size() == 2) return false;
      }

      prev = curr;
  }

  return true;

}

bool SGMap::followObstacle(const VectorXd& current, const VectorXd& a, vector<VectorXd>& actions)
{
  actions.clear();
  double DX = a(0) - current(0);
  double DY = a(1) - current(1);

  //FIXME use parameters
  double step = 0.5;
  double norm = sqrt(pow(DX, 2) + pow(DY, 2));
  //int k = floor(sqrt(pow(DX, 2) + pow(DY, 2)) / step);
  //k = (k == 0) ? 1 : k;
  double dx = (DX / norm) * step;
  double dy = (DY / norm) * step;
  VectorXd p = a;
  vector<VectorXd> dummy;

  while(map.insideBound(p) && map.isFree(p))
  {
      if(isCorner(p, dummy))
      {
        actions.push_back(p);
        return true;
      }
      p(0) += dx;
      p(1) += dy;
  }
  if(!map.insideBound(p))
    ROS_FATAL("KEK");

  ROS_FATAL("Start forced update");

  bool check = forcedUpdate(current, a, actions);
  if(check){
    ROS_FATAL("NOPE");
  }
  return false;

}

bool SGMap::isTrueCornerWOW(const VectorXd& current)
{
  double step = 0.3;
  double x = current(0) + cos(current(2)) * step;
  double y = current(1) + sin(current(2)) * step;

  return map.isFree(Vector3d(x, y, current(2)));
}

bool SGMap::isCorner(const VectorXd& current, vector<VectorXd>& points)
{
    points.clear();
    int count = 0;
    VectorXd point = current;
    double delta = 2*M_PI / discretization;
    double angle = current(2);

    for(uint i = 0; i < discretization; i++)
    {
      angle += delta;
      point(0) = current(0) + ray*cos(angle);
      point(1) = current(1) + ray*sin(angle);
      points.push_back(point);
      if(!map.isFree(point)){
        count++;
        if(count >= threshold*discretization)
          return false;
      }

    }

    return true;
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
