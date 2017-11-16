/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rrt_planning/map/ROSMap.h"

#include <costmap_2d/cost_values.h>


namespace rrt_planning
{

ROSMap::ROSMap(costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros(costmap_ros),
    costmap(costmap_ros->getCostmap())
{
    bounds.minX = costmap->getOriginX();
    bounds.minY = costmap->getOriginY();
    bounds.maxX = bounds.minX + costmap->getSizeInMetersX();
    bounds.maxY = bounds.minY + costmap->getSizeInMetersY();

    bounds.minZ = 0;
    bounds.maxZ = 0;
}

bool ROSMap::isFree(const Eigen::VectorXd& p)
{
    return getCost(p) <= costmap_2d::FREE_SPACE;
}

unsigned char ROSMap::getCost(const Eigen::VectorXd& p)
{
    double wx = p(0);
    double wy = p(1);

    unsigned int mx;
    unsigned int my;

    if(costmap->worldToMap(wx, wy, mx, my))
        return costmap->getCost(mx, my);
    else
    {
        std::cerr << "out of map" << std::endl;
        return costmap_2d::NO_INFORMATION;
    }

}

bool ROSMap::collisionPoints(const Eigen::VectorXd& a, const Eigen::VectorXd& b, std::vector<Eigen::VectorXd>& actions)
{
  double DX = b(0) - a(0);
  double DY = b(1) - a(1);

  //FIXME use parameters
  double step = 0.05;

  int k = floor(sqrt(pow(DX, 2) + pow(DY, 2)) / step);
  double dx = DX / k;
  double dy = DY / k;
  Eigen::VectorXd p = a;
  bool curr, prev;
  curr = prev = isFree(p);

  for(uint i = 0; i < k; i++)
  {
    p(0) += dx;
    p(1) += dy;
    curr = isFree(p);

    if(curr != prev)
    {
      actions.push_back(p);
      if(actions.size() == 2)
        return false;
    }
    prev = curr;
  }

  return true;

}

Eigen::VectorXd ROSMap::exitPoint(const Eigen::VectorXd& current, const Eigen::VectorXd& middle, bool cw)
{
  Eigen::Vector3d c_point(current(0), current(1), current(2));
  Eigen::Vector3d m_point(middle(0), middle(1), middle(2));
  Eigen::Vector3d line = c_point.cross(m_point);
  double c = (-middle(0) * -line(1)) + (-middle(1) * line(0));
  Eigen::Vector3d normal(-line(1), line(0), c);
  normal /= normal(2);

  double DX = normal(1);
  double DY = -normal(0);

  //FIXME use parameters
  double step = 0.05;

  int k = floor(sqrt(pow(DX, 2) + pow(DY, 2)) / step);
  double dx = DX / k;
  double dy = DY / k;
  Eigen::VectorXd p = middle;
  bool curr, prev;
  prev = isFree(p);

  //Check direction
  p(0) += dx;
  p(1) += dy;
  bool dir = clockwise(middle - current, p - current);
  if(dir != cw)
  {
      dx = -dx;
      dy = -dy;
      p = middle;
      p(0) += dx;
      p(1) += dy;
  }

  while (insideBound(p))
  {
      curr = isFree(p);
      if(curr != prev)
      {
        return p;
      }
      prev = curr;

      p(0) += dx;
      p(1) += dy;
  }

  return Eigen::Vector3d(-1, -1, -1);
}

bool ROSMap::forcedUpdate(const Eigen::VectorXd& a, const Eigen::VectorXd& b, std::vector<Eigen::VectorXd>& actions)
{
  double DX = b(0) - a(0);
  double DY = b(1) - a(1);

  //FIXME use parameters
  double step = 0.05;

  int k = floor(sqrt(pow(DX, 2) + pow(DY, 2)) / step);
  double dx = DX / k;
  double dy = DY / k;
  Eigen::VectorXd p = b;
  bool curr, prev;
  curr = prev = isFree(p);

  while (insideBound(p))
  {
      curr = isFree(p);
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

bool ROSMap::isCorner(const Eigen::VectorXd& current)
{
  double step = 0.3;
  double x = current(0) + cos(current(2)) * step;
  double y = current(1) + sin(current(2)) * step;

  return isFree(Eigen::Vector3d(x, y, current(2)));
}

Eigen::VectorXd ROSMap::computeMiddle(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
  double dx = fabs(b(0) - a(0));
  double dy = fabs(b(1) - a(1));
  Eigen::VectorXd middle;

  middle(0) = (b(0) > a(0)) ? (a(0) + dx/2) : (b(0) + dx/2);
  middle(1) = (b(1) > a(1)) ? (a(1) + dy/2) : (b(1) + dy/2);
  middle(2) = a(2);

  return middle;
}

bool ROSMap::clockwise(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
  double angle = std::atan2(a(1), a(0)) - std::atan2(b(1), b(0));
  if(fabs(angle) > M_PI){
      angle = ( angle > 0 ) ? (angle - 2*M_PI) : (angle + 2*M_PI);
  }
  return (angle < 0);
}

bool ROSMap::insideBound(const Eigen::VectorXd& p)
{
  return ((p(0) >= bounds.minX) && (p(0) < bounds.maxX) &&
          (p(1) >= bounds.minY) && (p(1) < bounds.maxY));
}

ROSMap::~ROSMap()
{

}

}
