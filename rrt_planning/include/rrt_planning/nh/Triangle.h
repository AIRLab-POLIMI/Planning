#ifndef INCLUDE_RRT_PLANNING_NH_TRIANGLE_H
#define INCLUDE_RRT_PLANNING_NH_TRIANGLE_H

#include <Eigen/Dense>

namespace rrt_planning
{
  struct Triangle
  {
    inline Triangle(){}
    inline Triangle(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
      const Eigen::VectorXd& c): a(a), b(b), c(c)
    {
        double magics = (-b(1) * c(0) + a(1) * (-b(0) + c(0)) + a(0) * (b(1) - c(1)) + b(0) * c(1));
        area = 0.5 * magics;
    }

    bool contains(const Eigen::VectorXd& p)
    {
        int sign = area < 0 ? -1 : 1;
        double s = (a(1) * c(0) - a(0) * c(1) + (c(1) - a(1)) * p(0) + (a(0) - c(0)) * p(1)) * sign;
        double t = (a(0) * b(1) - a(1) * b(0) + (a(1) - b(1)) * p(0) + (b(0) - a(0)) * p(1)) * sign;

        return ((s > 0) && (t > 0) && ((s + t) < 2 * area * sign));
    }

    Eigen::VectorXd a;
    Eigen::VectorXd b;
    Eigen::VectorXd c;
    double area;

  };


}

#endif // INCLUDE_RRT_PLANNING_NH_TRIANGLE_H
