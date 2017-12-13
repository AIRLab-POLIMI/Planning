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

#ifndef INCLUDE_RRT_PLANNING_DISTANCE_DISTANCE_H_
#define INCLUDE_RRT_PLANNING_DISTANCE_DISTANCE_H_

#include <Eigen/Dense>

namespace rrt_planning
{

class Distance
{
public:
    virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) = 0;
    virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, double length) = 0;
    virtual ~Distance()
    {

    }
};

class L2Distance : public Distance
{
public:
    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) override
    {
        return (x1.head(2)-x2.head(2)).norm();
    }

    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, double length) override
    {
        return (x1.head(2)-x2.head(2)).norm();
    }
};


class L2ThetaDistance : public Distance
{
public:
    L2ThetaDistance(double wt = 1.0, double wr = 0.05) : wt(wt), wr(wr){}

    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) override
    {
        double poseDistance = (x1.head(2)-x2.head(2)).squaredNorm();
        double angleDistance = std::pow(1.0 - std::cos(x1(2) - x2(2)), 2);

        return wt*poseDistance + wr*angleDistance;
    }

    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, double length) override
    {
        double poseDistance = (x1.head(2)-x2.head(2)).squaredNorm();
        double angleDistance = std::pow(1.0 - std::cos(x1(2) - x2(2)), 2);

        return wt*poseDistance + wr*angleDistance;
    }

private:
    const double wt;
    const double wr;

};

class WeightedL2ThetaDistance: public Distance
{
public:
    WeightedL2ThetaDistance(double wt = 1.0, double r_min = 0.01, double r_max = 0.1): wt(wt), r_min(r_min), r_max(r_max) {}

    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) override
    {
        return 0;
    }

    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, double length)
    {
        double poseDistance = (x1.head(2)-x2.head(2)).squaredNorm();
        double angleDistance = std::pow(1.0 - std::cos(x1(2) - x2(2)), 2);
        double t = (length -  poseDistance) / length;
        double wr = (1 - t)* r_min + t * r_max;

        return wt * poseDistance + wr * angleDistance;
    }
private:
    const double wt;
    const double r_min;
    const double r_max;

};

class ThetaDistance : public Distance
{
public:
    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, double length) override
    {
        return length;
    }

    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) override
    {
        double angle = x1(2) - x2(2);
        if(fabs(angle) > M_PI){
            angle = ( angle > 0 ) ? (angle - 2*M_PI) : (angle + 2*M_PI);
        }

        return fabs(angle);
}

};

}

#endif /* INCLUDE_RRT_PLANNING_DISTANCE_DISTANCE_H_ */
