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
#include "angles/angles.h"
#include <iostream>

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
    L2ThetaDistance(double wt = 1.0, double wr = 0.5) : wt(wt), wr(wr){}

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
    WeightedL2ThetaDistance(double wt = 1.0, double wr_a = 0.5, double wr_p = 0.5): wt(wt), wr_a(wr_a), wr_p(wr_p) {}

    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) override
    {
        double poseDistance = (x1.head(2)-x2.head(2)).squaredNorm();
        double difference = angles::shortest_angular_distance(x1(2), x2(2));
        double angleDistance = std::pow(1.0 - std::cos(difference), 2);

        return wt*poseDistance + wr_p*angleDistance;
    }

    inline virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, double length)
    {
        double rho = (x1.head(2)-x2.head(2)).squaredNorm();
        double alpha = atan2(x2(1) - x1(1), x2(0) - x1(0));
        double phi = x2(2);
        double theta = x1(2);

        //double angleDistance = std::pow(1.0 - std::cos(x1(2) - x2(2)), 2);
        double dalpha = angles::shortest_angular_distance(alpha, phi);
        double deltaAlpha = std::pow(1.0 - std::cos(dalpha), 2);
        double dphi = angles::shortest_angular_distance(theta, phi);
        double deltaPhi = std::pow(1.0 - std::cos(dphi), 2);

        //double deltaAlpha = fabs(alpha - theta);
        //double deltaPhi = fabs(phi - theta);
        double t = (length - rho) / length;

        return wt*rho + wr_a*deltaAlpha*(1-t) + wr_p*deltaPhi*t;
    }
private:
    const double wt;
    const double wr_a;
    const double wr_p;

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
