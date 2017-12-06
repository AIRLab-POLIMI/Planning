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

#include "rrt_planning/extenders/ClosedLoopExtender.h"

using namespace Eigen;

namespace rrt_planning
{

ClosedLoopExtender::ClosedLoopExtender(KinematicModel& model, Controller& controller,
                                       Map& map, Distance& distance) :
    Extender(map, distance), model(model), controller(controller)
{
    deltaT = 0;
    loopN = 0;
}

bool ClosedLoopExtender::compute(const VectorXd& x0, const VectorXd& xRand, VectorXd& xNew)
{
    controller.setGoal(xRand);

    VectorXd xStart = x0;

    bool valid = false;

    for(unsigned i = 0; i < loopN; i++)
    {
        VectorXd x = model.compute(xStart, deltaT);

        if(map.isFree(x))
        {
            xNew = x;
            valid = true;
            xStart = x;
        }
        else
        {
            break;
        }
    }

    return valid;
}

bool ClosedLoopExtender::check(const VectorXd& x0, const VectorXd& xGoal)
{
    controller.setGoal(xGoal);
    VectorXd xNew;

    VectorXd xStart = x0;

    bool valid = false;

    for(unsigned i = 0; i < loopN; i++)
    {
        VectorXd x = model.compute(xStart, deltaT);

        if(map.isFree(x))
        {
            xNew = x;
            valid = true;
            xStart = x;
        }
        else
        {
            break;
        }
    }

    if(valid && xNew != xGoal) {valid = false;}

    return valid;
}

bool ClosedLoopExtender::los(const VectorXd& x0, const VectorXd& xRand, VectorXd& xNew, double length)
{
  controller.setGoal(xRand);

  VectorXd xStart = x0;

  bool valid = true;

  while (valid) {
      VectorXd x = model.compute(xStart, deltaT);

      if(map.isFree(x))
      {
        xNew = x;
        xStart = x;
        if(distance(xNew, xRand) < 0.5)
          break;
      }
       else
      {
        valid = false;
      }
  }

  return valid;
}


void ClosedLoopExtender::initialize(ros::NodeHandle& nh)
{
    nh.param("deltaT", deltaT, 0.5);
    nh.param("loopN", loopN, 3);
}


ClosedLoopExtender::~ClosedLoopExtender()
{

}


}
