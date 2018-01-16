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

#include "rrt_planning/extenders/ExtenderFactory.h"

#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/kinematics_models/Bicycle.h"

#include "rrt_planning/kinematics_models/controllers/ConstantController.h"
#include "rrt_planning/kinematics_models/controllers/POSQ.h"

#include "rrt_planning/extenders/ClosedLoopExtender.h"
#include "rrt_planning/extenders/MotionPrimitivesExtender.h"

#include <stdexcept>

//#define DEBUG_CONF

namespace rrt_planning
{

void ExtenderFactory::initialize(ros::NodeHandle& nh, Map& map, Distance& distance)
{
    std::string extenderName;
    nh.param("extender", extenderName, std::string("MotionPrimitives"));

    if(extenderName == "MotionPrimitives")
    {
        ConstantController* controller = new ConstantController();

        this->controller = controller;

        initializeKinematic(nh);

        extender = new MotionPrimitivesExtender(*kinematicModel, *controller, map, distance);
#ifdef DEBUG_CONF
        ROS_FATAL("motion primitives");
#endif
    }
    else if(extenderName == "ClosedLoop")
    {
        std::string controllerName;

        nh.param("controller", controllerName, std::string("POSQ"));

        if(controllerName == "POSQ")
        {
            //FIXME use parameters
            double k_rho, kv, k_alpha, k_phi;
            nh.param("k_rho", k_rho, 1.0);
            nh.param("kv", kv, 1.0);
            nh.param("k_alpha", k_alpha, 1.0);
            nh.param("k_phi", k_phi, 1.0);
            controller = new POSQ(k_rho, kv, k_alpha, k_phi);
#ifdef DEBUG_CONF
            ROS_FATAL("controller");
#endif
        }
        else if(controllerName == "Costant")
        {
            controller = new ConstantController();
        }
        else
        {
            throw std::runtime_error("Unknown controller " + controllerName);
        }

        initializeKinematic(nh);

        extender = new ClosedLoopExtender(*kinematicModel, *controller, map, distance);
    }
    else
    {
        throw std::runtime_error("Unknown extender " + extenderName);
    }

    extender->initialize(nh);

}

void ExtenderFactory::initializeKinematic(ros::NodeHandle& nh)
{
    std::string kinematicModelName;
    nh.param("kinematicModel", kinematicModelName, std::string("DifferentialDrive"));

    if(kinematicModelName == "DifferentialDrive")
    {
        kinematicModel = new DifferentialDrive(*controller);
#ifdef DEBUG_CONF
        ROS_FATAL("DifferentialDrive");
#endif
    }
    else if(kinematicModelName == "Bicycle")
    {
        kinematicModel = new Bicycle(*controller);
#ifdef DEBUG_CONF
        ROS_FATAL("Bicycle");
#endif
    }

}

ExtenderFactory::~ExtenderFactory()
{
    if(extender)
        delete extender;

    if(kinematicModel)
        delete kinematicModel;

    if(controller)
        delete controller;
}


}
