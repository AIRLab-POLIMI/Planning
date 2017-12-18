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

#ifndef INCLUDE_RRT_PLANNING_UTILS_RANDOMGENERATOR_H_
#define INCLUDE_RRT_PLANNING_UTILS_RANDOMGENERATOR_H_

#include <random>

namespace rrt_planning
{

class RandomGenerator
{
public:
    static bool sampleEvent(double p);
    static double sampleUniform(double a, double b);
    static double sampleExponential(double lambda);
    static double sampleAngle();

private:
    static std::random_device rd;
    static std::mt19937 gen;
};

}

#endif /* INCLUDE_RRT_PLANNING_UTILS_RANDOMGENERATOR_H_ */
