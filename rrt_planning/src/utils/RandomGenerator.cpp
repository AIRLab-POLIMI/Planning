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

#include "rrt_planning/utils/RandomGenerator.h"
#include <iostream>
namespace rrt_planning
{

std::random_device RandomGenerator::rd;
std::mt19937 RandomGenerator::gen(rd());

bool RandomGenerator::sampleEvent(double p)
{
    std::bernoulli_distribution d(p);

    return d(gen);
}

double RandomGenerator::sampleUniform(double a, double b)
{
    std::uniform_real_distribution<double> d(a, b);

    return d(gen);
}

double RandomGenerator::sampleExponential(double lambda)
{
  std::exponential_distribution<double> d(lambda);

  return d(gen);
}

double RandomGenerator::sampleAngle()
{
    std::normal_distribution<double> d_sample(0.0, 0.1);
    double sample = d_sample(gen);
    //std::cerr << "normal: " << sample << std::endl;

    std::bernoulli_distribution d_sign(0.5);
    bool positive = d_sign(gen);
    int sign = (positive) ? 1 : -1;
    //double magics = (1 - exp(-fabs(sample)));
    double magics = atan(sample);

    double direction = 2 * magics ;
   // std::cerr << "magics: " << magics << std::endl;

    return direction;
}

}
