#ifndef ROBOT_UTILS_HEADER
#define ROBOT_UTILS_HEADER

#include <webots/Robot.hpp>
#include <random>
#include "defs.h"

using namespace webots;
using namespace std;

int getRobotTimestep(Robot *robot);
double fModulo(double n, double m);
double uniform(double min, double max);

#endif