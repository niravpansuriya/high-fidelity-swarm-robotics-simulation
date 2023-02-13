#include "robotsUtils.h"

int getRobotTimestep(Robot *robot)
{
  return (int)robot->getBasicTimeStep();
}

double fModulo(double n, double m)
{
  while (n >= m)
    n -= m;
  return   n;
}