#include "robotsUtils.h"

int getRobotTimestep(Robot *robot)
{
  return (int)robot->getBasicTimeStep();
}

double fModulo(double n, double m)
{
  while (n >= m)
    n -= m;
  return n;
}

double uniform(double min, double max)
{
  random_device rd;
  mt19937 gen(rd());
  uniform_real_distribution<> dist(min, max);

  return dist(gen);
}
