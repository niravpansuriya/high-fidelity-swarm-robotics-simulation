#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/Supervisor.hpp>
#include <cmath>
#include <vector>
#include "sensors.h"
#include "motor.h"

using namespace webots;
using namespace std;

int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // // SensorsController *sensorsController = new SensorsController(robot);
  MotorController *motorController = new MotorController(robot);

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  bool f = true;
  const double dest[2] = {-8.45689, -8.0107};

  while (robot->step(timeStep) != -1)
  {
    if (f)
    {
      // motorController->moveToDestination(dest);
      motorController->motorRotateLeftInDegree(180);


      f = false;
    }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
