#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/Supervisor.hpp>
#include <cmath>
#include <vector>
#include "sensors.h"
#include "motor.h"
#include "robotController.h"

using namespace webots;
using namespace std;

int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  RobotController *robotController = new RobotController(robot);

  SensorsController *sensorsController = new SensorsController(robot);
  sensorsController->initSensors();

  MotorController *motorController = new MotorController(robot);

  PositionController *positionController = new PositionController(robot);

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  bool f = true;
  const double dest[2] = {-8.45689, -8.0107};

  int count = 0;
  while (robot->step(timeStep) != -1)
  {
    if (f && count++ <= 40)
    {
      // motorController->moveToDestination(dest);
      // motorController->motorRotateRightInDegree(10);
      robotController->moveToRandomLocation();
      vector<double> loc = positionController->getRobotCoordinates();
      robotController->addLocationToVisitedPath(loc);
      cout << count << endl;
    }
    else
    {
      cout << "done visiting" << endl;
      if (f)
      {
        robotController->followPath("end");
        f = false;
      }
      else
      {
        f = true;
        robotController->followPath("start");
      }
      cout << "done visiting" << endl;
    }
    // cout << "==================================" << endl;
    // motorController->motorMoveForward(6);
    // sensorsController->getSensorsCondition();
    // cout << "front: " << sensorsController->isSomethingInFront() << endl;
    // cout << "left: " << sensorsController->isSomethingInFrontLeft() << endl;
    // cout << "right: " << sensorsController->isSomethingInFrontRight() << endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
