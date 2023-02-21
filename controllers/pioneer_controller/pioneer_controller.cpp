#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Camera.hpp>
#include <cmath>
#include <vector>
#include "sensors.h"
#include "motor.h"
#include "robotController.h"
#include "communication.h"
#include <webots/Display.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

using namespace webots;
using namespace std;

int main(int argc, char **argv)
{

  // create the Robot instance.
  Robot *robot = new Robot();


  RobotController *robotController = new RobotController(robot);

  // SensorsController *sensorsController = new SensorsController(robot);
  // sensorsController->initSensors();

  MotorController *motorController = new MotorController(robot);

  PositionController *positionController = new PositionController(robot);

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  // Camera *camera = robot->getCamera("camera");
  // camera->enable(timeStep);
  bool f = true;
  bool of = true;
  const vector<double> dest = {-3.84202,9.50867};

  int count = 0;
  // vector<double> midPoint = {-2.55493, -5.52426};
  vector<double> midPoint = {-3.95704, 9.56572};
  while (robot->step(timeStep) != -1)
  {
    // if (robot->getName() == "robot1")
    // {
    //   if (of)
    //   {
    //     if (robotController->gotoDestination())
    //     {
    //       vector<double> loc = positionController->getRobotCoordinates();
    //       robotController->addLocationToVisitedPath(loc);
    //       cout << "reached" << endl;
    //       of = false;
    //       continue;
    //     }

    //     // motorController->moveToDestination(dest);
    //     // motorController->motorRotateRightInDegree(10);
    //     robotController->moveToRandomLocation();
    //     vector<double> loc = positionController->getRobotCoordinates();

    //     robotController->addLocationToVisitedPath(loc);
    //     cout << count << endl;
    //     count++;
    //   }
    //   else
    //   {
    //     cout << "done visiting" << endl;
    //     if (f)
    //     {
    //       cout << "end to start" << endl;
    //       robotController->followPath(false);
    //       f = false;
    //     }
    //     else
    //     {
    //       f = true;
    //       cout << "start to end" << endl;
    //       robotController->followPath(true);
    //     }
    //   }
    // }
    // // // cout<<robotController->isDestinationReachable(dest)<<endl;
    // // cout<<"inn"<<endl;
    // sensorsController->getSensorsCondition();
    // cout<<robotController->isDestinationReachable(dest)<<endl;
    // vector<vector<double>> pC = getPerimeterCoords(midPoint[0],midPoint[1],2);
    //
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
