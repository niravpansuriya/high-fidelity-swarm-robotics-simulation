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
#include "defs.h"
#include "FileManager.h"
#include <webots/Display.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Camera.hpp>
#include <webots/ImageRef.hpp>

using namespace webots;
using namespace std;

int main(int argc, char **argv)
{

  // create the Robot instance.
  Robot *robot = new Robot();

  string robotId = getRobotId((robot->getName()));
  // FileManager *fileManager = new FileManager("./data", robotId + ".csv");

  RobotController *robotController = new RobotController(robot, robotId);

  SensorsController *sensorsController = new SensorsController(robot);
  sensorsController->initSensors();

  MotorController *motorController = new MotorController(robot);
  PositionController *positionController = new PositionController(robot);

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  // // Camera *camera = robot->getCamera("camera");
  // // camera->enable(timeStep);
  // bool f = true;
  // bool of = true;

  Emitter *emitter = robot->getEmitter(robot->getName() + "_emitter");
  Receiver *receiver = robot->getReceiver(robot->getName() + "_receiver");
  receiver->enable(timeStep);
  emitter->setRange(2);

  // vector<double> midPoint = {-2.55493, -5.52426};
  // vector<double> midPoint = {-3.95704, 9.56572};
  // motorController->motorMoveForward(6);

  Communication *comm = new Communication(robot, robot->getName() + "_emitter", robot->getName() + "_receiver",
                                          robotController->getRobotId(), robotController->getRobotModeReference());
  comm->enableReceiver();

  bool f = true;

  const vector<double> dest = {-14.5021, -39.5893};
  bool forward;
  // motorController->motorMoveForward(6);
  int count = 0;

  // Display *display = robot->getDisplay("display");

  // Camera *camera = robot->getCamera("camera");
  // int width = camera->getWidth();
  // int height = camera->getHeight();
  // camera->enable(timeStep);
  // cout << width << " " << height << endl;
  // set up path for both robots
  // if (robot->getName() == "robot_3")
  // {
  //   robotController->getPath() = {
  //       {-31.2494, -28.0773},
  //       {-31.591, -29.0617},
  //       {-31.9304, -30.0668},
  //       {-32.3348, -31.0122},
  //       {-32.8497, -31.9399},
  //       {-33.3646, -32.8673},
  //       {-33.8794, -33.7948},
  //       {-34.3897, -34.7141},
  //       {-34.9046, -35.6416},
  //       {-35.4195, -36.5691},
  //       {-35.9343, -37.4966},
  //       {-36.4492, -38.424},
  //       {-36.9641, -39.3515},
  //       {-37.479, -40.279},
  //       {-37.9939, -41.2064},
  //       {-38.5088, -42.1339},
  //       {-39.0236, -43.0613},
  //       {-39.5385, -43.9888},
  //       {-40.0535, -44.9162},
  //       {-40.5684, -45.8437},
  //       {-41.0833, -46.7711},
  //       {-41.5725, -47.6522},
  //       {-40.5756, -47.5066},
  //       {-39.6864, -46.9282},
  //       {-38.7972, -46.3497},
  //       {-37.908, -45.7712},
  //       {-37.0399, -45.2448},
  //       {-36.0633, -44.7381},
  //       {-35.0855, -44.3339},
  //       {-34.1055, -43.9726},
  //       {-33.0925, -43.657},
  //       {-32.0797, -43.3414},
  //       {-31.0669, -43.0259},
  //       {-30.0541, -42.7103},
  //       {-29.0414, -42.3948},
  //   };
  // }
  // else
  // {
  //   robotController->getPath() = {
  //       {-42.9054, -29.6857},
  //       {-42.3786, -30.5383},
  //       {-41.9937, -31.527},
  //       {-41.7799, -32.5209},
  //       {-41.628, -33.562},
  //       {-41.4694, -34.6589},
  //       {-41.326, -35.6507},
  //       {-41.1777, -36.6997},
  //       {-41.0299, -37.7501},
  //       {-40.8821, -38.8005},
  //       {-40.7358, -39.8414},
  //       {-40.5882, -40.8921},
  //       {-40.4408, -41.9425},
  //       {-40.2918, -42.9949},
  //       {-40.1538, -44.0415},
  //       {-40.0214, -45.0941},
  //       {-39.8889, -46.1437},
  //       {-39.7525, -47.1897},
  //       {-40.7889, -47.5713},
  //       {-41.6813, -47.101},
  //       {-41.6506, -46.0497},
  //       {-41.4078, -45.017},
  //       {-41.165, -43.9844},
  //       {-40.9221, -42.9518},
  //       {-40.627, -41.9501},
  //       {-40.1714, -41.0597},
  //       {-39.6881, -40.1154},
  //       {-39.2048, -39.1711},
  //       {-38.7215, -38.2268},
  //       {-38.2449, -37.2956},
  //       {-37.7875, -36.402},
  //       {-37.3043, -35.4577},
  //       {-36.7971, -34.4668},
  //       {-36.3135, -33.5219},
  //       {-35.8225, -32.5283},
  //       {-35.4569, -31.5515},
  //       {-35.1714, -30.5318},
  //       {-34.8864, -29.51},
  //   };
  // }

  // if (robot->getName() == "robot_2")
  //   robotController->updateMode("transport");
  // else
  // {
  //   robotController->updateMode("explore");
  // }

  // motorController->setSpeed(10, 10, 20);

  int temp = 0;
  // robotController->updateMode("transport");
  // if (robot->getName() == "robot_1")
  // {
  //   robotController->updateMode("transport");
  //   vector<vector<double>> p = {
  //     {1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{4,5}
  //   };
  //   robotController->updatePath(p);
  //   motorController->setSpeed(4,4);
  // }
  while (robot->step(timeStep) != -1)
  {
    // robotController->middleware();

    // {
    //   if (robot->getName() == "robot_1" || robot->getName() == "robot_2" || robot->getName() == "robot_3" || robot->getName() == "robot_4")
    //   {
    //     motorController->setSpeed(5,5);
    //     // robotController->moveToDestination({-48.1516,5.92378});
    //     robotController->middleware();
    //   }
    // else if(robot->getName() == "robot_2"){
    //   robotController->moveToDestination({-45.952,2.7318});
    // }
    // if (robot->getName() == "robot_3" || robot->getName() == "robot_2")
    // {
    // if (robot->getName() == "robot_1")
    // {

    if (robotController->getMode() == "explore")
    {
      // display->setColor(0xFF0000);
      robotController->exploreEnvironment();
      robotController->updateMode("transport");
      f = false;
    }
    else
    {
      // display->setColor(0x00FF00);

      if (temp % 2 == 0)
      {
        robotController->followPath(false);
      }
      else
      {
        robotController->followPath(true);
      }
      temp++;
    }
    // }

    // }
    // if (f)
    // {
    //   cout<<"here"<<endl;
    //   robotController->exploreEnvironment();
    //   f = false;
    //   robotController->updateMode("transport");
    //   cout<<"end"<<endl;
    // }
    // else{
    // if (temp % 2 == 1)
    // {
    //   robotController->followPath(false);
    // }
    // else
    // {
    //   robotController->followPath(true);
    // }
    // temp++;
    // }
    // if (f)
    // {
    //   // for(vector<double> c: robotController->getPath()){
    //   //   cout<<c[0]<<" "<<c[1]<<endl<<endl;;
    //   //   robotController->moveToDestination(c);
    //   // }
    //   const unsigned char *image = camera->getImage();
    //   int count = 0;
    //   for (int x = 0; x < width; x++)
    //   {
    //     for (int y = 0; y < height; y++)
    //     {
    //       int green = (int)camera->imageGetGreen(image, width, x, y);
    //       int red = (int)camera->imageGetRed(image, width, x, y);
    //       int blue = (int)camera->imageGetBlue(image, width, x, y);
    //       if (red < 40 && blue < 40 && green > 100)
    //       {
    //         cout << "found green" << count++ << endl;
    //       }
    //     }
    //   }
    //   // f = false;
    // }
    // if (f)
    // {
    // motorController->motorMoveForward(10);
    // if (sensorsController->isSomethingInFront())
    // {
    //   motorController->getLeftMotorReference()->setVelocity(0);
    //   motorController->getRightMotorReference()->setVelocity(0);
    //   f = false;
    // }

    // }
    // robotController->moveToRandomLocation();
    // if(f){

    //   // robotController->turnTowardDestination({-44.6707,-7.96985});
    //   // motorController->stopMotor();
    //   // motorController->motorMoveForward(6);
    //   robotController->moveToDestination({-44.6707,-7.96985});

    //   f = false;

    // }
    // if (f)
    // {
    //   // follow path
    //   robotController->followPath(true);
    //   f = false;
    // }
    // if (f)
    // {
    //   if (count == 800)
    //   {
    //     cout<<
    //     // print path
    //     cout<<"{"<<endl;
    //     for(vector<double> c: robotController->getPath()){
    //       cout<<"{"<<c[0]<<","<<c[1]<<"},"<<endl;
    //     }
    //     cout<<"}"<<endl;
    //     f = false;

    //     motorController->stopMotor();
    //     continue;
    //   }
    //   // after some count remember the path
    //   if (count++ % 40 == 0)
    //   {
    //     robotController->addLocationToVisitedPath(positionController->getRobotCoordinates());
    //   }
    // }

    // // move ahead for some steps and remember the path
    // // during the movement broadcast the wait signal
    // comm->broadcastWaitMessage();

    // // check if partner is there or not
    // if (comm->isPartnerNear())
    // {
    //   vector<int> motorState = motorController->getCurrentState();

    //   // stop robot
    //   motorController->stopMotor();

    //   // if yes, then go ahead and establish the connection
    //   if (comm->establishConnection())
    //   {

    //     vector<vector<double>> partnerPath;
    //     // if robot's mode is not explore mode, then send the path
    //     if (robotController->getMode() != "explore")
    //       comm->broadcastPath(robotController->getPath());
    //     if(comm->receivePath(partnerPath)){
    //       cout<<"path received"<<endl;
    //     }
    //   }
    // }
    // then print the path received path
    // continue movement

    // motorController->moveToDestination({-40.2961,-14.2314});
    // count++;
    // if (robotController->getMode() == "explore")
    // {
    //   robotController->addLocationToVisitedPath(positionController->getRobotCoordinates());
    //   if (count >= 30 || robotController->gotoDestination())
    //   {
    //     // go to destination
    //     robotController->addLocationToVisitedPath(positionController->getRobotCoordinates());

    //     // change robot mode
    //     robotController->updateMode("transport");

    //     motorController->stopMotor();

    //     forward = false;
    //   }
    //   else
    //   {
    //     robotController->moveToRandomLocation();
    //   }
    // }
    // else if (robotController->getMode() == "transport")
    // {
    //   cout << "forward: " << forward << endl;
    //   cout << "transport mode enabled" << endl;
    //   robotController->followPath(forward);
    //   forward = !forward;
    // }
    // // get current location

    // cout << "3 " << sensorsController->getDistanceSensorReading(3) << endl;
    // cout << "4 " << sensorsController->getDistanceSensorReading(4) << endl;

    // if (sensorsController->isSomethingInFront())
    // {
    //   motorController->stopMotor();
    // }
    // robotController->moveToRandomLocation();
    // comm->broadcastWaitMessage();
    // if (comm->isPartnerNear())
    // {
    //   // stop robot
    //   motorController->stopMotor();
    //   if (comm->establishConnection())
    //   {
    //     cout<<"connection establish successful"<<endl;
    //     vector<vector<double>> p = {{1, 2}, {10, 20}};
    //     if(robotController->getMode() != "explore")
    //     comm->broadcastPath(p);
    //     vector<vector<double>> receivedPath;

    //    if( comm->receivePath(receivedPath)){
    //     for(vector<double> c: receivedPath){
    //       cout<<c[0]<<" "<<c[1]<<endl;
    //     }
    //    }
    //   }
    //   else
    //   {
    //     cout << "connection establish not successful";
    //   }
    // }

    // if (f)
    // {
    //   // emit the data
    //   emitter->setChannel(-1);
    //   char *s = "Nirav";
    //   emitter->send(s, 5 * sizeof(char));
    // }

    // if (receiver->getQueueLength())
    // {
    //   cout << "data received: " << count++ << endl;
    //   char* d = (char*) receiver->getData();
    //   int size = receiver->getDataSize();
    //   cout<<size<<endl;
    //   for(int i=0;i<size/sizeof(char);i++){
    //     cout<<*(d+i)<<endl;
    //   }
    //   receiver->nextPacket();

    //   // stop the robot
    //   motorController->stopMotor();
    //   f = false;
    //   // double *package = (double *)receiver->getData();
    //   // int size = receiver->getDataSize() / sizeof(double);

    //   // // get path vector
    //   // vector<vector<double>> receivedPath = getDataFromPackage(package, size);

    //   // set up the path
    //   // robotController->replacePath(receivedPath);

    //   // // follow path
    //   // robotController->followPath(true);
    // }
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
