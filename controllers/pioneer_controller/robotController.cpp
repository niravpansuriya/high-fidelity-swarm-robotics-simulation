#include "robotController.h"

RobotController::RobotController(Robot *robot)
{
    this->robot = robot;
    this->motorController = new MotorController(robot);
    this->positionController = new PositionController(robot);
    this->sensorController = new SensorsController(robot);
    sensorController->initSensors();
}

void RobotController::moveToRandomLocation()
{
    int timeStep = getRobotTimestep(robot);
    int failedAttemps = 0;
    while (true)
    {
        double maxRadian = MAX_BOT_TURN_RADIAN;
        if ((0.5 * failedAttemps * 0.25) < 2 * PI)
        {
            maxRadian += failedAttemps * 0.05;
        }
        else
        {
            maxRadian += 2 * PI;
        }

        double theta = uniform(-1 * maxRadian, maxRadian);
        cout << maxRadian << " " << theta << " " << convertRadToDegree(fabs(theta)) << endl;
        if (theta < 0)
        {
            motorController->motorRotateLeftInDegree(convertRadToDegree(-1 * theta));
        }
        else
        {
            motorController->motorRotateRightInDegree(convertRadToDegree(theta));
        }

        // check if there is any thing in the way
        if (sensorController->isSomethingInFront() ||
            sensorController->isSomethingInFrontLeft() ||
            sensorController->isSomethingInFrontRight())
        {
            cout << "failedAttemps: " << endl;
            failedAttemps += 1;
        }
        else
        {
            // move forward
            motorController->motorMoveForward(ROBOT_SPEED);
            int count = 0;
            while (robot->step(timeStep) != -1)
            {
                if (count++ >= MAX_STEP_TIME)
                {
                    cout << "1" << endl;
                    break;
                }
                // cout<<"======="<<endl;
                // cout << sensorController->isSomethingInFront() << " " << sensorController->isSomethingInFrontLeft() << " " << sensorController->isSomethingInFrontRight() << endl;
                // cout<<"======="<<endl;

                if (sensorController->isSomethingInFront() ||
                    sensorController->isSomethingInFrontLeft() ||
                    sensorController->isSomethingInFrontRight())
                {
                    cout << "2" << endl;
                    break;
                }
            }
            motorController->stopMotor();
            break;
        }
    }
}

void RobotController::addLocationToVisitedPath(vector<double> location)
{
    path.push_back(location);
}

void RobotController::followPath(string origin)
{
    if (!path.size())
        return;

    if (origin == "start")
    {
        for (int i = 0; i < path.size(); i++)
        {
            vector<double> destination = {path[i][0], path[i][1]};
            motorController->moveToDestination(destination);
        }
    }
    else if (origin == "end")
    {
        for (int i = path.size() - 1; i >= 0; i--)
        {
            vector<double> destination = {path[i][0], path[i][1]};
            motorController->moveToDestination(destination);
        }
    }
}
