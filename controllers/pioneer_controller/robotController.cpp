#include "robotController.h"

RobotController::RobotController(Robot *robot)
{
    this->robot = robot;
    this->motorController = new MotorController(robot);
    this->positionController = new PositionController(robot);
    this->sensorController = new SensorsController(robot);
    // this->communication = new Communication(robot, robot->getName() + "_emitter", robot->getName() + "_receiver");
    this->currentPathIndex = -1;
    sensorController->initSensors();
    mode = "explore";
}

string RobotController::getMode()
{
    return mode;
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
                    break;
                }

                if (sensorController->isSomethingInFront() ||
                    sensorController->isSomethingInFrontLeft() ||
                    sensorController->isSomethingInFrontRight())
                {
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

void RobotController::followPath(bool forward)
{
    if (!path.size())
        return;

    if (forward)
    {
        for (llu i = 0; i < path.size(); i++)
        {
            currentPathIndex = i;
            cout << currentPathIndex << endl;

            vector<double> destination = {path[i][0], path[i][1]};
            motorController->moveToDestination(destination);

            // print path
            // for (int j = 0; j < path.size(); j++)
            // {
            //     cout << j << " " << path[j][0] << " " << path[j][1] << endl;
            // }

            llu nearestIndex = getNearestPointIndexForPathSmoothing(forward);
            if (nearestIndex != __LONG_LONG_MAX__ && nearestIndex - currentPathIndex >= 5)
            {
                removeVectorElements(path, currentPathIndex + 1, nearestIndex);
                cout << "Curr Size: " << path.size() << endl;
                // cout<<"index: "<<index<<endl;
                // cout<<"currentIndex: "<<currentPathIndex<<endl;
            }
        }
    }
    else
    {
        // cout<<"here"<<endl;
        for (llu i = path.size() - 1; i >= 0; i--)
        {
            currentPathIndex = i;
            cout << currentPathIndex << endl;
            // vector<double> destination = {path[i][0], path[i][1]};
            motorController->moveToDestination(path[i]);

            llu nearestIndex = getNearestPointIndexForPathSmoothing(forward);
            if (nearestIndex != __LONG_LONG_MAX__ && currentPathIndex - nearestIndex >= 5)
            {
                removeVectorElements(path, nearestIndex + 1, currentPathIndex);
                cout << "Curr Size: " << path.size() << endl;
                i = nearestIndex + 1;
                // cout<<"index: "<<index<<endl;
                // cout<<"currentIndex: "<<currentPathIndex<<endl;
            }
        }
        // cout << "here" << endl;
    }
}

llu RobotController::getNearestPointIndexForPathSmoothing(bool forward)
{
    // get robot current location
    vector<double> currLocation = positionController->getRobotCoordinates();

    // get perimeter coordinates
    // vector<vector<double>> perimeterCoordinates = getPerimeterCoords(currLocation[0], currLocation[1], PATH_SMOOTHING_RADIUS);

    // cout<<"perimeterCoordinates: ========"<<endl;
    // for(llu i=0;i<perimeterCoordinates.size();i++){
    //     cout<<i<<" "<<perimeterCoordinates[i][0]<<" "<<perimeterCoordinates[i][1]<<endl;
    // }
    // cout<<"================="<<endl;
    // // filter perimeter coordinates which are part of robot path
    // vector<pair<vector<double>, llu>> cpWithpc = getPerimeterCoordsWithPathCoords(perimeterCoordinates, path, currentPathIndex, forward);

    // get the most further coordinate in robot path direction
    // if (!cpWithpc.size())
    //     return __LONG_LONG_MAX__;

    if (forward)
    {
        for (llu i = path.size() - 1; i >= currentPathIndex; i--)
        {
            double distance = getDistance(path[i], path[currentPathIndex]);
            if (distance <= PATH_SMOOTHING_RADIUS && isDestinationReachable(path[i]))
                return i;
        }
    }
    else
    {
        // cout<<"here 1"<<endl;
        for (llu i = 0; i <= currentPathIndex; i++)
        {
            double distance = getDistance(path[i], path[currentPathIndex]);
            // cout<<distance<<endl;
            if (distance <= PATH_SMOOTHING_RADIUS && isDestinationReachable(path[i]))
                return i;
        }
    }

    return __LONG_LONG_MAX__;
}

bool RobotController::isDestinationReachable(const vector<double> dest)
{
    // motorController->turnTowardDestination(dest);

    double nearestObjDistance = sensorController->getDistanceSensorReading(3);

    double destinationDistance = getDistance(positionController->getRobotCoordinates(), dest);

    return nearestObjDistance > destinationDistance + 0.2;
}

bool RobotController::gotoDestination()
{
    const vector<double> dest = {DESTINATION_X, DESTINATION_Y};

    double d = getDistance(positionController->getRobotCoordinates(), dest);

    if (d < 5)
    {
        bool d = isDestinationReachable(dest);
        if (d)
        {
            motorController->moveToDestination(dest);
            return true;
        }
        else
        {
            return false;
        }
    }
    else
        return false;
}

void RobotController::replacePath(vector<vector<double>> path)
{
    this->path = path;
    currentPathIndex = 0;
}
