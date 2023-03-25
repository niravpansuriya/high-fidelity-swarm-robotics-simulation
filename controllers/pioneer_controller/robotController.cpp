#include "robotController.h"

RobotController::RobotController(Robot *robot, string robotId)
{
    this->robot = robot;
    this->motorController = new MotorController(robot);
    this->positionController = new PositionController(robot);
    this->sensorController = new SensorsController(robot);
    this->robotId = robotId;
    this->communication = new Communication(robot, robot->getName() + "_emitter", robot->getName() + "_receiver", getRobotId(), getRobotModeReference());
    this->currentPathIndex = -1;
    sensorController->initSensors();
    mode = "explore";

    // trackerApp = new TrackerApp(robot->getDisplay("display"));
}

void RobotController::updateMode(string mode)
{
    if (mode == "explore" || mode == "transport")
    {
        this->mode = mode;
    }
}

string RobotController::getMode()
{
    return mode;
}

void RobotController::exploreEnvironment()
{
    int timeStep = getRobotTimestep(robot);

    int speed;

    string last = "";
    double leftF = 1, rightF = 1;
    int l = uniform(20, 40);
    // int c = 0;
    int count = 0;
    int oppoCount = 0;
    vector<double> lastCords;
    double lastCheck = -1;

    while (robot->step(timeStep) != -1)
    {
        // if (c >= l)
        // {
        //     motorController->setSpeed(0, 0, 20);
        //     // print path
        //     cout<<robot->getName()<<endl;
        //     cout << "{" << endl;
        //     for (vector<double> c : path)
        //     {
        //         cout << "{" << c[0] << "," << c[1] << "}," << endl;
        //     }
        //     cout << "}" << endl;
        //     return;
        // }

        middleware();

        if (mode == "transport")
            return;
        vector<double> currLocation = positionController->getRobotCoordinates();

        if (!lastCords.size())
        {
            lastCords = currLocation;
            lastCheck = robot->getTime();
        }
        else if (robot->getTime() - lastCheck >= 10)
        {
            // cout<<"Here "<<cartesianIsCoordinateEqual(currLocation, lastCords)<<endl;
            // cout<<"currLocation "<<currLocation[0]<<" "<<currLocation[1]<<endl;
            // cout<<"lastCords "<<lastCords[0]<<" "<<lastCords[1]<<endl;
            if (cartesianIsCoordinateEqual(currLocation, lastCords))
            {
                // go back
                int count = 0;
                motorController->setSpeed(-2, -2);

                while (robot->step(timeStep) != -1)
                {
                    // cout<<"inn"<<endl;
                    if (count++ >= 50)
                        break;
                }

                // turn right
                motorController->setSpeed(2, -2);
                while (robot->step(timeStep) != -1)
                {
                    // cout<<"inn"<<endl;
                    if (count++ >= 50)
                        break;
                }
            }
            lastCheck = robot->getTime();
            // cout<<"lastCheck updated "<<endl;
            lastCords = currLocation;
        }

        if (oppoCount-- >= 0)
        {
            // leftF = -0.1;
            // rightF = 0.1;
            motorController->setSpeed(-1, 1, 1);
            continue;
        }

        if (count-- <= 0)
        {
            leftF = 1;
            rightF = 1;
        }

        if (path.size() == 0)
            addLocationToVisitedPath(currLocation);
        else if (getDistance(path[path.size() - 1], currLocation) >= PATH_RADIUS_THRESHOLD)
        {
            count = uniform(-18, 18);
            if (count < 0)
            {
                leftF = 0.8;
                rightF = 1;
            }
            else
            {
                leftF = 1;
                rightF = 0.8;
            }
            // c++;
            addLocationToVisitedPath(currLocation);
        }

        if (sensorController->getDistanceAtFront() < 1 || sensorController->getDistanceAtFrontLeft() < 1 || sensorController->getDistanceAtFrontRight() < 1)
        {
            // cout<<"sensorController->getDistanceAtFront() "<<sensorController->getDistanceAtFront()<<endl;
            // cout<<"sensorController->getDistanceAtFrontLeft() "<<sensorController->getDistanceAtFrontLeft()<<endl;
            // cout<<"sensorController->getDistanceAtFrontRight() "<<sensorController->getDistanceAtFrontRight()<<endl;

            // cout << "near" << endl;
            count = 0;
            speed = 3;
        }
        else
        {
            speed = 10;
        }

        if ((sensorController->isSomethingInFrontLeft() &&
             sensorController->isSomethingInFrontRight() &&
             sensorController->isSomethingInLeft() &&
             sensorController->isSomethingInRight()))
        {
            count = 0;
            oppoCount = 80;
        }
        else if (sensorController->isSomethingInFront())
        {
            // get left distance and right distance
            if (last == "left")
            {
                motorController->setSpeed(1, -1, 20);
            }
            else if (last == "right")
                motorController->setSpeed(-1, 1, 20);
            else
            {
                // take random turn
                double r = uniform(-1, 1);
                if (r < 0)
                {
                    motorController->setSpeed(1, -1, 20);
                    last = "left";
                }
                else
                {
                    motorController->setSpeed(-1, 1, 20);
                    last = "right";
                }
            }
        }
        else if (sensorController->isSomethingInFrontLeft())
        {
            // cout<<"left"<<endl;
            last = "left";

            motorController->setSpeed(1, 0, 20);
        }
        else if (sensorController->isSomethingInFrontRight())
        {
            // cout<<"right"<<endl;

            last = "right";

            motorController->setSpeed(0, 1, 20);
        }
        else if (sensorController->isSomethingInLeft())
        {
            // cout<<"sleft"<<endl;

            last = "left";

            motorController->setSpeed(1, 0.5, 20);
        }
        else if (sensorController->isSomethingInRight())
        {
            // cout<<"sright"<<endl;
            last = "right";

            motorController->setSpeed(0.5, 1, 20);
            // cout<<"out"<<endl;
        }
        else
        {
            if (gotoDestination())
            {
                cout << "reached " << robot->getName() << " " << path.size() << endl;
                motorController->setSpeed(0, 0, 1);
                currLocation = positionController->getRobotCoordinates();
                addLocationToVisitedPath(currLocation);
                return;
            }
            else
            {
                last = "";
                motorController->setSpeed(speed * leftF, speed * rightF, 1);
            }
        }
    }
}

void RobotController::moveToRandomLocation()
{
    int timeStep = getRobotTimestep(robot);
    int failedAttemps = 0;
    double theta;
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
        if (failedAttemps)
        {
            if (theta < 0)
                theta = uniform(-1 * maxRadian, 0);
            else
                theta = uniform(0, maxRadian);
        }
        else
        {
            theta = uniform(-1 * maxRadian, maxRadian);
        }

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
            motorController->stopMotor();
        }
        else
        {
            // move forward
            // motorController->motorMoveForward(ROBOT_SPEED);
            motorController->setSpeed(10, 10);
            int count = 0;
            while (robot->step(timeStep) != -1)
            {
                // middleware();
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
            // motorController->stopMotor();
            break;
        }
    }
}

vector<vector<double>> &RobotController::getPath()
{
    return path;
}

void RobotController::addLocationToVisitedPath(vector<double> location)
{
    path.push_back(location);
}

void RobotController::followPath(bool forward)
{
    if (!path.size())
        return;

    currentPathIndex = getNearestPathIndex(path);

    if (forward)
    {
        cout << "follow forward" << endl;

        // get nearest Index
        // for (llu i = currentPathIndex; i < path.size(); i++)
        while (currentPathIndex != path.size())
        {
            // currentPathIndex = i;
            int i = currentPathIndex;
            // cout << robot->getName() << " " << currentPathIndex << " " << path[path.size() - 1][0] << " " << path[path.size() - 1][1] << endl;

            // cout << "currentPathIndex: " << currentPathIndex << endl;
            vector<double> destination = {path[i][0], path[i][1]};
            moveToDestination(destination);

            llu nearestIndex = getNearestPointIndexForPathSmoothing(forward);
            if ((nearestIndex != path.size() - 1 && nearestIndex != __LONG_LONG_MAX__) && nearestIndex - currentPathIndex >= 2)
            {
                removeVectorElements(path, currentPathIndex + 1, nearestIndex);
                // cout << robot->getName() << " "
                //      << "Curr Size: " << path.size() << endl;
                // cout << robot->getName() << " " << path[0][0] << " " << path[0][1] << endl;
                // cout << robot->getName() << " " << path[path.size() - 1][0] << " " << path[path.size() - 1][1] << endl;
                // cout<<"index: "<<index<<endl;
                // cout<<"currentIndex: "<<currentPathIndex<<endl;
            }
            currentPathIndex++;
        }
    }
    else
    {
        cout << "follow reverse" << endl;
        // cout<<"here"<<endl;
        // for (llu i = currentPathIndex; i >= 0; i--)
        while (currentPathIndex >= 0)
        {
            // currentPathIndex = i;
            int i = currentPathIndex;
            // cout << robot->getName() << " " << currentPathIndex << " " << path[path.size() - 1][0] << " " << path[path.size() - 1][1] << endl;

            // vector<double> destination = {path[i][0], path[i][1]};
            moveToDestination(path[i]);
            llu nearestIndex = getNearestPointIndexForPathSmoothing(forward);
            if ((nearestIndex != 0 && nearestIndex != __LONG_LONG_MAX__) && currentPathIndex - nearestIndex >= 2)
            {
                removeVectorElements(path, nearestIndex + 1, currentPathIndex);
                // cout << robot->getName() << " "
                //      << "Curr Size: " << path.size() << endl;
                // cout << robot->getName() << " " << path[0][0] << " " << path[0][1] << endl;
                // cout << robot->getName() << " " << path[path.size() - 1][0] << " " << path[path.size() - 1][1] << endl;
                currentPathIndex = nearestIndex + 1;
                // cout<<"index: "<<index<<endl;
                // cout<<"currentIndex: "<<currentPathIndex<<endl;
            }
            currentPathIndex--;
        }
    }
}

llu RobotController::getNearestPointIndexForPathSmoothing(bool forward)
{
    // get robot current location
    vector<double> currLocation = positionController->getRobotCoordinates();

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
            if (distance <= PATH_SMOOTHING_RADIUS && isDestinationReachable(path[i]))
                return i;
        }
    }

    return __LONG_LONG_MAX__;
}

bool RobotController::isDestinationReachable(const vector<double> dest)
{
    // motorController->turnTowardDestination(dest);

    double nearestObjDistance = min(sensorController->getDistanceSensorReading(3), sensorController->getDistanceSensorReading(4));
    nearestObjDistance = nearestObjDistance < 0 ? 99 : nearestObjDistance;

    double destinationDistance = getDistance(positionController->getRobotCoordinates(), dest);
    // cout << "destinationDistance " << destinationDistance << endl;
    // cout << "nearestObjDistance " << nearestObjDistance << endl;

    return nearestObjDistance > destinationDistance + 0.2;
}

bool RobotController::gotoDestination()
{
    const vector<double> dest = {DESTINATION_X, DESTINATION_Y};

    double d = getDistance(positionController->getRobotCoordinates(), dest);

    if (d < 2)
    {
        bool d = isDestinationReachable(dest);
        if (d)
        {
            moveToDestination(dest);
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

string RobotController::getRobotId()
{
    return robotId;
}

string *RobotController::getRobotModeReference()
{
    return &mode;
}

void RobotController::updatePath(vector<vector<double>> &receivedPath)
{
    // if robot's mode is explored, then just update the path and get nearest index
    if (mode == "explore")
    {
        path = receivedPath;
        currentPathIndex = getNearestPathIndex(path);
        cout << "receivedPath path 1" << receivedPath.size() << " " << receivedPath[receivedPath.size() - 1][0] << " " << receivedPath[receivedPath.size() - 1][1] << endl;
        cout << "path path 1" << path.size() << " " << path[path.size() - 1][0] << " " << path[path.size() - 1][1] << endl;
        cout << "currentPathIndex path 1" << currentPathIndex << endl;

        updateMode("transport");
    }
    else
    {
        int receivedPathNearestIndex = getNearestPathIndex(receivedPath);
        int currentPathIndex = getNearestPathIndex(path);

        // calculate the distance of first half and second half
        vector<vector<double>> v(path.begin(), path.begin() + currentPathIndex + 1);
        double robotPathFirstHalfDistance = getPathLength(v);

        v = {path.begin() + currentPathIndex, path.end()};
        double robotPathSecondHalfDistance = getPathLength(v);

        v = {receivedPath.begin(), receivedPath.begin() + receivedPathNearestIndex + 1};
        double receivedPathFirstHalfDistance = getPathLength(v);

        v = {receivedPath.begin() + receivedPathNearestIndex, receivedPath.end()};
        double receivedPathSecondHalfDistance = getPathLength(v);

        vector<vector<double>> updatedPath;

        if (robotPathFirstHalfDistance < receivedPathFirstHalfDistance)
            updatedPath.insert(updatedPath.end(), path.begin(), path.begin() + currentPathIndex + 1);
        else
            updatedPath.insert(updatedPath.end(), receivedPath.begin(), receivedPath.begin() + receivedPathNearestIndex + 1);

        if (robotPathSecondHalfDistance < receivedPathSecondHalfDistance)
        {
            updatedPath.insert(updatedPath.end(), path.begin() + currentPathIndex + 1, path.end());
        }
        else
        {
            updatedPath.insert(updatedPath.end(), receivedPath.begin() + receivedPathNearestIndex + 1, receivedPath.end());
        }

        // update path
        path = updatedPath;

        // update index
        currentPathIndex = getNearestPathIndex(updatedPath);
    }
}

int RobotController::getNearestPathIndex(vector<vector<double>> &path)
{
    vector<double> currLocation = positionController->getRobotCoordinates();

    double minDistance = numeric_limits<double>::max();
    double minIndex = -1;

    for (int i = 0; i < path.size(); i++)
    {
        double distance = getDistance(currLocation, path[i]);
        if (distance < minDistance)
        {
            minDistance = distance;
            minIndex = i;
        }
    }

    return minIndex;
}

void RobotController::turnTowardDestination(const vector<double> destinationCoordinate)
{

    int timeStep = getRobotTimestep(robot);

    vector<double> currLocation = positionController->getRobotCoordinates();
    double xDiff = destinationCoordinate[0] - currLocation[0];
    double yDiff = destinationCoordinate[1] - currLocation[1];

    double initAngle = positionController->getCompassReadingInDegrees();

    double turn = (atan2(yDiff, xDiff) * 180 / PI);
    turn = (turn - initAngle);

    while (turn >= 360)
    {
        turn -= 360;
    }
    if (turn < -180)
        turn += 360;
    else if (turn > 180)
        turn -= 360;
    if (fabs(turn) <= 1)
        return;

    // stop robot
    motorController->setSpeed(0, 0);
    double finalAngle = (initAngle + turn + 360);

    while (finalAngle >= 360)
    {
        finalAngle -= 360;
    }

    if (turn > 0)
    {
        // Turn left
        // motorController->motorRotateLeft(1);
        motorController->getLeftMotorReference()->setVelocity(-1);
        motorController->getRightMotorReference()->setVelocity(1);
    }
    else
    {
        // Turn right
        // motorController->motorRotateRight(1);
        motorController->getLeftMotorReference()->setVelocity(1);
        motorController->getRightMotorReference()->setVelocity(-1);
    }

    while (robot->step(timeStep) != -1)
    {
        // middleware();
        double degrees = fModulo(positionController->getCompassReadingInDegrees() + 360, 360);
        if (cartesianIsAngleEqual(degrees, finalAngle))
            break;
        if (turn > 0)
        {
            if (finalAngle == 0)
            {
                if ((degrees < 180) && (degrees > finalAngle))
                    break;
            }
            else
            {
                if (finalAngle <= 180)
                {
                    if (degrees <= 180)
                    {
                        if (degrees >= finalAngle)
                            break;
                    }
                    else
                    {
                        if ((degrees >= finalAngle) && (degrees <= 181))
                            break;
                    }
                }
                else
                {
                    if (degrees <= 180)
                    {
                        if (degrees >= finalAngle)
                            break;
                    }
                    else
                    {
                        if ((degrees >= finalAngle) || (degrees == 0))
                            break;
                    }
                }
            }
        }
        else
        {
            if (finalAngle == 0)
            {
                if ((degrees > 180) || (degrees == 0))
                    break;
            }
            else
            {
                if (finalAngle <= 180)
                {
                    if (degrees <= finalAngle)
                        break;
                }
                else
                {
                    if (degrees >= 180)
                    {
                        if (degrees <= finalAngle)
                            break;
                    }
                    else
                    {
                        if ((degrees <= finalAngle) && (degrees >= 180))
                            break;
                    }
                }
            }
        }
    }

    // motorController->stopMotor();
    motorController->getLeftMotorReference()->setVelocity(0);
    motorController->getRightMotorReference()->setVelocity(0);
}

int RobotController::middleware()
{
    // broadcast wait message
    communication->broadcastWaitMessage();
    int status = 0;
    if (communication->isPartnerNear())
    {
        // stor previous state
        vector<int> currMotorState = motorController->getCurrentState();

        // stop robot
        // motorController->stopMotor();
        motorController->setSpeed(0, 0, 20);

        // cout << robot->getName() << " motor stopped" << endl;
        if (communication->establishConnection())
        {
            cout << robot->getName() << " connection establish successful " << endl;

            if (mode != "explore")
                communication->broadcastPath(path);

            // cout << "1000" << endl;
            vector<vector<double>> receivedPath;
            if (communication->receivePath(receivedPath))
            {
                cout << "partner path outt" << receivedPath.size() << " " << receivedPath[receivedPath.size() - 1][0] << " " << receivedPath[receivedPath.size() - 1][1] << endl;

                // cout << "20000" << endl;
                updatePath(receivedPath);

                status = 1;
            }
        }
        else
        {
            cout << robot->getName() << " connection establish not successful" << endl;
        }

        motorController->setCurrentSet(currMotorState);
    }
    return status;
}

void RobotController::moveToDestination(const vector<double> destinationCoordinate)
{
    // cout<<"here"<<endl;
    // double STEERING_ADJUSTMENT = 0.1;
    // double speed = 4.0;

    // int timeStep = getRobotTimestep(robot);
    // vector<double> currLocation = positionController->getRobotCoordinates();

    // double heading = positionController->getCompassRef()->getValues()[1];

    // double dx = currLocation[0] - destinationCoordinate[0];
    // double dy = currLocation[1] - destinationCoordinate[1];
    // double distance = sqrt(dx * dx + dy * dy);
    // double angle = atan2(dy, dx);
    // double angleDiff = angle - heading;

    // // Adjust steering to minimize angle difference
    // if (angleDiff > PI)
    // {
    //     angleDiff -= 2 * PI;
    // }
    // else if (angleDiff < -PI)
    // {
    //     angleDiff += 2 * PI;
    // }

    // cout<<"angleDiff "<<angleDiff<<endl;
    // double steering = angleDiff * STEERING_ADJUSTMENT;

    // if (distance < 0.5)
    // {
    //     //   currentWaypoint = (currentWaypoint + 1) % numWaypoints;
    //     //   speed = CRUISING_SPEED;
    //     cout << "reached" << endl;
    //     motorController->setSpeed(0, 0, 20);
    // }
    // else
    // {
    //     speed = 6.28 / (distance / 0.5);
    //     speed = max(speed, 0.0);
    //     speed = min(speed, 4.0);
    // }

    // // Set wheel speeds based on steering and speed
    // double leftSpeed = speed - 0.5 * steering * MAX_SPEED;
    // double rightSpeed = speed + 0.5 * steering * MAX_SPEED;
    // cout<<leftSpeed<<" "<<rightSpeed<<endl;
    // motorController->setSpeed(leftSpeed, rightSpeed, 20);

    int timeStep = getRobotTimestep(robot);
    vector<double> currLocation = positionController->getRobotCoordinates();
    int count = 0;

    while (robot->step(timeStep) != -1)
    {
        // cout << robot->getName() << " " << sensorController->getDistanceSensorReading(3) << " " << sensorController->getDistanceSensorReading(4) << " " << communication->getQueueLength() << endl;
        if ((sensorController->getDistanceSensorReading(3) < 1 || sensorController->getDistanceSensorReading(4) < 1) && communication->getQueueLength())
        {
            cout << "there is a robot near" << endl;

            motorController->setSpeed(-2, -2);
            // // wait for 5 seconds
            double currTime = robot->getTime();
            while (robot->step(timeStep) != -1)
            {
                if (robot->getTime() - currTime >= 4)
                {
                    break;
                }
            }
            currTime = robot->getTime();
            motorController->setSpeed(2, -2);
            while (robot->step(timeStep) != -1)
            {
                if (robot->getTime() - currTime >= 2)
                {
                    break;
                }
            }
            motorController->motorMoveForward(10);
            continue;
        }

        if (middleware())
            return;

        vector<double> currLocation = positionController->getRobotCoordinates();

        if ((count++ % 10) == 0)
        {
            turnTowardDestination(destinationCoordinate);
        }

        if (getDistance(currLocation, destinationCoordinate) < 0.5 || (sensorController->getDistanceSensorReading(3) < 1 || sensorController->getDistanceSensorReading(4) < 1))
        {
            // motorController->motorMoveForward(3);
            motorController->setSpeed(3, 3);
        }
        else
        {
            motorController->motorMoveForward(10);
        }

        if (cartesianIsCoordinateEqual(currLocation, destinationCoordinate))
        {
            // motorController->stopMotor();
            motorController->getLeftMotorReference()->setVelocity(0);
            motorController->getRightMotorReference()->setVelocity(0);
            return;
        }
    }
}

void RobotController::displayEstimate()
{
    // Display the actual position
    vector<double> values = positionController->getRobotCoordinates();
    // vector<double> values = {0,0};
    // values[0] = uniform(0,200);
    // values[1] = uniform(0,200);

    cout << "=== " << values[0] << " " << values[1] << endl;
    trackerApp->addActualLocation((int)((values[0] + 48) * 2.5), (int)((values[1] + 20) * 2.5)); // Need to negate the Y value

    // System.out.printf("Actual(x, y) = (%2.1f, %2.1f)\n", (values[0]), -(values[2]));
    //  Display the estimated position
    //  trackerApp->addEstimatedLocation((int)(values[0]), (int)(values[1]));
    //  cout<<"3"<<endl;
}