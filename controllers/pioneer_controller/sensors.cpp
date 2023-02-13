#include <sensors.h>

SensorsController::SensorsController(Robot *robot)
{
    this->robot = robot;
}

void SensorsController::initSensors()
{
    int timeStep = getRobotTimestep(robot);
    for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
    {
        this->distanceSensors[i] = this->robot->getDistanceSensor(this->distanceSensorsName[i]);
        this->distanceSensors[i]->enable(timeStep);
    }
}

vector<bool> SensorsController::getSensorsCondition()
{
    vector<bool> sensorsConditions;
    for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
    {
        double val = this->distanceSensors[i]->getValue();
        if (val < TOO_CLOSE_DISTANCE)
            sensorsConditions.push_back(true);
        else
            sensorsConditions.push_back(false);
    }

    return sensorsConditions;
}

bool SensorsController::isSomethingInFront()
{
    vector<bool> distanceSensorsCondition = getSensorsCondition();
    return distanceSensorsCondition[3] || distanceSensorsCondition[4];
}

bool SensorsController::isSomethingInFrontLeft()
{
    vector<bool> distanceSensorsCondition = getSensorsCondition();
    return distanceSensorsCondition[1] || distanceSensorsCondition[3];
}

bool SensorsController::isSomethingInFrontRight()
{
    vector<bool> distanceSensorsCondition = getSensorsCondition();
    return distanceSensorsCondition[4] || distanceSensorsCondition[6];
}