#include <sensors.h>

SensorsController::SensorsController(Robot *robot)
{
    this->robot = robot;
}

void SensorsController::initSensors(int timeStep)
{
    for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
    {
        this->sensors[i] = this->robot->getDistanceSensor(this->distanceSensorsName[i]);
        this->sensors[i]->enable(timeStep);
    }
}

vector<bool> SensorsController::getSensorsCondition()
{
    vector<bool> sensorsConditions;
    for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
    {
        double val = this->sensors[i]->getValue();
        if (val >= SENSOR_VALUE_DETECTION_THRESHOLD)
            sensorsConditions.push_back(true);
        else
            sensorsConditions.push_back(false);
    }

    return sensorsConditions;
}