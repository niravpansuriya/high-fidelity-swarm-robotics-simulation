#include <motor.h>

MotorController::MotorController(Robot *robot)
{
    this->robot = robot;
    this->leftMotor = robot->getMotor("left wheel");
    this->rightMotor = robot->getMotor("right wheel");
    this->leftMotor->setPosition(INFINITE);
    this->rightMotor->setPosition(INFINITE);
    this->leftMotor->setVelocity(0.0);
    this->rightMotor->setVelocity(0.0);

    this->positionController = new PositionController(robot);
}

void MotorController::stopMotor()
{
    this->leftMotor->setVelocity(0.0);
    this->rightMotor->setVelocity(0.0);
}

void MotorController::motorRotateRight(int speed)
{
    this->leftMotor->setVelocity(speed ? speed : MAX_SPEED);
    this->rightMotor->setVelocity(-1 * (speed ? speed : MAX_SPEED));
}

void MotorController::motorMoveForward(int speed)
{
    this->leftMotor->setVelocity(speed ? speed : MAX_SPEED);
    this->rightMotor->setVelocity(speed ? speed : MAX_SPEED);
}

void MotorController::motorRotateLeft(int speed)
{
    this->leftMotor->setVelocity(-1 * (speed ? speed : MAX_SPEED));
    this->rightMotor->setVelocity(speed ? speed : MAX_SPEED);
}

void MotorController::motorRotateLeftInDegree(double degrees)
{
    int timeStep = (int)robot->getBasicTimeStep();

    this->motorRotateLeft(1.2);
    double initAngle = positionController->getCompassReadingInDegrees();
    if (initAngle < 0)
        initAngle += 360;

    double desiredAngle = fModulo((initAngle + degrees), 360);

    double currentAngle = positionController->getCompassReadingInDegrees();
    if (currentAngle < 0)
        currentAngle += 360;

    do
    {
        this->robot->step(timeStep);
        currentAngle = positionController->getCompassReadingInDegrees();
        if (currentAngle < 0)
            currentAngle += 360;
    } while (!cartesianIsAngleEqual(currentAngle, desiredAngle));

    this->stopMotor();
}

void MotorController::moveToDestination(const double destinationCoordinate[2])
{
    int timeStep = getRobotTimestep(robot);
    while (robot->step(timeStep) != -1)
    {
        double *currLocation = positionController->getRobotCoordinates();

        turnTowardDestination(destinationCoordinate);

        motorMoveForward(6);
        if (cartesianIsCoordinateEqual(currLocation, destinationCoordinate))
        {
            stopMotor();
            return;
        }
    }
}

void MotorController::turnTowardDestination(const double destinationCoordinate[2])
{
    int timeStep = getRobotTimestep(robot);

    double *currLocation = positionController->getRobotCoordinates();
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

    double finalAngle = (initAngle + turn + 360);

    while (finalAngle >= 360)
    {
        finalAngle -= 360;
    }

    if (turn > 0)
    {
        // Turn left
        motorRotateLeft(1);
    }
    else
    {
        // Turn right
        motorRotateRight(1);
    }

    while (robot->step(timeStep) != -1)
    {
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

    stopMotor();
}
