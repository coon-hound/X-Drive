#include <chrono>
#include <cmath>
#include "bot.h"
#include "api.h"

using namespace pros;

double Bot::Abs(double k)
{
    if (k > 0) return k;
    return -k;
}

double Bot::getSine(double angle)
{
    return sin(angle / 180 * 3.14159265358979323846);
}

double Bot::getCosine(double angle)
{
    return cos(angle / 180 * 3.14159265358979323846);
}

void Bot::AdjustHeading(double x, double y, double degree)
{
    status = gps.get_status();
    double relativeX = 0 - (x - status.x);
    double relativeY = 0 - (x - status.y);
    angleError = degree - status.yaw;
    currentHeading = status.yaw - 45;

    sine = getSine(currentHeading);
    cosine = getCosine(currentHeading);

    orthogonal1 = (cosine * relativeX) + (sine * relativeY);
    orthogonal2 = (cosine * relativeY) - (sine * relativeX);

    proportional1 = orthogonal1 * kP;
    proportional2 = orthogonal2 * kP;
    derivative1 = (orthogonal1 - lastError1) * kD;
    derivative2 = (orthogonal2 - lastError2) * kD;
    orthogonal1Speed = proportional1 + derivative1;
    orthogonal2Speed = proportional2 + derivative2;

    proportionalAngle = angleError * kP_angle;
    derivativeAngle = (angleError - lastAngleError) * kD_angle;
    turnSpeed = proportionalAngle + derivativeAngle;

    lastAngleError = angleError;
    lastError1 = orthogonal1;
    lastError2 = orthogonal2;

    rightMotor1Speed = 0 - orthogonal1Speed;
    leftMotor2Speed = 0 - orthogonal1Speed;
    leftMotor1Speed = orthogonal2Speed;
    leftMotor2Speed = orthogonal2Speed;

    leftMotor1Speed -= turnSpeed;
    leftMotor2Speed -= turnSpeed;
    rightMotor1Speed += turnSpeed;
    rightMotor2Speed += turnSpeed;
}

void Bot::Spin()
{
    int lm1s = (int) round(leftMotor1Speed);
    int lm2s = (int) round(leftMotor2Speed);
    int rm1s = (int) round(rightMotor1Speed);
    int rm2s = (int) round(rightMotor2Speed);

    leftMotor1.move_velocity(lm1s);
    leftMotor2.move_velocity(lm2s);
    rightMotor1.move_velocity(rm1s);
    rightMotor2.move_velocity(rm2s);
}

void Bot::Move(double x, double y, double angle, double lengthTolerance = 100, double angleTolerance = 10, double tickLength = 0)
{
    status = gps.get_status();
    lastAngleError = angle - status.yaw;
    double initialcos = cos((status.yaw - 45) / 180 * 3.14159265358979323846);
    double initialsin = sin((status.yaw - 45) / 180 * 3.14159265358979323846);
    lastError1 = (initialcos * (y - status.y)) - (initialsin * x - status.x);
    lastError2 = (initialsin * (y - status.y)) + (initialcos * (x - status.x));
    while (Abs(status.x - x) > lengthTolerance ||
           Abs(status.y - y) > lengthTolerance ||
           Abs(status.yaw - angle) > angleTolerance)
    {
        AdjustHeading(x, y, angle);
        Spin();
        delay(tickLength);
        status = gps.get_status();
    }
}

void Bot::Turn(double angle)
{
    status = gps.get_status();
    Move(status.x, status.y, status.yaw + angle, 100.0, 10.0, 20.0);
}

void Bot::SetHeading(double angle)
{
    status = gps.get_status();
    Move(status.x, status.y, angle, 100.0, 10.0, 20.0);
}