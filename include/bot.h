#ifndef BOT_H
#define BOT_H

#include <cmath>
#include "api.h"
#include "ports.h"

using namespace pros;

class Bot
{
private:
    const double kP = 0.12;
    const double kD = 0.075;
    const double kP_angle = 1.75;
    const double kD_angle = 1;

    Motor leftMotor1 = Motor(LM1, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
    Motor leftMotor2 = Motor(LM2, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
    Motor rightMotor1 = Motor(RM1, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
    Motor rightMotor2 = Motor(RM2, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);

    Gps gps = Gps(GPS);

    c::gps_status_s_t status;

    double leftMotor1Speed;
    double leftMotor2Speed;
    double rightMotor1Speed;
    double rightMotor2Speed;

    double orthogonal1;
    double orthogonal2;
    double orthogonal1Speed;
    double orthogonal2Speed;

    double angleError;
    double currentHeading;
    double sine;
    double cosine;

    double lastError1;
    double lastError2;
    double proportional1;
    double derivative1;
    double proportional2;
    double derivative2;

    double lastAngleError;
    double proportionalAngle;
    double derivativeAngle;
    double turnSpeed;

    double Abs(double k);
    double getSine(double angle);
    double getCosine(double angle);
    void AdjustHeading(double x, double y, double degree);
    void Spin();

public:
    void Move(double x, double y, double angle, double lengthTolerance, double angleTolerance, double tickLength);
    void Turn(double angle);
    void SetHeading(double angle);
};

#endif