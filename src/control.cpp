#include "control.h"
#include "api.h"
#include "ports.h"
using namespace pros;

int control()
{
    Controller controller = Controller(E_CONTROLLER_MASTER);

    Motor leftMotor1 = Motor(LM1, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
    Motor leftMotor2 = Motor(LM2, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
    Motor rightMotor1 = Motor(RM1, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
    Motor rightMotor2 = Motor(RM2, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);

    int precisionFactor = 1.0;
    int LX, LY, RX;
    int leftMotor1Speed;
    int leftMotor2Speed;
    int rightMotor1Speed;
    int rightMotor2Speed;

    bool precisionEnabled = false;

    while (true)
    {
        LX = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        LY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        RX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y))
            precisionEnabled = not precisionEnabled;

        if (precisionEnabled)
            precisionFactor = 5;
        else
            precisionFactor = 1;

        leftMotor1Speed = (LY + LX + RX) / precisionFactor / 127 * 100;
        leftMotor2Speed = (LY - LX + RX) / precisionFactor / 127 * 100;
        rightMotor1Speed = (LY - LX - RX) / precisionFactor / 127 * 100;
        rightMotor2Speed = (LY + LX - RX) / precisionFactor / 127 * 100;

        leftMotor1.move_velocity(leftMotor1Speed);
        leftMotor2.move_velocity(leftMotor2Speed);
        rightMotor1.move_velocity(rightMotor1Speed);
        rightMotor2.move_velocity(rightMotor2Speed);
    }
    return 0;
}