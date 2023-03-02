# VEX Pro V5 X-Drivetrain Drive and Autonomous Code

Coded in [VEX PROS](https://pros.cs.purdue.edu/v5/api/cpp/index.html).

@ericssonl07 [Github](https://github.com/ericssonl07)

@coon-hound [Github](https://github.com/coon-hound)

## Features
As the robot has an X-Drivetrain, it is very manoeuvrable and can strafe in any direction without turning.
The Drivetrain autonomous Move function uses the current GPS coordinates and yaw to calculate each motor's
power individually with trigonometric functions, then feeds it into a PID controller for efficient motion.
In control mode, the button Y will toggle precision mode, which divides each motor's power by five. Precision
mode enables drivers to accurately turn and move the robot if needed.

## Controller Binds
```
Axis 1      Turn
Axis 3      Forward/backward
Axis 4      Left/right

Button Y    Toggle precision mode
```
![](https://kb.vex.com/hc/article_attachments/360042385051/5dc33ee3d12f1.jpeg)

<sup>The VEX Controller Axis arrangement</sup>

## Autonomous Drive Functions
Declared in src/bot.hpp
1. `void Move(double x, double y, double angle, double lengthTolerance, double angleTolerance, double tickLength)` moves the robot to coordinates `(x, y)` in meters at a final angle of `angle` degrees with an acceptable error of `lengthTolerance` meters and `angleTolerance` degrees, readjusting heading every `tickLength` milliseconds.

2. `void Turn(double angle)` turns the robot `angle` degrees.

3. `void SetHeading(double angle)` turns the robot to `angle` degrees.

## To do
- [ ] Implement Integration for PID Controller- "I" component
- [ ] Tune PID Controller constants with [Ziegler-Nichols method](https://en.wikipedia.org/wiki/Ziegler-Nichols_method)
- [ ] Template callback for autonomous functions during control, ex. on left button execute predefined move sequence
