<h1>arcade_control package<h1>

<h2>`ArcadeDriver` component</h2>

Subscribes to joystick's Twist messages and computes two values (`l` and `r`) for left and right side motor speeds.

`l` and `r` are published to the `cmd_vel` topic.

<h2>`MotorSpeedController component</h2>

Subscribes to `cmd_vel` topic and computes 6 motor speeds (3 left, 3 right) using `l` and `r` as baseline.

The 6 motor speeds are published to the `cmd_vel_out` topic.

