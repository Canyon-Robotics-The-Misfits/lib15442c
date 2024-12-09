Team 15442c's library for programming vex robots

This repo contains the following:
* Drivetrain control code
* Odometry (using tracking wheels, the GPS sensor, or monte carlo localization)
* Trajectory/2D Motion Profile generation using Cubic Hermite Splines
* Motion Algorithms, like simple PIDs, Boomerang, RAMSETE, and more
* Math utilities, like for managing angles, vectors, and poses
* Wrapper classes for motors and pneumatics
* Logger macros

The project is half setup to be built as a PC executable for unit testing, but it doesn't work yet because I haven't had time to get it working and it's not that important anyways. If I have extra time I may finish making that