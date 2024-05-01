#include "main.h"

void opcontrol() {
	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::MotorGroup left_mg({1,-2,3}); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	// pros::MotorGroup right_mg({-4,5,-6}); // Creates a motor group with forwards port 4 and reversed ports 4 & 6

	// while (true) {
	// 	// Arcade control scheme
	// 	int dir = master.get_analog(ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
	// 	int turn = master.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
	// 	left_mg = dir - turn; // Sets left motor voltage
	// 	right_mg = dir + turn; // Sets right motor voltage
	// 	pros::delay(20); // Run for 20 ms then update
	// }
}