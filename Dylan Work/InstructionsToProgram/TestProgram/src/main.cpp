#include "vex.h"
#include "motion.h"
#include "math.h"
using namespace vex;
competition Competition;
void pre_auton(void) {
	vexcodeInit();
}
void autonomous(void) {
  isAuton = true; resetPID = true; resetTurning = true; resetFlywheel = true; isUser = false;
  task StartAuton(autonController);
  //start website provided code
	GoToPoint(10,5);
	RotateBot(45);
	GoToPoint(-5,5);
	RotateBot(-30);
  //end website provided code
  SpinMotors(0);
}
void usercontrol(void) {
  isAuton = false; resetPID = true; resetTurning = true; resetFlywheel = true; isUser = true;
  task StartUser(userController);
}
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {wait(100, msec);}
}
