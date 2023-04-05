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
	GoToPoint(5,27);
	RotateBot(180);
	GoToPoint(115,115);
	GoToPoint(1,1);
	GoToPoint(20,20);
	GoToPoint(2000,2000);
	RotateBot(2000);
  //end website provided code
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
