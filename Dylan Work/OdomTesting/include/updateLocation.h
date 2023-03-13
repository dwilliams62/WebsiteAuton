#include "robot-config.h"
#include <math.h>

//convert degrees to inches
double ConvertDegreesToInches(double setDegrees, double turnDiameter = 12.17) {
  double requiredInches = setDegrees / 360.0 * M_PI * turnDiameter;
  return requiredInches;
}

//convert the inches to revolutions
double ConvertInchesToRevolutions(double requiredInches) {
  double circumferenceOfWheel = 3.86 * M_PI;
  double outputRat = 3.0/5.0;
  double requiredRevolutions = (requiredInches / circumferenceOfWheel) * outputRat * 360.0;
  return requiredRevolutions;
}

//convert radians to degrees
double ConvertRadiansToDegrees(double radian) {
  radian = radian * (180.0 / M_PI);
  return radian;
}

//reset all the encoders
void ResetEncoders() {
  lMotor1.setPosition(0,degrees); lMotor2.setPosition(0,degrees);
  lMotor3.setPosition(0,degrees); lMotor4.setPosition(0,degrees);
  rMotor1.setPosition(0,degrees); rMotor2.setPosition(0,degrees);
  rMotor3.setPosition(0,degrees); rMotor4.setPosition(0,degrees);
}

//variables for the odometry
double xSelf = 0, ySelf = 0, tSelf = 0, xCurrent = 0, yCurrent = 0, tCurrent = 0;
double goalAngle, requiredAngle, requiredDistance, displayCount = 0;
double prevLeftEncoder = 0, prevRightEncoder = 0, currLeftEncoder, currRightEncoder, changeLeftEncoder, changeRightEncoder;
double changeAngle, changeX, changeY, tempHyp, currAngle, prevAngle, averageAngle, tempAngle;
double localOffsetX = 0, localOffsetY = 0, changeLeftReset, changeRightReset;
double globalOffsetX = 0, globalOffsetY = 0, length = 12.09;

void DisplayLocation(double x, double y, double t) {
    if (displayCount == 100) {
    Brain.Screen.print(x);
    Brain.Screen.print(" , ");
    Brain.Screen.print(y);
    Brain.Screen.print(" , ");
    Brain.Screen.print(ConvertRadiansToDegrees(t));
    Brain.Screen.newLine();
    displayCount = 0;
  }
  else if (displayCount == 95) {
    Brain.Screen.clearScreen();
    displayCount++;
  }
  else {
    displayCount++;
  }
}

void UpdateLocationRandom(double t = 1) {
  currLeftEncoder = (lMotor1.position(degrees) + lMotor2.position(degrees) + 
    lMotor3.position(degrees) + lMotor4.position(degrees)) / 4;
  currRightEncoder = (rMotor1.position(degrees) + rMotor2.position(degrees) + 
    rMotor3.position(degrees) + rMotor4.position(degrees)) / 4;

  changeLeftEncoder = ConvertDegreesToInches(currLeftEncoder - prevLeftEncoder, 7.975);
  changeRightEncoder = ConvertDegreesToInches(currRightEncoder - prevRightEncoder, 7.975);

  ySelf += ((changeLeftEncoder + changeRightEncoder) / 2);
  prevLeftEncoder = currLeftEncoder;
  prevRightEncoder = currRightEncoder;

  if (changeLeftEncoder != 0 || changeRightEncoder != 0) {
    currAngle = prevAngle + ((changeLeftEncoder - changeRightEncoder) / 12.09);

    changeAngle = currAngle - prevAngle;

    if (fabs(changeAngle) < 0.03) {
      changeX = (currLeftEncoder + currRightEncoder / 2) * (cos(changeAngle));
      changeY = (currLeftEncoder + currRightEncoder / 2) * (sin(changeAngle));
    }
    else {
      changeX = 2 * sin(changeAngle / 2);
      changeY = (2 * sin(changeAngle / 2)) * ((changeRightEncoder / changeAngle) + 6.08);
    }

    xSelf += changeX;
    ySelf += changeY;
    tSelf = currAngle;
    prevAngle = currAngle;
  }

  prevLeftEncoder = currLeftEncoder;
  prevRightEncoder = currRightEncoder;
  DisplayLocation(xSelf,ySelf,tSelf);
}

void UpdateLocationTracking(double t = 1) {
  // tracking.pdf
  //step 1
  currLeftEncoder = (lMotor1.position(degrees) + lMotor2.position(degrees) + 
    lMotor3.position(degrees) + lMotor4.position(degrees)) / 4;
  currRightEncoder = (rMotor1.position(degrees) + rMotor2.position(degrees) + 
    rMotor3.position(degrees) + rMotor4.position(degrees)) / 4;

  currLeftEncoder = ConvertDegreesToInches(currLeftEncoder,7.975);
  currRightEncoder = ConvertDegreesToInches(currRightEncoder,7.975);

  //step 2
  changeLeftEncoder = currLeftEncoder - prevLeftEncoder;
  changeRightEncoder = currRightEncoder - prevRightEncoder;

  //step 3
  prevLeftEncoder = currLeftEncoder;
  prevRightEncoder = currRightEncoder;

  //step 4
  changeLeftReset += changeLeftEncoder;
  changeRightReset += changeRightEncoder;

  //step 5
  currAngle = prevAngle + ((changeLeftReset - changeRightReset) / 12.09);

  //step 6
  changeAngle = currAngle - prevAngle;

  //step 7
  if (changeAngle == 0) {
    localOffsetX = 0;
    localOffsetY = changeRightReset;
  }
  //step 8
  else {
    localOffsetX = 0;
    localOffsetY = (2 * sin(changeAngle / 2)) * ((changeRightReset / changeAngle) + 6.045);
  }

  //step 9
  averageAngle = prevAngle + (changeAngle / 2);

  //step 10 - still confused about the "change the angle" part, could be causing issues
  tempHyp = sqrt((localOffsetX * localOffsetX) + (localOffsetY * localOffsetY));
  globalOffsetX = tempHyp * cos(averageAngle);
  globalOffsetY = tempHyp * sin(averageAngle);

  //step 11
  xSelf += globalOffsetX;
  ySelf += globalOffsetY;
  tSelf = currAngle;
  prevAngle = currAngle;

  DisplayLocation(xSelf,ySelf,tSelf);
}

void UpdateLocationMob(double t = 1) {
  currLeftEncoder = (lMotor1.position(degrees) + lMotor2.position(degrees) + 
    lMotor3.position(degrees) + lMotor4.position(degrees)) / 4;
  currRightEncoder = (rMotor1.position(degrees) + rMotor2.position(degrees) + 
    rMotor3.position(degrees) + rMotor4.position(degrees)) / 4;
  
  currLeftEncoder = ConvertDegreesToInches(currLeftEncoder,7.975);
  currRightEncoder = ConvertDegreesToInches(currRightEncoder,7.975);

  changeLeftEncoder = currLeftEncoder - prevLeftEncoder;
  changeRightEncoder = currRightEncoder - prevRightEncoder;

  prevLeftEncoder = currLeftEncoder;
  prevRightEncoder = currRightEncoder;

  currAngle = prevAngle + (((changeLeftEncoder + changeRightEncoder) / length) * t);
  changeAngle = currAngle - prevAngle;

  if (changeLeftEncoder == changeRightEncoder) {
    changeX = changeLeftEncoder * cos(currAngle);
    changeY = changeRightEncoder * sin(currAngle);
  }
  else {
    changeX = ((changeRightEncoder + changeLeftEncoder) / (changeRightEncoder - changeLeftEncoder)) * (length / 2) * sin(changeAngle);
    changeY = (((changeRightEncoder + changeLeftEncoder) / (changeRightEncoder - changeLeftEncoder)) * (length / 2) * cos(changeAngle)) + 
      (((changeRightEncoder + changeLeftEncoder) / (changeRightEncoder - changeLeftEncoder)) * (length / 2));
  }

  xSelf += changeX;
  ySelf += changeY;
  tSelf = currAngle;

  DisplayLocation(xSelf,ySelf,tSelf);
}