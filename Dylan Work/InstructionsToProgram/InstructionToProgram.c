#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

//prints an error message and exits the code
void err_sys(char* str) {
	printf("%s\n", str);
	exit(1);
}

int main(int argc, char **argv)
{
	//initalize frankly too many variables if we being honest
	int inputFile, outputFile, count = 0, column = 0, ascii = 0, xSpotInt, ySpotInt, tSpotInt;
	int currentInstructSpot = 0, currentXSpot = 0, currentYSpot = 0, currentTSpot = 0;
	char c;
	char instruction[32];
	char xSpot[8];
	char ySpot[8];
	char tSpot[8];
	
	//open the csv file with the instructions, and create a new main in the given VEXcode project
	if ((inputFile = open("testCSV.csv", O_RDONLY, 0777)) == -1) {
		err_sys("Error Opening Input File!");
	}
	if ((outputFile = open("TestProgram/src/main.cpp", O_WRONLY|O_CREAT|O_TRUNC, 0777)) == -1) {
		err_sys("Error Opening Output File!");
	}
	
	//change the standard output from the console to the created text file
	dup2(outputFile, 1);
	
	//initialize the file with all of the lines of code that appear before the autonomous routine
	printf("%s\n", "#include \"vex.h\"");
	printf("%s\n", "#include \"motion.h\"");
	printf("%s\n", "#include \"math.h\"");
	printf("%s\n", "using namespace vex;");
	printf("%s\n", "competition Competition;");
	printf("%s\n", "void pre_auton(void) {");
	printf("%s\n", "	vexcodeInit();");
	printf("%s\n", "}");
	printf("%s\n", "void autonomous(void) {");
	printf("%s\n", "  isAuton = true; resetPID = true; resetTurning = true; resetFlywheel = true; isUser = false;");
	printf("%s\n", "  task StartAuton(autonController);");
	printf("%s\n", "  //start website provided code");
	
	//goes through the input file bit by bit, doing various operations depending on the bit
	while ((count = read(inputFile, &c, 1)) > 0) {
		//converts the character bit to it's ascii value
		ascii = c;
		
		//if the character is a new line, it has finished reading the instructions for what line of code to put in, and acts upon this information
		if (ascii == 10) { 
			//cuts off the strings where the data is not relevant anymore so no extra data from previous cycles messes it up
			instruction[currentInstructSpot] = '\0';
			xSpot[currentXSpot] = '\0';
			ySpot[currentYSpot] = '\0';
			tSpot[currentTSpot] = '\0';
			
			//resets the column back to 0 and converts data to usable forms
			column = 0;
			xSpotInt = atoi(xSpot); 
			ySpotInt = atoi(ySpot);
			tSpotInt = atoi(tSpot);
			
			//if the string in the instructions says "Move", it adds a GoToPoint function to the main.cpp
			if ((strcmp(instruction, "Move")) == 0) {
				printf("	GoToPoint(%d,%d);\n", xSpotInt, ySpotInt);
			}
			
			//if the string in the instructions says "Rotate", it adds a RotateBot function to the main.cpp
			if ((strcmp(instruction, "Rotate")) == 0) {
				printf("	RotateBot(%d);\n", tSpotInt);
			}
			
			//resets all of the positions in the various strings
			currentInstructSpot = 0;
			currentXSpot = 0;
			currentYSpot = 0;
			currentTSpot = 0;
		}
		//if the ascii is a comma, increments the column variable
		else if (ascii == 44) {
			column++;
		}
		//for anything else it checks what column it is in and adds it to the corresponding string
		else {
			if (column == 0) {
				instruction[currentInstructSpot] = c;
				currentInstructSpot++;
			}
			if (column == 1) {
				xSpot[currentXSpot] = c;
				currentXSpot++;
			}
			if (column == 2) {
				ySpot[currentYSpot] = c;
				currentYSpot++;
			}
			if (column == 3) {
				tSpot[currentTSpot] = c;
				currentTSpot++;
			}
		}	
	}
	if (count == -1) {err_sys("Error Reading Input!");}
	
	//adds the end of the code that will always be there, includes functionality for drive mode if so desired
	printf("%s\n", "  //end website provided code");
	printf("%s\n", "}");
	printf("%s\n", "void usercontrol(void) {");
	printf("%s\n", "  isAuton = false; resetPID = true; resetTurning = true; resetFlywheel = true; isUser = true;");
	printf("%s\n", "  task StartUser(userController);");
	printf("%s\n", "}");
	printf("%s\n", "int main() {");
	printf("%s\n", "  Competition.autonomous(autonomous);");
	printf("%s\n", "  Competition.drivercontrol(usercontrol);");
	printf("%s\n", "  pre_auton();");
	printf("%s\n", "  while (true) {wait(100, msec);}");
	printf("%s\n", "}");
	
	close(inputFile);
	close(outputFile);
	return 0;
}

