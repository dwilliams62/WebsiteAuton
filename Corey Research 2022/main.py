import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

#the class for the plot stuff
class myPlot():
    def __init__(self):
        self.image = plt.imread(r"C:\Users\cphil\Documents\GitHub\WebsiteAuton\Corey Research 2022\robot.png")

    def drawPlot(self, x, y, theta):
        imgX = 3
        imgY = 3
        if(np.degrees(theta[-1]) < 0):
            plt.imshow(
                (ndimage.rotate(self.image, (270+np.degrees(theta[-1]))) * 255)
                .astype(np.uint8), extent=[
                    x[-1]-1*imgX,
                    x[-1]+1*imgX,
                    y[-1]-0.75*imgY,
                    y[-1]+0.75*imgY], zorder=2)


        else:
            plt.imshow(
                (ndimage.rotate(self.image, (-90+np.degrees(theta[-1]))) * 255)
                .astype(np.uint8), extent=[
                    x[-1]-.5*imgX,
                    x[-1]+.1*imgX,
                    y[-1]-0.75*imgY,
                    y[-1]+0.75*imgY], zorder=2)

        plt.grid(True)
        plt.xlim(-50, 50)
        plt.ylim(-50, 50)

        plt.plot(x, y, "purple", label="trajectory", zorder=1)
        plt.show()

#defines a class for setting and describing the target point
class targetPoint():
    def __init__(self, x_, y_, theta_):
        if not (x_ is None or y_ is None or theta_ is None):
            self.x = x_
            self.y = y_
            self.theta = theta_
        else:
            self.x = 0
            self.y = 0
            self.theta = 0

    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.theta)

#does most of the computational work and all the math
class Controller():
    #initializing variables
    def __init__(
            self, start_, goal_, R_=0.0325, L_=0.1,
            kP=1.0, kI=0.01, kD=0.01, dT=0.1, v=1.0,
            arrive_distance=1):

        self.current = start_ #start point
        self.goal = goal_ # goal point
        self.R = R_  # in meter
        self.L = L_  # in meter

        self.E = 0   # Cummulative error
        self.old_e = 0  # Previous error

        self.Kp = kP
        self.Ki = kI
        self.Kd = kD

        self.desiredV = v
        self.dt = dT  # in second
        self.arrive_distance = arrive_distance
        return

    #not used at all
    def uniToDiff(self, v, w):
        vR = (2*v + w*self.L)/(2*self.R)
        vL = (2*v - w*self.L)/(2*self.R)
        return vR, vL

    #not used at all
    def diffToUni(self, vR, vL):
        v = self.R/2*(vR+vL)
        w = self.R/self.L*(vR-vL)
        return v, w

    #does one loop of the pid
    def iteratePID(self):
        # Difference in x and y
        d_x = self.goal.x - self.current.x
        d_y = self.goal.y - self.current.y

        # Angle from robot to goal
        g_theta = np.arctan2(d_y, d_x)

        # Error between the goal angle and robot angle
        alpha = g_theta - self.current.theta
        # alpha = g_theta - math.radians(90)
        e = np.arctan2(np.sin(alpha), np.cos(alpha))

        #proportional = new error
        e_P = e
        #integral = error + new error
        e_I = self.E + e
        #derivative = new error - old error
        e_D = e - self.old_e

        # This PID controller only calculates the angular
        # velocity with constant speed of v
        # The value of v can be specified by giving in parameter or
        # using the pre-defined value defined above.
        w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D

        w = np.arctan2(np.sin(w), np.cos(w))

        #adds error to culmative error
        self.E = self.E + e
        #updates previous errorhhhh
        self.old_e = e
        #returns angular velocity?
        v = self.desiredV

        return v, w

    #not sure
    def fixAngle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    #akin to motor.spin, actually changes the position of the robot using the calculations from iteratePID
    def makeAction(self, v, w):
        x_dt = v*np.cos(self.current.theta)
        y_dt = v*np.sin(self.current.theta)
        theta_dt = w

        self.current.x = self.current.x + x_dt * self.dt
        self.current.y = self.current.y + y_dt * self.dt
        self.current.theta = self.fixAngle(self.current.theta + self.fixAngle(theta_dt * self.dt))
        return

    #checks if at the goal
    def isArrived(self):
        # print("Arrive check:", str(abs(self.current.x - self.goal.x)),
        #       str(abs(self.current.y - self.goal.y)))
        current_state = np.array([self.current.x, self.current.y])
        goal_state = np.array([self.goal.x, self.goal.y])
        difference = current_state - goal_state

        distance_err = difference @ difference.T
        if distance_err < self.arrive_distance:
            return True
        else:
            return False

    def runPID(self, myPlot=None):
        #sets the current x, y, and theta arrays to the current location
        x = [self.current.x]
        y = [self.current.y]
        theta = [self.current.theta]

        #continues looping until arriving at the target point
        while(not self.isArrived()):
            #run the pid action
            v, w = self.iteratePID()
            self.makeAction(v, w)
            x.append(self.current.x)
            y.append(self.current.y)
            theta.append(self.current.theta)
            if myPlot:

                myPlot.drawPlot(x, y, theta)
            # time.sleep(self.dt)

            # Print or plot some things in here
            # Also it can be needed to add some max iteration for
            # error situations and make the code stable.
            # print(self.current.x, self.current.y, self.current.theta)
        return x, y, theta


def trackRoute(start, targets):
    #initalize the current variable to be at the start point
    #x, y, and theta are arrays that hold the various points of the robot for drawing the graph at the end
    current = start
    x = []
    y = []
    theta = []

    #loops through all the target points in the targets array
    for target in targets:
        #initalize the controller object as an object of the controller class with the current location and target location
        controller = Controller(current, target)
        #runs the pid for the current movement
        x_, y_, theta_ = controller.runPID()
        #updates the arrays with all the new x, y, theta values from the pid in this loop
        x.extend(x_)
        y.extend(y_)
        theta.extend(theta_)
        #updates current to have the new current location
        current = controller.current
    return x, y, theta


# Starting point of the code
def main():
    # Define dt time, create controller, define start and goal points
    # In every iteration, get an action from PID and make the action,
    # after that, sleep for dt. Repeat that loop until reaching the goal state.

    #define the starting point, np.radians = 90 degrees converted to radians
    start = targetPoint(0, 0, np.radians(90))

    #initalize the targets array with points for the map to go to
    targets = [targetPoint(0,40,0), targetPoint(40,20,0), targetPoint(-10,-20,0), targetPoint(-20,15, 0), targetPoint(0,3,0)]
    #starts the loop for the robot to travel to each point
    x, y, theta = trackRoute(start, targets)

    #define the object lines as a myPlot object
    lines = myPlot()
    #draws the plot
    lines.drawPlot(x, y, theta)


main()