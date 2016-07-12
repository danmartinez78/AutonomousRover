#!/usr/bin/python

import numpy as np
import math

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp=2
        self.ka=6
        self.kb=0
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        self.threshold = 0.6
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        # print(",".join([str(dx), str(dy)]))

        rho = math.sqrt(dx*dx + dy*dy)
        if rho < self.threshold:
            return [0, 0, 1]
        alpha = -state[2] + math.atan2(dy,dx)
        beta = -state[2] - alpha

        vel_cmd = min(self.MAX_SPEED, self.kp*rho)
        omega_cmd = min(self.MAX_OMEGA, self.ka*alpha + self.kb*beta)

        return([vel_cmd, omega_cmd, 0])
        pass
