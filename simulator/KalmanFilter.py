#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = 0 #None # Used to keep track of time between measurements
        self.Q_t = np.eye(3)
        self.R_t = np.eye(3)

        # YOUR CODE HERE
        self.x_t = np.array([0, 0, 0])
        self.x = 0
        self.y = 0
        self.theta = 0
        self.P_t = np.eye(3) * 1000000




        
    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        # YOUR CODE HERE
        self.prediction(v, imu_meas)
        #print(self.x_t)
        if z_t != None and z_t != []:
            self.update(z_t)
        #print(self.x_t)
        pass

    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predction of the state
        predicted_covariance - a 3 by 3 numpy array of the predction of the
            covariance
        """
        # YOUR CODE HERE
        # propogate pose x,y,theta
        #print self.x_t
        #print v
        dt = imu_meas[4] - self.last_time
        self.last_time = imu_meas[4]
        dtheta = imu_meas[3]
        self.theta += dtheta*dt
        self.theta = self.theta%(2*math.pi)
        dx = v * math.cos(self.theta)*dt
        dy = v * math.sin(self.theta)*dt
        #print dx,dy,dtheta
        self.x += dx
        self.y += dy
        self.P_t += np.eye(3)
        self.x_t = np.array([self.x, self.y, self.theta]) # probably should fix this
        #print self.x_t
        # propogate covariance
        self.P_t =+ self.Q_t
        pass

    def update(self, z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        # YOUR CODE HERE
        #print(z_t)
        # iterate through detected tags
        for tag in z_t:
            # for each tag -> find world frame coords from self.markers
            x_r = tag[0]
            y_r = tag[1]
            theta_r = tag[2]
        for ref in self.markers:
                if tag[3] == ref[3]:
                    x_w = ref[0]
                    y_w = ref[1]
                    theta_w = ref[2]
                    # with body frame & world frame coords of tag -> find world frame coords of robot
                    
                    # calc kalman gain
        # combine observation w/ prediction
        # update covariance
        pass