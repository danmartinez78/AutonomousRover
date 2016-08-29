#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys
import time

from RosInterface import ROSInterface

# User files, uncomment as completed
from ShortestPath import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        self.pos_goal = pos_goal
        self.pos_init = pos_init
        self.state = np.array([0,0,0])
        self.x = pos_init[0]
        self.y = pos_init[1]
        self.theta = pos_init[2]
        self.timeout = 0
        self.timed_out = False
        self.cmd = np.array([0,0,0])
        self.state = pos_init
        self.path = []
        self.index = 1

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        # Uncomment as completed
        self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        # INITIAL POSE ESTIMATION
        meas = None
        i = 0
        while (i<10):
            #print("Trying to localize...")
            while (meas!=None):
                self.state = self.kalman_filter.step_filter(None, None, meas)
                #print self.state
                i += 1
                meas = None
            meas = self.ros_interface.get_measurements()
            
        print self.state
        self.pos_init = np.array([self.state[0], self.state[1]])
        self.path = dijkstras(occupancy_map,x_spacing,y_spacing,self.pos_init,pos_goal)
        print self.path
        print len(self.path)

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        This function is called at 60Hz
        """
        meas = self.ros_interface.get_measurements()
        self.state = self.kalman_filter.step_filter(None, None, meas)
        self.drivePath()
        #print self.state
        #self.drivePath()
        
        return

    def drivePath(self):
        goal = self.path[self.index]
        self.cmd = self.diff_drive_controller.compute_vel(self.state, goal)
        if self.cmd[2] == 0:
            self.ros_interface.command_velocity(self.cmd[0], self.cmd[1])
            imu_meas = self.ros_interface.get_imu()
            meas = self.ros_interface.get_measurements()
            self.state = self.kalman_filter.step_filter(self.cmd[0], imu_meas, meas)
            print self.cmd
            #print self.state
        else:		
            self.index += 1
	    print self.index
            print self.path[self.index]
        return
    
def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


