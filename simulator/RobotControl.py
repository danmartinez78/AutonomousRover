#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 19 2016.
"""
#import rospy

import yaml
import numpy as np
import time

import sys
import math

# TODO for student: Comment this section when running on the robot 
from RobotSim import RobotSim
import matplotlib.pyplot as plt

# TODO for student: uncomment when changing to the robot
# from ros_interface import ROSInterface

# TODO for student: User files, uncomment as completed
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
        Inputs: (all loaded from the parameter YAML file)
        world_map - a P by 4 numpy array specifying the location, orientation,
            and identification of all the markers/AprilTags in the world. The
            format of each row is (x,y,theta,id) with x,y giving 2D position,
            theta giving orientation, and id being an integer specifying the
            unique identifier of the tag.
        occupancy_map - an N by M numpy array of boolean values (represented as
            integers of either 0 or 1). This represents the parts of the map
            that have obstacles. It is mapped to metric coordinates via
            x_spacing and y_spacing
        pos_init - a 3 by 1 array specifying the initial position of the robot,
            formatted as usual as (x,y,theta)
        pos_goal - a 3 by 1 array specifying the final position of the robot,
            also formatted as (x,y,theta)
        max_speed - a parameter specifying the maximum forward speed the robot
            can go (i.e. maximum control signal for v)
        max_omega - a parameter specifying the maximum angular speed the robot
            can go (i.e. maximum control signal for omega)
        x_spacing - a parameter specifying the spacing between adjacent columns
            of occupancy_map
        y_spacing - a parameter specifying the spacing between adjacent rows
            of occupancy_map
        t_cam_to_body - numpy transformation between the camera and the robot
            (not used in simulation)
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
        self.index = 0

        # TODO for student: Comment this when running on the robot 
        self.robot_sim = RobotSim(world_map, occupancy_map, pos_init, pos_goal,
                                  max_speed, max_omega, x_spacing, y_spacing)
        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        #self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # Uncomment as completed
        self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

        # initiallize pose and plan path
        self.path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
        print self.path
        print len(self.path)
        meas = self.robot_sim.get_measurements()
        self.state = self.kalman_filter.step_filter(None, None, meas)

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and esimtaiton
        are done. This function is called at 60Hz
        """
        # TODO for student: Comment this when running on the robot
        imu_meas = self.robot_sim.get_imu()
        # self.cmd = self.diff_drive_controller.compute_vel(self.state, self.path[5])
        # print self.cmd
        # self.robot_sim.command_velocity(self.cmd[0], self.cmd[1])
        goal = self.path[self.index]
        self.cmd = self.diff_drive_controller.compute_vel(self.robot_sim.pose, goal)
        if self.cmd[2] == 0:
            self.robot_sim.command_velocity(self.cmd[0], self.cmd[1])
            imu_meas = self.robot_sim.get_imu()
            meas = self.robot_sim.get_measurements()
            self.state = self.kalman_filter.step_filter(self.cmd[0], imu_meas, meas)
            self.robot_sim.set_est_state(self.state)
        else:
            self.index += 1

        # TODO for student: Use this when transferring code to robot
        # meas = self.ros_interface.get_measurements()
        # imu_meas = self.ros_interface.get_imu()

        return

    def trackTag(self, tag, meas):
        state = [0, 0, 0]
        target_tag = tag
        cmd = [0, 0]
        if (meas!=None):
            for tag in meas:
                if tag[3] == target_tag:
                    goal = [tag[0], tag[1], tag[2]]
                    cmd = self.diff_drive_controller.compute_vel(state, goal)
                    if cmd[2] != 1:
                        self.robot_sim.command_velocity(cmd[0], cmd[1])
                        return cmd
                    else:
                        cmd = [0,0]
                        self.robot_sim.command_velocity(0,0)
                        return cmd
        return None


def main(args):
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    # print(pos_init)
    # print(pos_goal)
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map, occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body)

    # TODO for student: Comment this when running on the robot 
    # Run the simulation
    while not robotControl.robot_sim.done and plt.get_fignums():
        robotControl.process_measurements()
        robotControl.robot_sim.update_frame()

    plt.ioff()
    plt.show()

    # TODO for student: Use this to run the interface on the robot
    # Call process_measurements at 60Hz
    """r = rospy.Rate(60)
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)"""

if __name__ == "__main__":
    main(sys.argv)


