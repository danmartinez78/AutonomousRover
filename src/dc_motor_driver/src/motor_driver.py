#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import sabertooth2x12 as SB

class MotorDriver:
    def __init__(self, motor_gain, wheel_sep, wheel_radius):
        self.test_mode = rospy.get_param("~test_mode",False)
        self._wheel_sep = wheel_sep
        self._wheel_rad = wheel_radius
        self._gear_ratio = 7.5*motor_gain
        self._max_rpm = 200
        self._max_pwm = 100

        self._mh = SB.Sabertooth()
        self._left_direction = "fwd"
        self._right_direction = "fwd"
        
        self.last_msg_time = None

        self.motors_on = False
        rospy.on_shutdown(self.turnOffMotors)
        
    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
	self._mh.stop
        self.motors_on = False
        
    def drive(self,twist):
        x = twist.linear.x
        w = twist.angular.z
        vel_left = (x-w*self._wheel_sep/2)/self._wheel_rad*self._gear_ratio
        vel_right = (x+w*self._wheel_sep/2)/self._wheel_rad*self._gear_ratio

        pwm_left = vel_left*self._max_pwm/self._max_rpm
        pwm_right = vel_right*self._max_pwm/self._max_rpm

        if (pwm_left < 0):
            self._left_direction = "rev"
            pwm_left = -pwm_left
        else:
            self._left_direction = "fwd"
        pwm_left = max(min(pwm_left,100),0)
        #if pwm_left < 45:
        #    pwm_left = 0
        if (pwm_right < 0):
            self._right_direction = "rev"
            pwm_right = -pwm_right
        else:
            self._right_direction = "fwd"
        pwm_right = max(min(pwm_right,100),0)

        #if pwm_right < 45:
        #    pwm_right = 0
        
        #print "Final pwm is:"
        #print int(pwm_left), int(pwm_right)
        
        self._mh.driveMotor('left', self._left_direction, int(pwm_left))
        self._mh.driveMotor('right', self._right_direction, int(pwm_right))

        self.motors_on = True
        
        #rospy.sleep(1)
        
        self.last_msg_time = rospy.get_rostime()
        

if __name__ == '__main__':
    node = rospy.init_node('motor_driver')

    param_path = rospy.get_param("~param_path")
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    motor_gain = params['motor_gain']
    wheel_sep = params['wheel_sep']
    wheel_radius = params['wheel_radius']

    driver = MotorDriver(motor_gain, wheel_sep, wheel_radius)
    
    rospy.Subscriber('cmd_vel', Twist, driver.drive, queue_size=1)

    rate = rospy.Rate(10)

    if driver.test_mode:
        timeout = 1
        rospy.loginfo("[motor_driver]: Test mode on")
    else:
        timeout = 0.1
    
    while not rospy.is_shutdown():
        #if driver.last_msg_time is not None and (((rospy.get_rostime() - driver.last_msg_time).to_sec()) > timeout) and driver.motors_on:
            #driver.turnOffMotors()
        rate.sleep()
