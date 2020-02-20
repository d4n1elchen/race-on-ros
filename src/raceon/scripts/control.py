#!/usr/bin/env python
# license removed for brevity

import rospy
from PID import PID
from std_msgs.msg import Int8, Bool
from geometry_msgs.msg import Pose
from raceon.msg import AckermannDrive

import numpy as np

SERVO_MIN = -900
SERVO_MIDDLE = 0
SERVO_MAX = 900
            
# Decrease speed when turning
def sigmoid(x):
    return 1/(1 + np.exp(-x))

def decreaseCurve(x):
    #return np.maximum(0.0, np.tanh((abs(x)-0.05)*16))
    return sigmoid((abs(x)+0.1)*500)

class Controller():
    
    def __init__(self):
        self.topic_name_pos_err = rospy.get_param("topic_name_position_error", "position/error")
        self.topic_name_pos_state = rospy.get_param("topic_name_position_state", "position/state")
        self.topic_name_manual_mode = rospy.get_param("topic_name_manual_mode", "control/manual_mode")
        self.topic_name_control = rospy.get_param("topic_name_control", "control")
        
        # Parameters for control
        self.motor_speed = rospy.get_param("~motor_speed", 200)
        self.steering_speed = rospy.get_param("~steering_speed", 160)
        self.track_width = rospy.get_param("~track_width", 600)
        self.target = rospy.get_param("~target", 0)
        self.kp = rospy.get_param("~kp", 1)
        self.ki = rospy.get_param("~ki", 0)
        self.kd = rospy.get_param("~kd", 0)
        
        # Init PID
        self.pid = PID(P=self.kp, I=self.ki, D=self.kd)
        
        # Speed command
        self.manual_mode = False
        self.state_u = 0
        self.state_d = 0
        self.break_cnt = 0
        self.decrease_max = self.motor_speed - self.steering_speed
    
    def start(self):
        self.sub_pos_err   = rospy.Subscriber(self.topic_name_pos_err, Pose, self.pos_err_callback)
        self.sub_pos_state = rospy.Subscriber(self.topic_name_pos_state, Int8, self.pos_state_callback)
        self.sub_manual_mode = rospy.Subscriber(self.topic_name_manual_mode, Bool, self.manual_mode_callback)
        self.pub_control = rospy.Publisher(self.topic_name_control, AckermannDrive, queue_size=10)
        rospy.spin()

    def manual_mode_callback(self, mode_msg):
        self.manual_mode = mode_msg.data
        if self.manual_mode:
            control_msg = AckermannDrive()
            control_msg.speed = 0
            control_msg.steering_angle = 0
            self.pub_control.publish(control_msg)

    def pos_err_callback(self, pos_err_msg):
        if self.manual_mode:
            rospy.loginfo("Mannual mode is on. Not running control")
        else:
            pos_err = pos_err_msg.position.x
            
            rospy.loginfo("Current error: pos_err = " + str(pos_err))
            
            servo_pos = self.control_servo(pos_err)
            motor_speed = self.decrease_speed_state_two_scan()
            
            rospy.loginfo("Control command: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(motor_speed))
            
            control_msg = AckermannDrive()
            control_msg.speed = motor_speed
            control_msg.steering_angle = servo_pos
            self.pub_control.publish(control_msg)
    
    def pos_state_callback(self, pos_state_msg):
        self.state_u = pos_state_msg.data >> 4
        self.state_d = pos_state_msg.data & 0b1111
    
    def decrease_speed_pos_err(self, pos_err):
        if self.state == 0: # Two line
            return self.motor_speed
        elif self.state > 0: # One line
            return self.motor_speed - self.decrease_max * decreaseCurve(pos_err / (self.track_width/2))
    
    def decrease_speed_servo(self, servo_pos):
        if self.state == 0: # Two line
            return self.motor_speed
        elif self.state > 0: # One line
            return self.motor_speed - self.decrease_max * decreaseCurve(servo_pos / SERVO_MAX)
        
    def decrease_speed_state(self):
        if self.state_d == 0: # Both two lines
            return self.motor_speed
        else: # One of the scan line contains one or zero line
            return self.steering_speed

    def decrease_speed_state_two_scan(self):
        if self.break_cnt > 4: # disable break after 4 frame passed
            if self.state_d == 0:
                self.break_cnt = 0 # enable break when back to straight
                return self.motor_speed
            else:
                return self.steering_speed
        elif self.break_cnt > 0: # during break
            self.break_cnt += 1
            return 0
        elif self.break_cnt == 0: # wait for break
            if self.state_d == 0 and self.state_u == 0: # if everything good full speed
                return self.motor_speed
            else: # upper or lower scan line lost one of the lines, break anyway!
                self.break_cnt = 1
                return 0
        
    def _pid(self, error):
        return error * self.kp

    def control_servo(self, error):
        self.pid.update(error)
        servo_pos = self.pid.output

        if servo_pos > SERVO_MAX:
            servo_pos = SERVO_MAX
        if servo_pos < SERVO_MIN:
            servo_pos = SERVO_MIN

        return servo_pos

if __name__ == "__main__":
    rospy.init_node("control")
    controller = Controller()
    try:
        controller.start()
    except rospy.ROSInterruptException:
        pass
