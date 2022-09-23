#!/usr/bin/env python3
import numpy as np
import math
import rospy

from std_msgs.msg import Float64MultiArray, Float64
from modules.sprites import *
from matplotlib.pyplot import plot

rospy.init_node('pure_pursuit_control')
long_drive_pub = rospy.Publisher('controls/throttle', Float64, queue_size=10)
steering_drive_pub = rospy.Publisher('controls/steer', Float64, queue_size=10) 

current_state = np.zeros(6)
def update_car_state(data):
    global current_state
    current_state[:3] = data.data  # [x, y, theta, speed, beta (slip angle), theta_dot]

def update_vel(data):
    global current_state
    current_state[3] = data.data

def update_steering(data):
    global current_state
    current_state[4] = data.data

curr_waypoint = None
def update_waypoint(data):
    global curr_waypoint
    curr_waypoint = np.array(data.data)  # [x, y, yaw_path] of next waypoint

rospy.Subscriber("vehicle_model/state", Float64MultiArray, update_car_state)
rospy.Subscriber('car_actions/vel', Float64, update_vel)
rospy.Subscriber('car_actions/vel', Float64, update_steering)
rospy.Subscriber("waypoints", Float64MultiArray, update_waypoint)

class Controller:
    def __init__(self, L=4.9):
        self.L = L
        self.error_prev = 0
        self.error_I = 0
       
    def get_longitudinal_control(self,v_current,v_desired,dt):
        '''
        PID Longitudinal controller
        Parameters
        ----------
        v_current: float
            Current speed of the vehicle
        v_desired: float
            Desired speed of the vehicle
        dt: float
            Delta time since last time the function was called

        Returns
        -------
        throttle_output: float
            Value in the range [-1,1]
        '''
        gains = [0.19,0.0035,0.4]                     #Kp, Ki, Kd (ideal gains = [0.19,0.0035,0.4])
        error = v_desired - v_current
        error_d = (error - self.error_prev)
        self.error_I += error
        output = error*gains[0] + self.error_I*gains[1] + error_d*gains[2]
        throttle_output = sorted((-1, output, 1))[1]
        self.error_prev = error
        return throttle_output
        pass
    
    def get_lateral_pure_pursuit(self,current_xy,current_yaw,next_waypoint):
        '''
        Pure Pursuit, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        next_waypoint: np.array of floats, shape=2
            Next waypoint for the vehicle to reach [x,y]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''

        current_xy = (current_xy[0] - 0.5*self.L*np.cos(current_yaw)), (current_xy[1] - 0.5*self.L*np.sin(current_yaw))
        L_d = math.dist(current_xy,next_waypoint)
        direction = next_waypoint - current_xy
        unit_dir = direction / np.linalg.norm(direction)
        heading = np.arccos(np.clip(np.dot(unit_dir, [1,0]), -1.0, 1.0))
        if direction[1] < 0 :
            heading = -heading
        alpha = heading - current_yaw

        steer_angle = np.arctan((2*self.L*np.sin(alpha))/L_d)
        return steer_angle

        pass

    def get_lateral_stanley(self,current_xy,current_yaw,current_speed,next_waypoint):
        '''
        Stanley, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        current_speed: float
            Current speed of the vehicle
        next_waypoint: np.array of floats, shape=3
            Next waypoint for the vehicle to reach [x,y,yaw_path]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''
        pass
       
controller = Controller()
 
error_prev = 0
error_integ = 0


rate = 10
time = 0
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    r.sleep()
    if curr_waypoint is None:
        continue
        
    # Getting states variables from current car state (position, heading, speed)
    
    
    # Longitudinal and lateral control
    longitudinal_cont = controller.get_longitudinal_control(current_state[3],4,0.1)
    lateral_cont = controller.get_lateral_pure_pursuit(current_state[[0,1]],current_state[2],curr_waypoint[[0,1]])

    # Create longitudinal and lateral messages (of type Float64 imported above)
    throttle = Float64(longitudinal_cont)
    steer = Float64()
    steer.data = lateral_cont
    # Publish the 2 messages
    long_drive_pub.publish(throttle)
    steering_drive_pub.publish(steer)
    print("Torque: {:.2f}, Steering angle: {:.2f}".format(longitudinal_cont,lateral_cont))
    print(current_state)
