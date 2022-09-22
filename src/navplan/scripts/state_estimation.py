#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

rospy.init_node("state_estimation")

gps_observation = [0,0,0] # x, y, theta
steering = 0
velocity = 0

def predict():
    global predicted_state,car_state,velocity,predicted_uncertainty,estimated_uncertainty,model_error
    predicted_state = car_state
    predicted_state[0] += velocity * np.cos(steering + car_state[2]) * t
    predicted_state[1] += velocity * np.sin(steering + car_state[2]) * t
    predicted_state[2] += velocity/L * np.sin(steering) * t

    jacobian = np.array([[1,0,-velocity*t*np.sin(car_state[2]+steering)],[0,1,velocity*t*np.cos(car_state[2]+steering)],[0,0,1]], dtype= 'float64')
    predicted_uncertainty = (jacobian @ estimated_uncertainty @ jacobian.T) + model_error

def kalman_filter():
    global car_state,estimated_uncertainty
    kalman_gain = predicted_uncertainty @ h_t.T @ np.linalg.inv(h_t @ predicted_uncertainty @ h_t.T + gps_error)
    car_state = predicted_state + kalman_gain @ (gps_observation - predicted_state)
    estimated_uncertainty = (np.eye(3) - kalman_gain @ h_t) @ predicted_uncertainty

def gps_clbk(data):
    global gps_observation
    gps_observation = data.data

def steer_clbk(data):
    global steering
    steering = data.data

def vel_clbk(data):
    global velocity
    velocity = data.data
# Subscribe to the gps, steering, and velocity topics named below and update the global variables using callbacks
gps_sub = rospy.Subscriber("/gps", Float64MultiArray, gps_clbk)
steer_sub = rospy.Subscriber("/car_actions/steer", Float64, steer_clbk)
vel_sub = rospy.Subscriber("/car_actions/vel", Float64, vel_clbk)

# Publisher for the state
state_pub = rospy.Publisher('vehicle_model/state', Float64MultiArray, queue_size=10)
r = rospy.Rate(10)

# Initialize the start values and matrices here
t = 1/10
L = 4.9
car_state = np.array([0,0,0])
predicted_state = np.array([0,0,0])
estimated_uncertainty = np.array([0,0,0])
predicted_uncertainty = np.array([0,0,0])
h_t = np.diag([1]*3)
gps_error = np.diag([1,1,0.2])
model_error = np.diag([0.09,0.08,0.01])
kalman_gain = np.array([0,0,0])
while not rospy.is_shutdown():
    # Create the Kalman Filter here to estimate the vehicle's x, y, and theta
    predict()
    kalman_filter()

    # Create msg to publish#
    current_state = Float64MultiArray()
    layout = MultiArrayLayout()
    dimension = MultiArrayDimension()
    dimension.label = "current_state"
    dimension.size = 3
    dimension.stride = 3
    layout.data_offset = 0
    layout.dim = [dimension]
    current_state.layout = layout
    current_state.data = car_state

    state_pub.publish(current_state)
    r.sleep()