from controller import Robot
import numpy as np
import math
# Constants
TIME_STEP = 32

WHEEL_RADIUS = 0.03  # meters
AXLE_LENGTH = 0.08   # distance between wheels
MAP_RES = 0.04       # 4cmx4cm per cell
MAP_SIZE = 50      # 100x100 grid
MAP= np.zeros((MAP_SIZE, MAP_SIZE), dtype=int)  # Initialize the map


# Init robot
robot = Robot()


# Encoders
left_encoder = robot.getDevice("left encoder")
right_encoder = robot.getDevice("right encoder")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)


# IMU
imu = robot.getDevice("Inertial Unit")
imu.enable(TIME_STEP)

# Gyro
gyro = robot.getDevice("gyroScope")
gyro.enable(TIME_STEP)



ds_names = ['ps4', 'ps0', 'ps2']
sensors = [robot.getDevice(name) for name in ds_names]
for sensor in sensors:
    sensor.enable(TIME_STEP)


current_position = [-0.42,-0.54] # Initial position of the robot in meters
current_encoder_values = [0, 0]


def calculate_displacement():
    global current_encoder_values
    angle = imu.getRollPitchYaw()[2]
    left_enc = left_encoder.getValue()
    right_enc = right_encoder.getValue()
    distance = ((right_enc - current_encoder_values[1]) + (left_enc - current_encoder_values[0])) / 2 * WHEEL_RADIUS
    current_encoder_values[0] = left_enc
    current_encoder_values[1] = right_enc
    x_displacement = distance * math.cos(angle)
    y_displacement = distance * math.sin(angle)
    return [x_displacement, y_displacement]



def update_position():
    global current_position
    displacement = calculate_displacement()
    current_position[0] += displacement[0]
    current_position[1] += displacement[1]



def update_map():
    global MAP, current_position
    x_index = int((current_position[0] + 2.0) / MAP_RES) + MAP_SIZE // 2
    y_index = MAP_SIZE // 2 - int((current_position[1] + 2.0) / MAP_RES) 
    
    if 0 <= x_index < MAP_SIZE and 0 <= y_index < MAP_SIZE:
        MAP[y_index, x_index] = 1  # Mark the cell as visited


def print__map(path_map, robot_x, robot_y):
    pass
