from controller import Robot,Keyboard
import numpy as np
import math
from matrix_map import *

# Constants
TIME_STEP = 32
MAX_SPEED=6
FORWARD_SPEED = 3.0  # Speed for moving forward/backward
TURN_SPEED = 1.5     # Speed for turning


# Initialize Keyboard
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)


# Motors
left_motor = robot.getDevice("left motor")
right_motor = robot.getDevice("right motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

#Camera
cam=robot.getDevice("camera")
cam.enable(TIME_STEP)


#Ultrasonic Sensors
ds_names = ['ps4', 'ps0', 'ps2']
sensors = [robot.getDevice(name) for name in ds_names]
for sensor in sensors:
    sensor.enable(TIME_STEP)
    
    


KP = 0.005
KI = 0.0001
KD = 0.001
TARGET_DISTANCE = 60  # Ideal distance sensor reading from right wall
integral=0
prev_error=0
base_speed = 2

def print_path_map(path_map, robot_x, robot_y):
    rows, cols = path_map.shape
    for r in range(rows):
        line = ''
        for c in range(cols):
            if r == robot_y and c == robot_x:
                line += 'R'  # Robot current position
            elif path_map[r, c] == 1:
                line += '*'
            else:
                line += '.'
        print(line)

robot.step(TIME_STEP)
current_encoder_values[0]=left_encoder.getValue()
current_encoder_values[1]=right_encoder.getValue()

while robot.step(TIME_STEP) != -1:

        # 1. Read Keyboard Input
    key = keyboard.getKey()
    
    left_speed = 0.0
    right_speed = 0.0

    if key == Keyboard.UP or key == ord('W'):
        left_speed = FORWARD_SPEED
        right_speed = FORWARD_SPEED
    elif key == Keyboard.DOWN or key == ord('S'):
        left_speed = -FORWARD_SPEED
        right_speed = -FORWARD_SPEED
    elif key == Keyboard.LEFT or key == ord('A'):
        left_speed = -TURN_SPEED
        right_speed = TURN_SPEED
    elif key == Keyboard.RIGHT or key == ord('D'):
        left_speed = TURN_SPEED
        right_speed = -TURN_SPEED

    # Set motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


    update_position()
    update_map_with_all_sensors()
    update_map()
    current_simulation_time = robot.getTime()
    if current_simulation_time % 5 == 0:
        print(f"Sim time: {robot.getTime():.2f}s - Saving map...")
        # Ensure MAP_RES (from matrix_map.py) is accessible or pass it
        save_map_json(MAP, MAP_RES, f"robot_map_2cm.json")

