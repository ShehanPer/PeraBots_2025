import cv2
import numpy as np
from config import *


Kp = 0.15   # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.00  # Derivative gain

TARGET_DISTANCE = 60  # Ideal distance sensor reading from right wall
integral=0
previous_error=0
base_speed = 2


TIME_STEP = 32
MAX_SPEED=6
FORWARD_SPEED = 3.0  # Speed for moving forward/backward
TURN_SPEED = 1.2  
G= 1.5  # Gain for turning correction


# Extract middle 1-pixel strip from the camera frame
def get_middle_horizontal_strip(frame):
    height, width = frame.shape[:2]
    center = (height // 2)
    bottom = (height//2)+40
    blackline= bottom+30

    middlestrip = frame[center:center+1, :].squeeze(axis=0)
    bottomstrip = frame[bottom:bottom+1, :].squeeze(axis=0)
    blacktestStrip = frame[blackline:blackline+1, :].squeeze(axis=0)

    # Optional visualization
    frame_with_line = frame.copy()
    cv2.line(frame_with_line, (0, center), (width, center), (0, 255, 0), 1)
    cv2.circle(frame_with_line, (width // 2, center ), 5, (0, 0, 255), -1)
    cv2.line(frame_with_line, (0, bottom), (width, bottom), (0, 255, 0), 1)
    cv2.circle(frame_with_line, (width // 2, bottom), 5, (0, 0, 255), -1)
    cv2.putText(frame_with_line, f"Bottom Strip {bottom}", (10, bottom + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(frame_with_line, f"Middle Strip {width//2}", (10, center + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.line(frame_with_line, (0, blackline), (width, blackline), (0, 255, 0), 1)

    return middlestrip,bottomstrip,blacktestStrip,frame_with_line

# Convert strip of pixels into 0 (white) and 1 (black) based on color
def getcolorArray(strip):
    colArray = []
    for item in strip:
        if item[1] > 200 and item[2] > 200:
            colArray.append(0)  # White
        elif item[1] < 100 and item[2] < 100:
            colArray.append(1) 
        else:
            colArray.append(5)
    return colArray

# Calculate speed difference for turning
def turn(error):
    global base_speed, MAX_SPEED
    left_speed = base_speed + error
    right_speed = base_speed - error
    left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# Find the longest run of 0's (white pixels)
def find_longest_zero_run(arr):
    if all(val == 1 for val in arr):
        return (-1, -1)
    if all(val == 0 for val in arr):
        return (-2, -2)
        
    max_run = (-1, -1)  # Initialize with invalid indices
    max_length = 0
    in_zero_run = False
    start = 0

    for i, val in enumerate(arr):
        if val == 0 and not in_zero_run:
            in_zero_run = True
            start = i
        elif val == 1 and in_zero_run:
            in_zero_run = False
            length = i - start
            if length > max_length:
                max_length = length
                max_run = (start, i - 1)
    
    # Handle case where array ends with zeros
    if in_zero_run:
        length = len(arr) - start
        if length > max_length:
            max_run = (start, len(arr) - 1)

    return max_run

# PID controller output
def setPID(dif):
    global integral, previous_error
    integral += dif
    derivative = dif - previous_error
    output = Kp * dif + Ki * integral + Kd * derivative
    previous_error = dif
    return output

# Decide turning based on detected strip position
def decideDirection(val,width):
    #print(val,(width/2))
    #print("Value:", val, "Width:", width)
    error = setPID(val - (width/2))  # Assuming center of frame = 100
    turn(error)
