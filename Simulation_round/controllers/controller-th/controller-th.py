from controller import Robot
import cv2
import numpy as np
from movemap import *
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
camera=robot.getDevice("camera")
camera.enable(TIME_STEP)


#Ultrasonic Sensors
ds_names = ['ps4', 'ps0', 'ps2']
sensors = [robot.getDevice(name) for name in ds_names]
for sensor in sensors:
    sensor.enable(TIME_STEP)
    
    


robot.step(TIME_STEP)

current_encoder_values[0]=left_encoder.getValue()
current_encoder_values[1]=right_encoder.getValue()

set_movemap_Devices(left_motor, right_motor, camera, sensors,left_encoder,right_encoder,imu,gyro,right_IR,left_IR )




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



while robot.step(TIME_STEP) != -1:

    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    # Convert Webots BGRA image to BGR
    img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    frame = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
    height, width = frame.shape[:2]
    #print("Width:", width, "Height:", height)
    middlestrip,bottomstrip,frame_with_line=get_middle_horizontal_strip(frame)
    #print(len(middlestrip))

    middlecolArray=getcolorArray(middlestrip)
    bottomcolArray=getcolorArray(bottomstrip)
    # print(middlecolArray)
    # print(bottomcolArray)

    midzeroIndexes=find_longest_zero_run(middlecolArray)
    bottomzeroIndexes=find_longest_zero_run(bottomcolArray)

    print(f"Middle Zero indexes: {midzeroIndexes}")
    print(f"Bottom Zero indexes: {bottomzeroIndexes}")
    line_y_top = 0
    line_y_bottom = frame_with_line.shape[0] 
    cv2.line(frame_with_line, (midzeroIndexes[0], line_y_top), (midzeroIndexes[0], line_y_bottom), (0, 255, 0), 2)
    cv2.line(frame_with_line, (midzeroIndexes[1], line_y_top), (midzeroIndexes[1], line_y_bottom), (0, 255, 0), 2)

    cv2.line(frame_with_line, (bottomzeroIndexes[0], line_y_top), (bottomzeroIndexes[0], line_y_bottom), (255, 0, 0), 2)
    cv2.line(frame_with_line, (bottomzeroIndexes[1], line_y_top), (bottomzeroIndexes[1], line_y_bottom), (255, 0, 0), 2)

    if(midzeroIndexes==(-1,-1) or midzeroIndexes==(-2,-2)):
        print(f"Problem is encountered.....{midzeroIndexes}")
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
    else:
        if(midzeroIndexes[1]-midzeroIndexes[0]<90):
            if(bottomzeroIndexes==(-1,-1) or bottomzeroIndexes==(-2,-2)):
                print(f"Problem is encountered.....{bottomzeroIndexes}")
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
            else:
                decideDirection(int((bottomzeroIndexes[0]+bottomzeroIndexes[1])/2),width)
        else:
            decideDirection(int((midzeroIndexes[0]+midzeroIndexes[1])/2),width)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow("Middle Strip", frame_with_line)
    update_position()
    update_map_with_all_sensors()
    update_map()
    current_simulation_time = robot.getTime()
    if current_simulation_time % 5 == 0:
         print(f"Sim time: {robot.getTime():.2f}s - Saving map...")
         # Ensure MAP_RES (from matrix_map.py) is accessible or pass it
         save_map_json(MAP, MAP_RES, f"robot_map_2cm.json")
cv2.destroyAllWindows()