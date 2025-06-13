from controller import Robot
import cv2
import numpy as np
from movemap import*
from matrix_map import*


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

isredSeen=0

#Ultrasonic Sensors
ds_names = ['ps4', 'ps0', 'ps2']
sensors = [robot.getDevice(name) for name in ds_names]
for sensor in sensors:
    sensor.enable(TIME_STEP)
    
    




current_encoder_values[0]=left_encoder.getValue()
current_encoder_values[1]=right_encoder.getValue()

set_movemap_Devices(left_motor, right_motor, camera, sensors,left_encoder,right_encoder,imu,gyro,right_IR,left_IR )

robot.step(TIME_STEP)
def stopwhenredSeeRed(frame):
    isdet=isredDetected(frame)
    if isdet:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        return True
    return False
counter=0
lap=0
print("Started lap", lap+1)
while robot.step(TIME_STEP) != -1:
    counter=counter+1
    if(counter>10):
        break
    left_motor.setVelocity(3)
    right_motor.setVelocity(3)
while robot.step(TIME_STEP) != -1:

    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    # Convert Webots BGRA image to BGR
    img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    frame = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
    height, width = frame.shape[:2]
    middlestrip,bottomstrip,blacktestStrip,frame_with_line=get_middle_horizontal_strip(frame)

    blacktestStripArray=getcolorArray(blacktestStrip)
    middlecolArray=getcolorArray(middlestrip)
    bottomcolArray=getcolorArray(bottomstrip)

    midzeroIndexes=find_longest_zero_run(middlecolArray)
    bottomzeroIndexes=find_longest_zero_run(bottomcolArray)

    line_y_top = 0
    line_y_bottom = frame_with_line.shape[0] 
    cv2.line(frame_with_line, (midzeroIndexes[0], line_y_top), (midzeroIndexes[0], line_y_bottom), (0, 255, 0), 2)
    cv2.line(frame_with_line, (midzeroIndexes[1], line_y_top), (midzeroIndexes[1], line_y_bottom), (0, 255, 0), 2)

    cv2.line(frame_with_line, (bottomzeroIndexes[0], line_y_top), (bottomzeroIndexes[0], line_y_bottom), (255, 0, 0), 2)
    cv2.line(frame_with_line, (bottomzeroIndexes[1], line_y_top), (bottomzeroIndexes[1], line_y_bottom), (255, 0, 0), 2)
    cv2.rectangle(frame_with_line, (20, height//2), (280, height), (0, 0, 0), 2)
    cv2.rectangle(frame_with_line, ((width // 2)-25, (height//2)-10), ((width // 2)+25, (height//2) +70), (0, 0, 0), 2)

    midVal=int((midzeroIndexes[0]+midzeroIndexes[1])/2)
    midlength=midzeroIndexes[1]-midzeroIndexes[0]
    bottomval=int((bottomzeroIndexes[0]+bottomzeroIndexes[1])/2)
    bottomlength=bottomzeroIndexes[1]-bottomzeroIndexes[0]
    difval=midVal-bottomval
    
    if(not isredDetected(frame) and isredSeen==1):
        print(lap+1,"laps completed")
        if(lap>=4):#since need to complete 5 laps
            print("Stopping the robot.......")
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            break
        else:
            lap=lap+1
            print("Started lap", lap+1)
            isredSeen=0
            continue
    elif(stopwhenredSeeRed(frame)):
        isredSeen=1
        decideDirection(150,width)
    elif(midzeroIndexes==(-1,-1) or midzeroIndexes==(-2,-2)):
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
    else:
        if((blacktestStripArray[20]==1 or blacktestStripArray[280]==1 )and bottomzeroIndexes not in [(-1, -1), (-2, -2)] ):
            decideDirection(bottomval-difval/2,width)
        elif(bottomzeroIndexes not in [(-1, -1), (-2, -2)] and midlength > 90 ):
            decideDirection(bottomval,width)
        else:
            decideDirection(midVal,width)




    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow("frame", frame_with_line)
    update_position()
    #update_map_with_all_sensors()
    # update_map()
    # current_simulation_time = robot.getTime()
    # if current_simulation_time % 5 == 0:
    #     print(f"Sim time: {robot.getTime():.2f}s - Saving map...")
    #     # Ensure MAP_RES (from matrix_map.py) is accessible or pass it
    #     save_map_json(MAP, MAP_RES, f"robot_map_2cm.json")

cv2.destroyAllWindows()