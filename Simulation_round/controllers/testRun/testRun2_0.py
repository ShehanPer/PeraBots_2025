from controller import Robot
import numpy as np
import math
# Constants
TIME_STEP = 32
WHEEL_RADIUS = 0.02     # meters (adjust to your robot)
AXLE_LENGTH = 0.08      # meters (distance between wheels)
MAX_SPEED=6
TURN_SPEED = 2.0

WHEEL_RADIUS = 0.06  # meters
AXLE_LENGTH = 0.08   # distance between wheels
MAP_RES = 0.05       # 5cm per cell
MAP_SIZE = 50      # 100x100 grid
CENTER = MAP_SIZE // 2

# Init robot
robot = Robot()
path_map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=int)
x, y, theta = 0.0, 0.0, 0.0  # meters, radians
prev_left = 0.0
prev_right = 0.0
# Motors
left_motor = robot.getDevice("left motor")
right_motor = robot.getDevice("right motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Encoders
left_encoder = robot.getDevice("left encoder")
right_encoder = robot.getDevice("right encoder")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# Gyro
gyro = robot.getDevice("gyroScope")
gyro.enable(TIME_STEP)


cam=robot.getDevice("camera")
cam.enable(TIME_STEP)
width = cam.getWidth()
height = cam.getHeight()
# Initialize variables

# IR Sensors
right_IR=robot.getDevice("rightIR")
left_IR=robot.getDevice("leftIR")
left_IR.enable(TIME_STEP)
right_IR.enable(TIME_STEP)

def ir_to_distance(value):
    return 0.1594*value**(-0.8533)-0.02916 

def get_pathWidth():
    rightDist = ir_to_distance(right_IR.getValue())
    leftDist = ir_to_distance(left_IR.getValue())
    print("Width : ",leftDist,rightDist)
    return leftDist,rightDist
    
    
# Start moving
left_motor.setVelocity(1.0)
right_motor.setVelocity(1.0)
KP = 0.1
KI = 0
KD = 0.001
TARGET_DISTANCE = 60  # Ideal distance sensor reading from right wall
integral=0;
prev_error=0;
base_speed = 4

def is_white(pixel, threshold=230):
    arr=[]
    count=4
    r,b,w=0,0,0
    for color in pixel:
        if color[0]>threshold:
            if color[1]>threshold:
                r+=1
                w,b=0,0
                if r>count:
                    r=0
                    arr.append(1)  #red
                
            else:
                w+=1
                r,b=0,0
                if w>count:
                    w=0
                    arr.append(2) #white
        else:
            w,r=0,0
            b+=1
            if b>count:
                b=0
                arr.append(0) #black
    for i in arr:
        if i==1:
            ind=arr.index(1)
            break
        elif i==2:
            ind=arr.index(2)
            break
    else:ind=0
    c=ind
    areas=[]
    flg=1
    print('ind',ind)
    while c<len(arr)-1:
        if (arr[c]==1 or arr[c]==2) and (arr[c+1]==0 or c==len(arr)-2):
            areas.append([ind,(c-ind)])
            flg=1
        elif (arr[c]==1 or arr[c]==2) and flg :
            ind=c+1
            flg=0
        c+=1
    max=0
    point=0
    print(areas)
    for area in areas:
        if max<area[1]:
            max=area[1]
            point=area[0]
            
    center=(point+max/2)*count
    return center

blocktime=0
while robot.step(TIME_STEP) != -1:
    dt = TIME_STEP / 1000.0 
    image=cam.getImage()
    
    RIR,LIR=get_pathWidth()
    print("r",RIR,"LL",LIR)
    img = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            r = cam.imageGetRed(image, width, x, y)
            g = cam.imageGetGreen(image, width, x, y)
            b = cam.imageGetBlue(image, width, x, y)
            img[y, x] = [r, g, b]

    # Extract middle vertical line
    mid_x = width // 2
    vertical_line1 = img[70,:]
    vertical_line2 = img[110,:]
    vertical_line3 = img[20,:]
    center3= is_white(vertical_line3)
    center= is_white(vertical_line1)
    center2= is_white(vertical_line2)
    
    print("center",center,center2)
    #if center <center2:center=center2
    #if center>center3:center=center3
    if RIR< 0.16 and LIR <0.2:
        error = RIR-.03
        print('errorrs',error,RIR)
        KP = 3
        KI = 0
        KD = 0.03
    elif LIR< 0.16 and RIR <0.2:
        error = LIR-.03
        print('errorls',error)
        KP = 1
        KI = 0
        KD = 0.03
    else :
        KP = 0.08
        KI = 0.0
        KD = 0.01
        error = center-mid_x+50
    print('error',error)
    integral += error
    derivative = error - prev_error

    # PID output
    correction = KP * error + KI * integral + KD * derivative
    prev_error = error

    # Base speed
    
    print('correction',correction)
    # Simple obstacle avoidance (optional)
        # Apply correction: + correction = move away from wall
    left_speed = base_speed - correction
    right_speed = base_speed + correction

    # Limit speed
    left_speed = min(MAX_SPEED, max(-MAX_SPEED,left_speed))
    right_speed = min(MAX_SPEED, max(-MAX_SPEED,right_speed))

    # Set wheel speeds
    left_motor.setVelocity(right_speed)
    right_motor.setVelocity(left_speed)
    
