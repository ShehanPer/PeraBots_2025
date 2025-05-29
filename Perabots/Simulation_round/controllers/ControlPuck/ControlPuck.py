from controller import Robot
import cv2
import numpy as np


# Time step of the simulation in milliseconds
TIME_STEP = 64

# Initialize PID parameters
Kp = 0.2   # Proportional gain
Ki = 0.03  # Integral gain
Kd = 0.00  # Derivative gain

# Initialize integral and previous error
integral = 0
previous_error = 0


baseSpeed=5
max_speed=6
# Create the Robot instance
robot = Robot()

# Get motors
left_motor = robot.getDevice('left motor')
right_motor = robot.getDevice('right motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Get and enable camera
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

#Ultrasonic Sensors
ds_names = ['ps4', 'ps0', 'ps2']
sensors = [robot.getDevice(name) for name in ds_names]
for sensor in sensors:
    sensor.enable(TIME_STEP)

KP = 0.005
KI = 0.0001
KD = 0.001
TARGET_DISTANCE = 60  # Ideal distance sensor reading from right wall
integral=0;
prev_error=0;
base_speed = 2

# Define function to extract middle horizontal strip
def get_middle_horizontal_strip(frame):
    height, width = frame.shape[:2]
    center = height // 2

    # Extract exactly 1-pixel-tall horizontal strip
    strip = frame[center:center+1, :].squeeze(axis=0) 

    # Draw line on the original frame
    frame_with_line = frame.copy()
    cv2.line(frame_with_line, (0, center+25), (width, center+25), (0, 255, 0), 1)

    # Show the full frame with the line
    cv2.imshow("Full Frame with Center Line", frame_with_line)

    return strip
    
def getcolorArray(strip,striplen):
    colArray=[]
    for item in strip:
        if(item[1]>200 and item[2]>200):
            #print("White")
            colArray.append(0)
        if(item[1]<100 and item[2]<100):
            #print("Balck")
            colArray.append(1)

        #if red ditected implement a code to go until it dont see red
    #print('-///////////////////////////////////////////////-')       
    return colArray

#def trun(direction):

#    match direction:
#        case 1:#turn left
#            left_motor.setVelocity(normalSpeed)
#            right_motor.setVelocity(normalSpeed+2)
#        case 2:#turn right
#            left_motor.setVelocity(normalSpeed+2)
#            right_motor.setVelocity(normalSpeed)
#        case 3:#move forward
#            left_motor.setVelocity(normalSpeed)
#            right_motor.setVelocity(normalSpeed)
#        case _:#move backward
#            left_motor.setVelocity(-normalSpeed)
#            right_motor.setVelocity(-normalSpeed)

def turn(error):
    global baseSpeed,max_speed
    #max_turn = 5.0
    #pid_output = max(min(pid_output, max_turn), -max_turn)
  

    # Calculate left and right motor speeds with differential for turning
    left_speed = baseSpeed + error
    right_speed = baseSpeed - error

    # Optionally clamp speeds to allowed velocity range

    left_speed = max(min(left_speed, max_speed), -max_speed)
    right_speed = max(min(right_speed, max_speed), -max_speed)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
            
def find_longest_zero_run(arr):
    if all(val == 1 for val in arr):
        return (-1, -1)
    if all(val == 0 for val in arr):
        return (-2, -2)
        
    max_run = ()
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

    # If array ends in a zero run
    if in_zero_run:
        length = len(arr) - start
        if length > max_length:
            max_run = (start, len(arr) - 1)
    return max_run
def setPID(dif):
    global integral, previous_error
    integral += dif
    derivative = dif - previous_error
    output = Kp * dif + Ki * integral + Kd * derivative
    previous_error = dif
    
    return output

def decideDirection(val):
    error=setPID(val-100)
    print(error)
    turn(error)

#def decideDirection(val):
#    if(99<=val<=101):
#        trun(3)
#        return
#    if(101<val):
#        trun(2)
#        return
#    if(val<99):
#        trun(1)
#        return
#    pass
# Main control loop
while robot.step(TIME_STEP) != -1:
    # Get image from camera
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    # Convert Webots BGRA image to BGR
    img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    frame = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)

    # Process frame
    strip=get_middle_horizontal_strip(frame)
    striplen=len(strip)
    
    striplen=len(strip)
    
    colArray=getcolorArray(strip,striplen)
    print(colArray)
    zeroIndexes=find_longest_zero_run(colArray)
    if(zeroIndexes==(-1,-1) or zeroIndexes==(-2,-2)):
        print(f"Problem is encountered.....{zeroIndexes}")

    decideDirection(int((zeroIndexes[0]+zeroIndexes[1])/2))
    

    # Display results
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        


# Cleanup
cv2.destroyAllWindows()
