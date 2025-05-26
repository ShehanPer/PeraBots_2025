from controller import Robot
import numpy as np
import math
# Constants
TIME_STEP = 32
MAX_SPEED=6
TURN_SPEED = 2.0

WHEEL_RADIUS = 0.03  # meters
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


# IMU
imu = robot.getDevice("Inertial Unit")
imu.enable(TIME_STEP)

# Gyro
gyro = robot.getDevice("gyroScope")
gyro.enable(TIME_STEP)


cam=robot.getDevice("camera")
cam.enable(TIME_STEP)
# Initialize variables

orientation = 0.0  # Yaw in radians
ds_names = ['ps4', 'ps0', 'ps2']
sensors = [robot.getDevice(name) for name in ds_names]
for sensor in sensors:
    sensor.enable(TIME_STEP)
# Start moving
left_motor.setVelocity(1.0)
right_motor.setVelocity(1.0)
KP = 0.005
KI = 0.0001
KD = 0.001
TARGET_DISTANCE = 60  # Ideal distance sensor reading from right wall
integral=0;
prev_error=0;
base_speed = 2
path=[]
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
init_right = right_encoder.getValue()
init_left = left_encoder.getValue()
print("Initial Encoders")
print(init_right)
print(init_left)


while robot.step(TIME_STEP) != -1:

    # '''dt = TIME_STEP / 1000.0 
    # left_val = sensors[1].getValue()
    # front_val = sensors[2].getValue()
    # right_val = sensors[0].getValue()
    # print(f"L: {left_val:.2f}, F: {front_val:.2f}, R: {right_val:.2f}")
    ##---- ORIENTATION (YAW) FROM GYRO ----

    
    # error = TARGET_DISTANCE - left_val
    # integral += error
    # derivative = error - prev_error

    ##PID output
    # correction = KP * error + KI * integral + KD * derivative
    # prev_error = error

    ##Base speed
    

    ##Simple obstacle avoidance (optional)
    # if front_val > 950:
        # left_speed = -2.0
        # right_speed = 2.0
    # else:
        ##Apply correction: + correction = move away from wall
        # left_speed = base_speed - correction
        # right_speed = base_speed + correction

        ##Limit speed
        # left_speed = min(MAX_SPEED, max(-MAX_SPEED,1))
        # right_speed = min(MAX_SPEED, max(-MAX_SPEED,1.1))

    ##Set wheel speeds
    # left_motor.setVelocity(left_speed)
    # right_motor.setVelocity(right_speed)
    
    # print(f"Right Sensor: {right_val:.2f}, Error: {error:.2f}, Correction: {correction:.2f}")
    yaw=imu.getRollPitchYaw()[2]
    print(yaw)


    ##---- CALCULATING DISPLACEMENT FROM ENCODERS ----
    curr_left = left_encoder.getValue()
    
    curr_right = right_encoder.getValue()
    
    dist=(curr_right-init_right)*WHEEL_RADIUS
    
    print(curr_left)
    print(curr_right)
    print("Dist - ",dist)
    left_motor.setVelocity(1.0)
    right_motor.setVelocity(1.0)


    ##Mark on map
    # my = int(x)-130
    # mx = int( y )+30
    # print ('x ',mx,'y ',my,'thet',theta,v)
    # if 0 <= mx < MAP_SIZE and 0 <= my < MAP_SIZE:
        # path_map[my][mx] = 1
        # print('visit',path)
        # if [my,mx] not in path : path.append([my,mx])
        

    # print(f"x: {x:.2f}, y: {y:.2f}, θ: {math.degrees(theta):.1f}°, v: {v:.2f} m/s")
    # print_path_map(path_map, mx, my)

    ##Print values
    # print(f"Orientation (yaw): {orientation:.2f} rad")
    
    
