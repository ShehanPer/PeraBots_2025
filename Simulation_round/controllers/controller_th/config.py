from controller import Robot



# Constants
TIME_STEP = 32
MAX_SPEED=6
FORWARD_SPEED = 3.0  # Speed for moving forward/backward
TURN_SPEED = 1.5     # Speed for turning

WHEEL_RADIUS = 0.03  # meters
AXLE_LENGTH = 0.08   # distance between wheels



UNKNOWN_MARKER = 0
PATH_MARKER = 1
FREE_SPACE_MARKER = 2  # 0 will remain 'unknown'
OBSTACLE_MARKER = 3    # For future use, if you detect obstacles explicitly

# Init robot
robot = Robot()
current_position = [0.58,0.46] # Initial position of the robot in meters
current_encoder_values = [0, 0]
angle=0


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


# Encoders
left_encoder = robot.getDevice("left encoder")
right_encoder = robot.getDevice("right encoder")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)


# IMU
imu = robot.getDevice("Inertial Unit")
imu.enable(TIME_STEP)

# IR Sensors
right_IR=robot.getDevice("rightIR")
left_IR=robot.getDevice("leftIR")
left_IR.enable(TIME_STEP)
right_IR.enable(TIME_STEP)


robot.step(TIME_STEP)

current_encoder_values[0]=left_encoder.getValue()
current_encoder_values[1]=right_encoder.getValue()