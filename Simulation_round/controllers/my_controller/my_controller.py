from controller import Robot 
import numpy as np
import cv2
import time
import math


robot = Robot()
timestep = int(robot.getBasicTimeStep())
print("Controller started successfully!")

# Robot physical properties
WHEEL_RADIUS = 0.03  # meters (from AutoNova.proto)
AXLE_LENGTH = 0.08   # meters (distance between wheels, from AutoNova.proto)

# Robot position tracking (start in middle of grid)
robot_x = 0
robot_y = 0
robot_theta = 0  # radians, 0 is facing positive x direction

# Initialize encoders for odometry
prev_left_encoder = 0
prev_right_encoder = 0

# Display settings
VERBOSE_OUTPUT = False  # Set to False to limit terminal output


def get_device(device_name):
    device = robot.getDevice(device_name)
    if device:
        device.enable(timestep)
    return device


L_motor = robot.getDevice('left motor')
R_motor = robot.getDevice('right motor')

L_encoder = get_device('left encoder')
R_encoder = get_device('right encoder')

# Make sure encoders are properly initialized
if L_encoder and R_encoder:
    print("Encoders initialized successfully")
else:
    print("WARNING: One or both encoders not found!")

# Configure motors for velocity control
L_motor.setPosition(float('inf'))  # Set to infinite position for velocity control
R_motor.setPosition(float('inf'))  # Set to infinite position for velocity control


camera=get_device('camera')

gyro = get_device('gyroScope')

# Try to get GPS if available (for more accurate positioning)
gps = get_device('GPS')

us1,us2,us3,us4 = [get_device(name) for name in ['ps8', 'ps0', 'ps2', 'ps4']]

def use_camera(cam):
    image = cam.getImage()
    width = cam.getWidth()
    height = cam.getHeight()

    image_array = np.frombuffer(image,dtype=np.uint8).reshape((height,width,4))
    img_rgb = image_array[:,:,:3]
   
    return img_rgb

def control_with_keyboard(key, speed=2.0, turn_speed=1.0):
    """
    Control robot with keyboard inputs like a car racing game
    
    Arguments:
        key: The key pressed (from cv2.waitKey)
        speed: Forward/backward speed (default: 2.0)
        turn_speed: Turning speed (default: 1.0)
    """
    if key == ord('w'):  # Move forward
        L_motor.setVelocity(speed)
        R_motor.setVelocity(speed)
        if VERBOSE_OUTPUT:
            print("Moving forward")
    elif key == ord('s'):  # Move backward
        L_motor.setVelocity(-speed)
        R_motor.setVelocity(-speed)
        if VERBOSE_OUTPUT:
            print("Moving backward")
    elif key == ord('a'):  # Turn left
        L_motor.setVelocity(-turn_speed)
        R_motor.setVelocity(turn_speed)
        if VERBOSE_OUTPUT:
            print("Turning left")
    elif key == ord('d'):  # Turn right
        L_motor.setVelocity(turn_speed)
        R_motor.setVelocity(-turn_speed)
        if VERBOSE_OUTPUT:
            print("Turning right")
    elif key == ord('q'):  # Left forward arc
        L_motor.setVelocity(speed * 0.5)
        R_motor.setVelocity(speed)
        if VERBOSE_OUTPUT:
            print("Forward left arc")
    elif key == ord('e'):  # Right forward arc
        L_motor.setVelocity(speed)
        R_motor.setVelocity(speed * 0.5)
        if VERBOSE_OUTPUT:
            print("Forward right arc")
    elif key == ord('z'):  # Left backward arc
        L_motor.setVelocity(-speed * 0.5)
        R_motor.setVelocity(-speed)
        if VERBOSE_OUTPUT:
            print("Backward left arc")
    elif key == ord('c'):  # Right backward arc
        L_motor.setVelocity(-speed)
        R_motor.setVelocity(-speed * 0.5)
        if VERBOSE_OUTPUT:
            print("Backward right arc")
    elif key == ord(' '):  # Space bar - emergency stop
        L_motor.setVelocity(0.0)
        R_motor.setVelocity(0.0)
        print("Emergency stop!")
    else:
        # Default to gradual stop if no key is pressed
        current_left = L_motor.getVelocity()
        current_right = R_motor.getVelocity()
        L_motor.setVelocity(current_left * 0.8)  # Gradual slowdown
        R_motor.setVelocity(current_right * 0.8)  # Gradual slowdown


def read_ultrasonic_distances():
    """
    Read ultrasonic sensor values and convert them to real distances in centimeters
    Returns a dictionary with sensor names and their distances
    """
    # Get raw values from sensors
    left_raw = us2.getValue()
    right_raw = us4.getValue()
    back_raw = us3.getValue()
    front_raw = us1.getValue()
    
   
    conversion_factor = 1000.0  # This value needs calibration for your specific sensors
    
    front_dist = front_raw / conversion_factor * 100.0  # Convert to cm
    left_dist = left_raw / conversion_factor * 100.0
    right_dist = right_raw / conversion_factor * 100.0
    back_dist = back_raw / conversion_factor * 100.0
    
    # Create a dictionary with all distances
    distances = {
        "front": front_dist,
        "left": left_dist,
        "right": right_dist,
        "back": back_dist
    }
    
    # Print the distances
    if VERBOSE_OUTPUT:
        print(f"Distances (cm): Front: {front_dist:.1f}, Left: {left_dist:.1f}, Right: {right_dist:.1f}, Back: {back_dist:.1f}")
    
    return distances

def map_edges_to_distance_array(edges):
    """map edges from bottum middle point of the frame and return matrix of distances"""   
    height, width = edges.shape
    middle_x = width // 2
    distances = np.zeros((height,width), dtype=np.float32)
    
    #take each edge point and map in to same x and change y regarding to distance
    for y in range(height):
        for x in range(width):
            if edges[y, x] > 0:  # Edge detected
                distance = height - y  # Distance from bottom to edge
                distances[y, x] = distance
    
    return distances

def update_position_from_encoders():
    """
    Update robot position and orientation based on encoder values.
    Simplified version without grid tracking.
    """
    global robot_x, robot_y, robot_theta, prev_left_encoder, prev_right_encoder
    
    # Get current encoder values
    left_encoder_val = L_encoder.getValue()
    right_encoder_val = R_encoder.getValue()
    
    # Calculate change in encoder values
    left_delta = left_encoder_val - prev_left_encoder
    right_delta = right_encoder_val - prev_right_encoder
    
    # Update previous encoder values
    prev_left_encoder = left_encoder_val
    prev_right_encoder = right_encoder_val
    
    # Calculate distance traveled by each wheel
    left_distance = left_delta * WHEEL_RADIUS  # meters
    right_distance = right_delta * WHEEL_RADIUS  # meters
    
    # Calculate average distance traveled
    distance = (left_distance + right_distance) / 2.0
    
    # Calculate change in orientation (based on differential drive kinematics)
    orientation_change = (right_distance - left_distance) / AXLE_LENGTH
    
    # Update robot orientation
    robot_theta += orientation_change
    
    # Normalize angle to [-π, π]
    robot_theta = math.atan2(math.sin(robot_theta), math.cos(robot_theta))
    
    # Update robot position in real-world coordinates (meters)
    real_x_change = distance * math.cos(robot_theta)
    real_y_change = distance * math.sin(robot_theta)
    
    # Update position
    robot_x += real_x_change
    robot_y += real_y_change
    
    # Print current position
    print(f"Robot position: ({robot_x:.2f}, {robot_y:.2f}) m | Orientation: {math.degrees(robot_theta):.1f}°")
    
    return robot_x, robot_y
    
# Main control loop
while robot.step(timestep) != -1:
    # Process camera image
    frame = use_camera(camera)
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (5, 5), 0)
    edges = cv2.Canny(img_gray, 50, 150, apertureSize=3)
    
    # Update robot position
    current_x, current_y = update_position_from_encoders()
    
    # Make a copy of the frame that can be modified
    display_frame = frame.copy()
    
    # Display current position on the image
    position_text = f"Position: ({current_x:.2f}, {current_y:.2f}) m"
    cv2.putText(display_frame, position_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Process keyboard input
    key = cv2.waitKey(1) & 0xFF
    control_with_keyboard(key)
 
    # Show camera views
    cv2.imshow("Robot Camera View", edges)
    cv2.imshow("Camera Feed", display_frame)
    
    # Read sensor distances if needed
    # distances = read_ultrasonic_distances()
    
    # Exit on 'q' key press
    if key == ord('q'):
        break
    
    # Uncomment this to also use autonomous movement
    # run()