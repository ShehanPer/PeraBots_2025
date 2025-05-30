from controller import Robot
import numpy as np
import math
import json


PATH_MARKER = 1
FREE_SPACE_MARKER = 2  # 0 will remain 'unknown'
OBSTACLE_MARKER = 3    # For future use, if you detect obstacles explicitly

SENSOR_OFFSET_SIDEWAYS = 0.02  # 2cm, distance of IR sensor from robot's longitudinal center
SENSOR_MAX_RANGE = 0.35       # Max reliable range to mark as free (e.g., Sharp GP2Y0A41SK0F is 4-30cm, up to 40cm)
                               # Adjust based on your sensor's effective reliable range.




# Constants
TIME_STEP = 32

WHEEL_RADIUS = 0.03  # meters
AXLE_LENGTH = 0.08   # distance between wheels
MAP_RES = 0.02       # 2cmx2cm per cell
MAP_SIZE = 100      # 100x100 grid
MAP= np.zeros((MAP_SIZE, MAP_SIZE), dtype=int)  # Initialize the map


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

# IR Sensors
right_IR=robot.getDevice("rightIR")
left_IR=robot.getDevice("leftIR")
left_IR.enable(TIME_STEP)
right_IR.enable(TIME_STEP)

cam=robot.getDevice("camera")
cam.enable(TIME_STEP)
width = cam.getWidth()
height = cam.getHeight()

def ir_to_distance(value):
    return 0.1594*value**(-0.8533)-0.02916 

def get_pathWidth():
    rightDist = ir_to_distance(right_IR.getValue())
    leftDist = ir_to_distance(left_IR.getValue())
    print("Width : ",leftDist,rightDist)

def calculate_displacement():
    global current_encoder_values,angle
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
    print("ppppppppppppppppppppppppppppppppppppppppppppppppppppppp")
    
    
    
def mark_point_on_map(world_x, world_y, marker_value):
    """Helper function to mark a single point on the global MAP."""
    global MAP, MAP_RES, MAP_SIZE
    
    # Using your established map indexing convention
    map_x_idx = int(world_x / MAP_RES)
    map_y_idx = MAP_SIZE - int(world_y / MAP_RES) # Potential out-of-bounds if world_y is too small

    # Ensure correct handling for 0-indexed array if map_y_idx can be MAP_SIZE
    # A common safe way for this type of inverted Y indexing:
    # map_y_idx = (MAP_SIZE - 1) - int(world_y / MAP_RES)
    # However, to be consistent with your update_map, let's use its exact logic and rely on bounds check.

    if 0 <= map_x_idx < MAP_SIZE and 0 <= map_y_idx < MAP_SIZE:
        # Mark as free space only if currently unknown
        # Don't overwrite the robot's path or already known obstacles
        if MAP[map_y_idx, map_x_idx] == 0: # If unknown
            MAP[map_y_idx, map_x_idx] = marker_value
    # else:
        # print(f"Debug: Point ({world_x:.2f}, {world_y:.2f}) -> ({map_x_idx}, {map_y_idx}) is off map.")


def mark_free_space_along_ray(sensor_world_x, sensor_world_y, ray_world_yaw,
                              measured_distance_from_sensor):
    """Marks cells along the sensor's line of sight as free up to the measured distance."""
    
    # Determine the actual length of the ray to mark as free
    # If measured_distance indicates object is further than SENSOR_MAX_RANGE,
    # we only mark free space up to SENSOR_MAX_RANGE.
    marking_distance = min(measured_distance_from_sensor, SENSOR_MAX_RANGE)

    ray_dir_x = math.cos(ray_world_yaw)
    ray_dir_y = math.sin(ray_world_yaw)

    # Step along the ray to mark cells
    # Start a tiny bit away from the sensor's actual position to avoid marking the sensor's own cell if it's on a boundary.
    # Step size can be MAP_RES or smaller for finer marking.
    step_size = MAP_RES / 2.0 
    current_step_dist = step_size 

    while current_step_dist < marking_distance:
        point_x = sensor_world_x + current_step_dist * ray_dir_x
        point_y = sensor_world_y + current_step_dist * ray_dir_y
        
        mark_point_on_map(point_x, point_y, FREE_SPACE_MARKER)
        
        current_step_dist += step_size

    # Optionally, if an object was detected *within* SENSOR_MAX_RANGE,
    # you could try to mark the cell *at* measured_distance_from_sensor as an obstacle.
    # This is more complex due to sensor noise and beam width.
    # For now, we are just marking the line of sight as free.
    # if measured_distance_from_sensor < SENSOR_MAX_RANGE - (MAP_RES/2): # if obstacle clearly detected
    #     obstacle_x = sensor_world_x + measured_distance_from_sensor * ray_dir_x
    #     obstacle_y = sensor_world_y + measured_distance_from_sensor * ray_dir_y
    #     # Be careful here: convert to int indices and ensure it's a different cell
    #     # mark_point_on_map(obstacle_x, obstacle_y, OBSTACLE_MARKER)
    
    
    
def update_map_with_all_sensors():
    """
    Updates the map with the robot's path and free space from IR sensors.
    This function assumes 'current_position' and 'angle' (robot's yaw) globals are up-to-date.
    """
    global current_position, angle, MAP, left_IR, right_IR # Make sure IR sensors are accessible

    # 1. Mark robot's current path (ensure update_position() has been called before this)
    # This is what your original update_map did. We can call it or replicate its core logic.
    # Let's assume update_map() is called separately to mark the path with PATH_MARKER.
    # If not, you'd add:
    # mark_point_on_map(current_position[0], current_position[1], PATH_MARKER)


    # 2. Process Left IR Sensor
    left_ir_raw_value = left_IR.getValue()
    left_measured_dist = ir_to_distance(left_ir_raw_value) # !! CRITICAL: See warning about this function !!
                                                          # This should return distance in meters.

    # Sensor is on the left, pointing left. Robot frame: X fwd, Y left.
    # Sensor orientation: robot_yaw + PI/2
    # Sensor position relative to robot center (robot frame): (0, SENSOR_OFFSET_SIDEWAYS)
    sensor_yaw_left = angle + (math.pi / 2.0)
    s_lx = current_position[0] - SENSOR_OFFSET_SIDEWAYS * math.sin(angle) # x = r_x - offset_y * sin(yaw)
    s_ly = current_position[1] + SENSOR_OFFSET_SIDEWAYS * math.cos(angle) # y = r_y + offset_y * cos(yaw)
    
    # print(f"Debug L: Raw={left_ir_raw_value:.2f}, Dist={left_measured_dist:.3f}, Pos=({s_lx:.2f},{s_ly:.2f}), Yaw={math.degrees(sensor_yaw_left):.1f}")
    mark_free_space_along_ray(s_lx, s_ly, sensor_yaw_left, left_measured_dist)

    # 3. Process Right IR Sensor
    right_ir_raw_value = right_IR.getValue()
    right_measured_dist = ir_to_distance(right_ir_raw_value) # !! CRITICAL !!

    # Sensor is on the right, pointing right. Robot frame: X fwd, Y left.
    # Sensor orientation: robot_yaw - PI/2
    # Sensor position relative to robot center (robot frame): (0, -SENSOR_OFFSET_SIDEWAYS)
    sensor_yaw_right = angle - (math.pi / 2.0)
    s_rx = current_position[0] - (-SENSOR_OFFSET_SIDEWAYS) * math.sin(angle) # x = r_x - (-offset_y) * sin(yaw)
    s_ry = current_position[1] + (-SENSOR_OFFSET_SIDEWAYS) * math.cos(angle) # y = r_y + (-offset_y) * cos(yaw)

    # print(f"Debug R: Raw={right_ir_raw_value:.2f}, Dist={right_measured_dist:.3f}, Pos=({s_rx:.2f},{s_ry:.2f}), Yaw={math.degrees(sensor_yaw_right):.1f}")
    mark_free_space_along_ray(s_rx, s_ry, sensor_yaw_right, right_measured_dist)
    print("sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss")

def update_map():
    global MAP, current_position
    print("Position : ",current_position)
    x_index = int((current_position[0]) / MAP_RES)
    y_index = MAP_SIZE - int((current_position[1]) / MAP_RES) 
    print("Indexes : ",x_index,y_index)
    for x in range(x_index-2,x_index+3):
        for y in range(y_index-2,y_index+3):
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                MAP[y, x] = 1  # Mark the cell as visited
    
    print("uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu")




def save_map_json(map_array, map_resolution_val, filename="robot_map_custom.json"):
    """
    Saves the map to a JSON file with custom formatting:
    - Outer dictionary keys are indented.
    - Each row in 'map_layout' starts on a new, indented line.
    - Elements within each row are on that single line (compact).

    Args:
        map_array (np.ndarray): The 2D NumPy array for the map.
        map_resolution_val (float): The resolution of the map (e.g., 0.04).
        filename (str): The name of the JSON file to save.
    """
    if not isinstance(map_array, np.ndarray):
        print("Error: map_array must be a NumPy array.")
        return
    if not isinstance(map_resolution_val, (int, float)):
        print(f"Error: map_resolution_val '{map_resolution_val}' must be a number.")
        # Attempt to convert if it's a string representation of a number,
        # otherwise this will cause issues in JSON formatting.
        try:
            map_resolution_val = float(map_resolution_val)
        except ValueError:
            print(f"Critical Error: map_resolution_val '{map_resolution_val}' cannot be converted to a float.")
            return

    map_list_of_lists = map_array.tolist()

    # Define indentation strings
    outer_indent = "  "  # Indentation for top-level keys and closing brace of map_layout
    row_indent = outer_indent * 2 # Indentation for each row string

    # 1. Format each row in map_layout compactly
    formatted_rows = []
    for row in map_list_of_lists:
        # json.dumps for a simple list with compact separators
        row_str_compact = json.dumps(row, separators=(',', ':'))
        formatted_rows.append(row_indent + row_str_compact)

    # 2. Construct the map_layout block string
    if formatted_rows:
        map_layout_block = "[\n" + ",\n".join(formatted_rows) + "\n" + outer_indent + "]"
    else:
        map_layout_block = "[]" # Handle empty map

    # 3. Construct the full JSON string manually
    # We use json.dumps for individual values to ensure correct JSON formatting (e.g., numbers vs strings)
    json_lines = [
        "{",
        outer_indent + f'"map_size": {json.dumps(map_array.shape[0])},',
        outer_indent + f'"map_resolution": {json.dumps(map_resolution_val)},', # map_resolution_val is now a number
        outer_indent + f'"map_layout": {map_layout_block}', # map_layout_block is already a string
        "}"
    ]
    final_json_string = "\n".join(json_lines)

    try:
        with open(filename, 'w') as f:
            f.write(final_json_string)
        print(f"Map successfully saved to {filename} with custom formatting.")
    except IOError as e:
        print(f"Error saving map to {filename}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred during saving: {e}")

# Your existing load function should still work, but let's make it more robust
# to the map_resolution type error you encountered in your file.
def load_map_from_json(filename="robot_map.json"):
    try:
        with open(filename, 'r') as f:
            map_data_loaded = json.load(f)
        
        map_list = map_data_loaded.get("map_layout")
        loaded_map_size = map_data_loaded.get("map_size")
        loaded_map_res_raw = map_data_loaded.get("map_resolution")
        loaded_map_res = None

        if loaded_map_res_raw is not None:
            if isinstance(loaded_map_res_raw, (int, float)):
                loaded_map_res = loaded_map_res_raw
            else:
                print(f"Warning: Loaded map_resolution '{loaded_map_res_raw}' is not a direct number. Attempting conversion.")
                try:
                    loaded_map_res = float(loaded_map_res_raw)
                except ValueError:
                    print(f"Error: Cannot convert loaded map_resolution '{loaded_map_res_raw}' to a float.")
                    # Decide: return None for resolution, raise error, or use a default.
                    # For now, we'll proceed with loaded_map_res as None if conversion fails.
        else:
            print("Warning: 'map_resolution' not found in JSON or is null.")


        if map_list is None:
            print(f"Error: 'map_layout' not found in {filename}.")
            return None, None # Must have map_layout

        map_array = np.array(map_list, dtype=int) 
        
        if loaded_map_size is not None and map_array.shape[0] != loaded_map_size:
            print(f"Warning: Loaded map dimensions {map_array.shape} "
                  f"do not match stored map_size {loaded_map_size}.")

        print(f"Map successfully loaded from {filename}. Resolution: {loaded_map_res}")
        return map_array, loaded_map_res
        
    except FileNotFoundError:
        print(f"Error: Map file {filename} not found.")
        return None, None
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON from {filename}: {e}")
        return None, None
    except Exception as e:
        print(f"An unexpected error occurred during loading: {e}")
        return None, None
TIME_STEP = 32
MAX_SPEED=6
KP = 0.1
KI = 0
KD = 0.001
TARGET_DISTANCE = 60  # Ideal distance sensor reading from right wall

integral=0
prev_error=0
base_speed = 4


def ir_to_distance(value):
    return 0.1594*value**(-0.8533)-0.02916 

def get_pathWidth():
    rightDist = ir_to_distance(right_IR.getValue())
    leftDist = ir_to_distance(left_IR.getValue())
    print("Width : ",leftDist,rightDist)
    return leftDist,rightDist
    
    

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
    blockl=(ind-1)*count
    blockr=0
    redline=0
    areas=[]
    flg=1
    while c<len(arr)-1:
        if (arr[c]==1 or arr[c]==2) and (arr[c+1]==0 or c==len(arr)-2):
            areas.append([ind,(c-ind)])
            if arr[c]==2:redline=True
            if blockr<c+1:blockr=c+1
            flg=1
        elif (arr[c]==1 or arr[c]==2) and flg :
            ind=c+1
            flg=0
        c+=1
    max=0
    point=0
    blockr=blockr*count
    print("blockll",blockl,"blockrr",blockr)
    print(areas)
    print('red',redline)
    for area in areas:
        if max<area[1]:
            max=area[1]
            point=area[0]
            
    center=(point+max/2)*count
    return center,blockl,blockr,redline
def delay(robot, milliseconds):
    steps = milliseconds // (TIME_STEP*1000)
    for _ in range(int(steps)):
        robot.step(TIME_STEP)

# Example: delay for 1 second (1000 ms)

def testRun():
    state1=[]
    state2=[]
    state3=[]
    global integral,prev_error
    image=cam.getImage()
    LIR,RIR=get_pathWidth()
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
    vertical_line1 = img[110,:]
    vertical_line2 = img[70,:]
    vertical_line3 = img[20,:]
    state3,l,r,red= is_white(vertical_line3)
    state2= is_white(vertical_line2)
    state3,l,r,red= is_white(vertical_line1)
    center=state2[0]
    flgred=1
    print('reddddd',red,state2)
    if red:
        flgred=0
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        
        return 0
    print("center",center)
    #if center <center2:center=center2
    #if center>center3:center=center3
    if RIR< 0.16 and LIR <0.2:
        error = .1-RIR
        print('errorrs',error,RIR)
        KP = 10
        KI = 0
        KD = 0.03
    elif LIR< 0.16 and RIR <0.2:
        error = 0.1- LIR
        print('errorls',error)
        KP = 10
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
    return 1
flg=1
while robot.step(TIME_STEP) != -1:
    update_position()
    update_map_with_all_sensors()
    update_map()
    if testRun():
        pass
    elif flg :
        flg=0
        print(f"Sim time: {robot.getTime():.2f}s - Saving map...")
        # Ensure MAP_RES (from matrix_map.py) is accessible or pass it
        save_map_json(MAP, MAP_RES, f"robot_map1_2cm.json")

