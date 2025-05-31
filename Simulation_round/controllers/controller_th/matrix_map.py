import numpy as np
import math

from config import *
from map_json import *


SENSOR_OFFSET_SIDEWAYS = 0.02  # 2cm, distance of IR sensor from robot's longitudinal center
SENSOR_MAX_RANGE = 0.40       # Max reliable range to mark as free 


MAP_RES = 0.02       # 2cmx2cm per cell
MAP_SIZE = 100      # 100x100 grid
MAP= np.zeros((MAP_SIZE, MAP_SIZE), dtype=int)  # Initialize the map



def ir_to_distance(value):
    return 0.1594*value**(-0.8533)-0.02916 


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
    
    
    
def mark_point_on_map(world_x, world_y, marker_value):
    """Helper function to mark a single point on the global MAP."""
    global MAP, MAP_RES, MAP_SIZE
    
    # Using your established map indexing convention
    map_x_idx = int(world_x / MAP_RES)
    map_y_idx = MAP_SIZE - int(world_y / MAP_RES) # Using inverted Y-indexing



    if 0 <= map_x_idx < MAP_SIZE and 0 <= map_y_idx < MAP_SIZE:
        # Mark as free space only if currently unknown
        if MAP[map_y_idx, map_x_idx] == 0: # If unknown
            MAP[map_y_idx, map_x_idx] = marker_value


def mark_free_space_along_ray(sensor_world_x, sensor_world_y, ray_world_yaw,
                              measured_distance_from_sensor):
    """Marks cells along the sensor's line of sight as free up to the measured distance."""
    
    # Determine the actual length of the ray to mark as free. Capped at sensor max range
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


    
def mark_robot():
    global MAP, current_position
    # print("Position : ",current_position)
    x_index = int((current_position[0]) / MAP_RES)
    y_index = MAP_SIZE - int((current_position[1]) / MAP_RES) 
    # print("Indexes : ",x_index,y_index)
    for x in range(x_index-1,x_index+2):
        for y in range(y_index-1,y_index+2):
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                MAP[y, x] = 1  # Mark the cell as free space 
    
def update_map():
    """
    Updates the map with the robot's path and free space from IR sensors.
    This function assumes 'current_position' and 'angle' (robot's yaw) globals are up-to-date.
    """
    global current_position, angle, MAP, left_IR, right_IR # Make sure IR sensors are accessible

    
    # 1. Process Left IR Sensor
    left_ir_raw_value = left_IR.getValue()
    left_measured_dist = ir_to_distance(left_ir_raw_value) 

    
    sensor_yaw_left = angle + (math.pi / 2.0)  # Sensor orientation: robot_yaw + PI/2
    s_lx = current_position[0] - SENSOR_OFFSET_SIDEWAYS * math.sin(angle) # x = r_x - offset_y * sin(yaw)
    s_ly = current_position[1] + SENSOR_OFFSET_SIDEWAYS * math.cos(angle) # y = r_y + offset_y * cos(yaw)
    
    mark_free_space_along_ray(s_lx, s_ly, sensor_yaw_left, left_measured_dist)

    # 2. Process Right IR Sensor
    right_ir_raw_value = right_IR.getValue()
    right_measured_dist = ir_to_distance(right_ir_raw_value)


    sensor_yaw_right = angle - (math.pi / 2.0)  # Sensor orientation: robot_yaw - PI/2
    s_rx = current_position[0] - (-SENSOR_OFFSET_SIDEWAYS) * math.sin(angle) # x = r_x - (-offset_y) * sin(yaw)
    s_ry = current_position[1] + (-SENSOR_OFFSET_SIDEWAYS) * math.cos(angle) # y = r_y + (-offset_y) * cos(yaw)

    mark_free_space_along_ray(s_rx, s_ry, sensor_yaw_right, right_measured_dist) 

    # 3. Mark robot's current path 
    mark_robot() 
    


    

