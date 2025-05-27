from controller import Robot
import numpy as np
import math
import json


# Constants
TIME_STEP = 32

WHEEL_RADIUS = 0.03  # meters
AXLE_LENGTH = 0.08   # distance between wheels
MAP_RES = 0.04       # 4cmx4cm per cell
MAP_SIZE = 50      # 100x100 grid
MAP= np.zeros((MAP_SIZE, MAP_SIZE), dtype=int)  # Initialize the map


# Init robot
robot = Robot()
current_position = [0.58,0.46] # Initial position of the robot in meters
current_encoder_values = [0, 0]
angle=0


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



def update_map():
    global MAP, current_position
    print("Position : ",current_position)
    x_index = int((current_position[0]) / MAP_RES)
    y_index = MAP_SIZE - int((current_position[1]) / MAP_RES) 
    print("Indexes : ",x_index,y_index)
    if 0 <= x_index < MAP_SIZE and 0 <= y_index < MAP_SIZE:
        MAP[y_index, x_index] = 1  # Mark the cell as visited


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