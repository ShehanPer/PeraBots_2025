import json
import numpy as np
from PIL import Image,ImageDraw # Import the Pillow library

# --- Configuration ---
# Define the mapping from your map values to RGB colors
# You can customize these colors as you like.
COLOR_MAP = {
    0: (220, 220, 220),  # Light Gray for Unknown/Empty (value 0)
    1: (0, 0, 255),      # Blue for Robot Path (value 1)
    2: (0, 255, 0),      # Green for Observed Free Space (value 2)
    # You can add more mappings if you use other values, e.g.:
    # 3: (255, 0, 0),      # Red for Obstacles (if you add value 3 later)
}
DEFAULT_COLOR = (0, 0, 0) # Black for any unexpected map values

# --- Function to load the map from JSON ---
# This function is based on our previous discussions.
def load_map_from_json(filename="robot_map.json"):
    """
    Loads a map from a JSON file and returns the map array and its resolution.
    """
    try:
        with open(filename, 'r') as f:
            map_data_loaded = json.load(f)
        
        map_list = map_data_loaded.get("map_layout")
        # loaded_map_size = map_data_loaded.get("map_size") # Can be used for validation
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
        else:
            print("Warning: 'map_resolution' not found in JSON or is null.")

        if map_list is None:
            print(f"Error: 'map_layout' not found in {filename}.")
            return None, None

        map_array = np.array(map_list, dtype=int) # Assuming map values are integers
        
        print(f"Map successfully loaded from {filename}. Resolution: {loaded_map_res}")
        return map_array, loaded_map_res
        
    except FileNotFoundError:
        print(f"Error: Map file '{filename}' not found.")
        return None, None
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON from {filename}: {e}")
        return None, None
    except Exception as e:
        print(f"An unexpected error occurred during loading: {e}")
        return None, None

# --- Function to create and save the map visualization ---
def create_map_image(map_array, output_image_filename="map_visualization.png", cell_pixel_size=5):
    """
    Creates an image from the map array and saves it.
    Each cell in the map can be rendered as a block of pixels.

    Args:
        map_array (np.ndarray): The 2D NumPy array representing the map.
        output_image_filename (str): The filename for the output image (e.g., "map.png").
        cell_pixel_size (int): The size (in pixels) for each map cell in the image.
    """
    if map_array is None:
        print("Error: No map array provided for visualization.")
        return

    map_height_cells, map_width_cells = map_array.shape
    if map_height_cells == 0 or map_width_cells == 0:
        print("Error: Map array is empty.")
        return

    # Calculate image dimensions based on cell size
    img_width = map_width_cells * cell_pixel_size
    img_height = map_height_cells * cell_pixel_size

    # Create a new RGB image
    image = Image.new("RGB", (img_width, img_height))
    pixels = image.load() # Allows direct pixel manipulation (though slower for blocks)

    # Efficiently draw colored blocks for each cell
    draw_context = ImageDraw.Draw(image)

    for r_cell in range(map_height_cells):  # r_cell is row index in map_array
        for c_cell in range(map_width_cells): # c_cell is column index in map_array
            map_value = map_array[r_cell, c_cell]
            color = COLOR_MAP.get(map_value, DEFAULT_COLOR)

            # Calculate the pixel coordinates for the top-left corner of the cell block
            x0 = c_cell * cell_pixel_size
            y0 = r_cell * cell_pixel_size
            # Calculate the bottom-right corner
            x1 = x0 + cell_pixel_size -1 # If cell_pixel_size is 1, x1=x0
            y1 = y0 + cell_pixel_size -1 # If cell_pixel_size is 1, y1=y0
            
            # For cell_pixel_size > 1, ensure x1, y1 correctly define the rectangle.
            # ImageDraw.rectangle takes [x0, y0, x1, y1] where x1, y1 are outside the rectangle
            # if we want a filled block of size cell_pixel_size x cell_pixel_size.
            # Or more simply: (x0,y0) to (x0+size-1, y0+size-1) are the pixels.
            # So the rectangle for fill is (x0, y0) to (x0+size, y0+size) exclusive for end.
            rect_x1 = x0 + cell_pixel_size
            rect_y1 = y0 + cell_pixel_size
            
            draw_context.rectangle([x0, y0, rect_x1, rect_y1], fill=color)
            
            # If cell_pixel_size is 1, this direct pixel access is fine too:
            # if cell_pixel_size == 1:
            #    pixels[c_cell, r_cell] = color
            # else: (handled by draw_context.rectangle)


    try:
        image.save(output_image_filename)
        print(f"Map visualization saved to {output_image_filename}")
        print(f"Image dimensions: {img_width}x{img_height} pixels.")
    except IOError as e:
        print(f"Error saving image to {output_image_filename}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred while saving image: {e}")

# --- Main part of the script ---
if __name__ == "__main__":
    # Path to your saved JSON map file (the one you uploaded)
    json_map_file_path = "E:/Projects/PeraBots_2025/Simulation_round/controllers/my_controller/robot_map_final.json" # Make sure this file is in the same directory or provide full path
    # Desired name for the output image
    output_image_file_path = "map_generated_final.png"

    # How many pixels to use for each map cell in the output image
    # If your map is 50x50:
    # - cell_pixel_size = 1 will result in a 50x50 pixel image (can be very small)
    # - cell_pixel_size = 10 will result in a 500x500 pixel image (much clearer)
    pixels_per_cell = 20

    print(f"Attempting to load map from: {json_map_file_path}")
    map_data, map_res = load_map_from_json(json_map_file_path)

    if map_data is not None:
        print(f"Map loaded. Shape: {map_data.shape}. Resolution: {map_res if map_res is not None else 'N/A'}")
        create_map_image(map_data, output_image_file_path, cell_pixel_size=pixels_per_cell)
    else:
        print("Could not load map data to visualize.")