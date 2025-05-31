import json
import numpy as np
from PIL import Image,ImageDraw # Import the Pillow library
from map_json import load_map_from_json


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
            
            if map_value >= 99: # the path of the centerline spine
                color = ((map_value-100)%255, map_value%255, (map_value+100)%255)

            else:
                color = COLOR_MAP.get(map_value, DEFAULT_COLOR)

            # Calculate the pixel coordinates for the top-left corner of the cell block
            x0 = c_cell * cell_pixel_size
            y0 = r_cell * cell_pixel_size
            # Calculate the bottom-right corner
            x1 = x0 + cell_pixel_size -1 # If cell_pixel_size is 1, x1=x0
            y1 = y0 + cell_pixel_size -1 # If cell_pixel_size is 1, y1=y0
            
            rect_x1 = x0 + cell_pixel_size
            rect_y1 = y0 + cell_pixel_size
            
            draw_context.rectangle([x0, y0, rect_x1, rect_y1], fill=color)
            


    try:
        image.save(output_image_filename)
        print(f"Map visualization saved to {output_image_filename}")
        print(f"Image dimensions: {img_width}x{img_height} pixels.")
    except IOError as e:
        print(f"Error saving image to {output_image_filename}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred while saving image: {e}")

# --- Main part of the script ---
def visualize_map(map_json_path):
    # Path to saved JSON map file
    json_map_file_path = map_json_path # Make sure this file is in the same directory 
    
    # Desired name for the output image
    output_image_file_path = "robot_map.png"

    pixels_per_cell = 20        # How many pixels to use for each map cell in the output image


    print(f"Attempting to load map from: {json_map_file_path}")
    map_data, map_res = load_map_from_json(json_map_file_path)

    if map_data is not None:
        print(f"Map loaded. Shape: {map_data.shape}. Resolution: {map_res if map_res is not None else 'N/A'}")
        create_map_image(map_data, output_image_file_path, cell_pixel_size=pixels_per_cell)
    else:
        print("Could not load map data to visualize.")