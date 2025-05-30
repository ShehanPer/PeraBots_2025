import numpy as np
import json
import math
from skimage.morphology import skeletonize
# from scipy.interpolate import splprep, splev # Optional for advanced smoothing

from config import *
from matrix_map import save_map_json


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




# Attempt to import shared functions and constants from matrix_map.py
# If matrix_map.py is in the same directory or Python path.
try:
    from matrix_map import load_map_from_json, save_map_json, \
                           PATH_MARKER, FREE_SPACE_MARKER, UNKNOWN_MARKER
except ImportError:
    print("Warning: Could not import from matrix_map.py. Ensure it's in the Python path.")
    print("Defining constants and I/O functions locally for path_planner.py.")

    # --- Fallback definitions if import fails ---
    PATH_MARKER = 1
    FREE_SPACE_MARKER = 2
    UNKNOWN_MARKER = 0

    def save_map_json(map_array, map_resolution_val, filename="robot_map_custom.json"):
        # (Copy the save_map_json function with custom formatting from your matrix_map.py here)
        # For brevity, I'm omitting the full function here.
        # Ensure it's the version that produces the desired custom-formatted JSON.
        if not isinstance(map_array, np.ndarray): return
        if not isinstance(map_resolution_val, (int, float)):
            try: map_resolution_val = float(map_resolution_val)
            except ValueError: print("Critical Error: map_resolution_val invalid."); return
        map_list_of_lists = map_array.tolist()
        outer_indent = "  "; row_indent = outer_indent * 2
        formatted_rows = [row_indent + json.dumps(row, separators=(',', ':')) for row in map_list_of_lists]
        map_layout_block = "[\n" + ",\n".join(formatted_rows) + "\n" + outer_indent + "]" if formatted_rows else "[]"
        json_lines = ["{",
                      outer_indent + f'"map_size": {json.dumps(map_array.shape[0])},',
                      outer_indent + f'"map_resolution": {json.dumps(map_resolution_val)},',
                      outer_indent + f'"map_layout": {map_layout_block}',"}"]
        final_json_string = "\n".join(json_lines)
        try:
            with open(filename, 'w') as f: f.write(final_json_string)
            print(f"Map successfully saved to {filename} (custom format).")
        except Exception as e: print(f"Fallback save_map_json error: {e}")
    # --- End of Fallback definitions ---

SKELETON_PATH_WAYPOINT_START = 100  # Start numbering for the planned path cells

def find_neighbors(r, c, shape):
    """ Helper to find 8-connectivity neighbors within bounds """
    neighbors = []
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if dr == 0 and dc == 0:
                continue
            nr, nc = r + dr, c + dc
            if 0 <= nr < shape[0] and 0 <= nc < shape[1]:
                neighbors.append((nr, nc))
    return neighbors

def find_centerline_path(current_map_array, path_marker_val, free_space_marker_val):
    """
    Finds a centerline path using skeletonization and attempts to order it.
    Returns an ordered list of (row, col) tuples.
    """
    print("Starting centerline path finding...")
    traversable_map = np.zeros_like(current_map_array, dtype=bool)
    traversable_map[(current_map_array == path_marker_val) | (current_map_array == free_space_marker_val)] = True

    if not np.any(traversable_map):
        print("No traversable space found for skeletonization.")
        return []

    skeleton = skeletonize(traversable_map)
    print(f"Skeletonization complete. Found {np.sum(skeleton)} skeleton pixels.")

    if not np.any(skeleton):
        print("Skeletonization resulted in an empty path.")
        return []

    skeleton_pixels_coords = np.argwhere(skeleton)
    if len(skeleton_pixels_coords) == 0:
        return []

    # Build adjacency list for skeleton pixels
    adj = {tuple(p): [] for p in skeleton_pixels_coords}
    pixel_set = set(map(tuple, skeleton_pixels_coords)) # For quick lookups

    for r, c in skeleton_pixels_coords:
        for nr, nc in find_neighbors(r, c, skeleton.shape):
            if (nr, nc) in pixel_set:
                adj[(r, c)].append((nr, nc))

    # Attempt to find a long, ordered path using DFS
    # This is a basic DFS; for complex tracks, more sophisticated graph traversal might be needed
    # (e.g., finding endpoints, handling junctions more gracefully for loops)
    
    start_node = tuple(skeleton_pixels_coords[0]) # Arbitrary start
    ordered_path = []
    stack = [(start_node, [start_node])] # (current_node, path_so_far)
    visited_dfs = {start_node} # Keep track of visited nodes in the current DFS path search
                               # to avoid trivial cycles within a single DFS exploration.
                               # For global "longest path", this might need adjustment.

    # We're looking for one continuous path. If the skeleton is a clean loop or line,
    # a simple DFS should trace it.
    
    # Simpler tracing for a single continuous path/loop:
    ordered_path_trace = []
    q = [start_node] # Queue for BFS-like neighbor finding, or stack for DFS
    visited_trace = {start_node}
    ordered_path_trace.append(start_node)

    current_trace_point = start_node
    while True:
        found_next_in_trace = False
        # Find an unvisited neighbor of the current_trace_point
        # Prefer neighbors that maintain a "straighter" line if possible (more complex)
        # For now, just take any unvisited neighbor
        
        # Get neighbors of current_trace_point from precomputed adjacency list
        # Sort neighbors to have some deterministic behavior if multiple choices (optional)
        # neighbors_of_current = sorted(adj[current_trace_point]) 

        unvisited_neighbors = [n for n in adj[current_trace_point] if n not in visited_trace]

        if unvisited_neighbors:
            # Simple strategy: pick the first unvisited neighbor
            # For a cleaner path, you might sort neighbors or use heuristics
            next_node = unvisited_neighbors[0]
            ordered_path_trace.append(next_node)
            visited_trace.add(next_node)
            current_trace_point = next_node
            found_next_in_trace = True
        
        if not found_next_in_trace:
            # No more unvisited direct neighbors from current_trace_point
            # This could be an endpoint, or we are stuck if it's a complex graph.
            # If len(visited_trace) < len(pixel_set), there are other components or we need to backtrack (full DFS)
            # For a single loop/path, this simple trace might be okay.
            break 
            
    # If the trace didn't visit all skeleton pixels (e.g. branches), it's incomplete.
    # The definition of "optimal" is key. If it's just "a centerline", this is one.
    if len(ordered_path_trace) < len(pixel_set) * 0.8: # Heuristic: if we missed a lot
        print(f"Warning: DFS trace might be incomplete. Visited {len(ordered_path_trace)} of {len(pixel_set)} skeleton pixels.")
        print("Using the longest found continuous segment. For complex tracks, enhance path ordering.")
        # Fallback to the raw (unordered) list if trace is too short and raw list is better.
        # This part depends on how critical perfect ordering vs. any centerline is.
        # For now, we'll use what ordered_path_trace found.
    
    print(f"Path ordering complete. Generated {len(ordered_path_trace)} waypoints.")
    return ordered_path_trace


def mark_ordered_path(map_to_modify, ordered_waypoints, start_value):
    if not ordered_waypoints:
        print("No waypoints to mark.")
        return
    for i, (r, c) in enumerate(ordered_waypoints):
        if 0 <= r < map_to_modify.shape[0] and 0 <= c < map_to_modify.shape[1]:
            map_to_modify[r, c] = start_value + i
    print(f"Marked {len(ordered_waypoints)} waypoints, starting from value {start_value}.")


# --- Main execution block for the path planner ---
if __name__ == "__main__":
    # Input: The JSON map file saved by your robot controller
    # This should be a map populated with 0s, 1s (path), and 2s (free space)
    input_json_map_file = "robot_map_2cm.json" # Or "robot_map_final_explored.json"
                                                # Or "robot_map_sensor.json" from your upload

    # Output files
    output_map_with_path_file = "map_with_planned_centerline.json"
    output_waypoints_file = "centerline_waypoints_ordered.json"

    print(f"Loading map from: {input_json_map_file}")
    map_data_array, map_resolution = load_map_from_json(input_json_map_file)

    if map_data_array is not None and map_resolution is not None:
        print(f"Map loaded. Shape: {map_data_array.shape}, Resolution: {map_resolution}")

        # Find the centerline path
        # These marker values should match how your map is generated.
        centerline_pts = find_centerline_path(map_data_array, 
                                              path_marker_val=PATH_MARKER, 
                                              free_space_marker_val=FREE_SPACE_MARKER)

        if centerline_pts:
            map_with_centerline = np.copy(map_data_array)
            mark_ordered_path(map_with_centerline, centerline_pts, SKELETON_PATH_WAYPOINT_START)
            
            # Save the map with the visualized ordered path
            save_map_json(map_with_centerline, map_resolution, output_map_with_path_file)

            # Save the ordered waypoints (list of [row, col])
            waypoints_data_to_save = {
                "map_resolution": map_resolution,
                "map_size_cells": list(map_data_array.shape),
                "path_start_value_in_map": SKELETON_PATH_WAYPOINT_START,
                "ordered_waypoints_rc": centerline_pts # [row, column]
            }
            try:
                with open(output_waypoints_file, "w") as f_wp:
                    json.dump(waypoints_data_to_save, f_wp, indent=2)
                print(f"Ordered waypoints saved to {output_waypoints_file}")
            except Exception as e:
                print(f"Error saving ordered waypoints: {e}")
            
            print("\nPath planning complete. You can now visualize:")
            print(f" - Map with numbered path: {output_map_with_path_file}")
            print(" - Waypoint coordinates: " + output_waypoints_file)
            print("Remember to update your visualize.py COLOR_MAP if needed for new marker values.")

        else:
            print("Failed to generate a centerline path.")
    else:
        print(f"Failed to load map from {input_json_map_file}. Path planning cannot proceed.")