import numpy as np
import math
from skimage.morphology import skeletonize
# from scipy.interpolate import splprep, splev # Optional for advanced smoothing

from config import *
from map_json import *



SKELETON_PATH_WAYPOINT_START = 100  # Start numbering for the planned path cells

def find_neighbors_skel(r, c, skeleton_array):
    """ Helper to find 8-connectivity neighbors that are part of the skeleton. """
    neighbors = []
    shape = skeleton_array.shape
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if dr == 0 and dc == 0:
                continue
            nr, nc = r + dr, c + dc
            if 0 <= nr < shape[0] and 0 <= nc < shape[1] and skeleton_array[nr, nc]:
                neighbors.append((nr, nc))
    return neighbors


def find_centerline_path(current_map_array, path_marker_val, free_space_marker_val, 
                         preferred_start_rc): 
    """
    Finds a centerline path using skeletonization and improved ordering.
    Tries to use preferred_start_rc if provided and valid.

    Args:
        current_map_array (np.ndarray): The input map.
        path_marker_val (int): Value for robot's traversed path.
        free_space_marker_val (int): Value for confirmed free space.
        preferred_start_rc (tuple, optional): Preferred (row, col) to start path ordering.

    Returns:
        list: An ordered list of (row, col) tuples representing the centerline path.
    """
    print("Starting centerline path finding...")
    traversable_map = np.zeros_like(current_map_array, dtype=bool)
    traversable_map[(current_map_array == path_marker_val) | (current_map_array == free_space_marker_val)] = True

    if not np.any(traversable_map):
        print("No traversable space found for skeletonization.")
        return []

    skeleton = skeletonize(traversable_map)
    total_skeleton_pixels = np.sum(skeleton)
    print(f"Skeletonization complete. Found {total_skeleton_pixels} skeleton pixels.")

    if total_skeleton_pixels == 0:
        print("Skeletonization resulted in an empty path.")
        return []

    skeleton_pixels_coords = np.argwhere(skeleton)
    
    # --- Find Connected Components and select the largest one ---
    nodes = {tuple(p) for p in skeleton_pixels_coords} # Use a set for efficient lookup
    visited_for_components = set()
    all_components = []

    for r_node_start, c_node_start in nodes: 
        start_node_comp = (r_node_start, c_node_start)
        if start_node_comp not in visited_for_components:
            current_component_nodes = []
            q_comp = [start_node_comp]
            visited_bfs_component = {start_node_comp}
            
            head = 0
            while head < len(q_comp):
                curr_comp_node = q_comp[head]; head += 1
                current_component_nodes.append(curr_comp_node)
                visited_for_components.add(curr_comp_node)
                for neighbor in find_neighbors_skel(curr_comp_node[0], curr_comp_node[1], skeleton):
                    if neighbor not in visited_bfs_component:
                        visited_bfs_component.add(neighbor)
                        q_comp.append(neighbor)
            all_components.append(current_component_nodes)
            
    if not all_components:
        print("No connected components found in the skeleton.")
        return []

    largest_component_list = max(all_components, key=len)
    largest_component_set = set(largest_component_list) # For efficient "in" check
    print(f"Found {len(all_components)} skeleton component(s). Largest has {len(largest_component_list)} pixels.")
    
    if not largest_component_list:
        return []

    # --- Determine the starting node for DFS trace ---
    start_node_trace = None
    if preferred_start_rc in largest_component_set:
        start_node_trace = preferred_start_rc
        print(f"Using preferred start point: {start_node_trace}")
    else:
        print(f"Warning: Preferred start point {preferred_start_rc} is not on the largest skeleton component. Finding an alternative start.")

    if start_node_trace is None: # If preferred start not used or not provided
        # Fallback: try to find a point with fewer connections or just the first point
        min_degree = float('inf')
        # Build adjacency list only for the largest component for degree calculation
        adj_largest_comp_temp = {node: [] for node in largest_component_list}
        for r_node, c_node in largest_component_list:
            for nr, nc in find_neighbors_skel(r_node, c_node, skeleton):
                if (nr,nc) in largest_component_set:
                     adj_largest_comp_temp[(r_node, c_node)].append((nr, nc))

        for node in largest_component_list:
            degree = len(adj_largest_comp_temp.get(node,[]))
            if degree > 0 and degree < min_degree : 
                min_degree = degree
                start_node_trace = node
            elif start_node_trace is None and degree > 0: # First valid node if all have same min_degree > 0
                 start_node_trace = node
        
        if start_node_trace is None: # Should not happen if largest_component_list is not empty
            start_node_trace = largest_component_list[0]
        print(f"Using automatically selected start point: {start_node_trace}")


    # --- Perform DFS trace on the largest component from the chosen start_node_trace ---
    adj_largest_comp = {node: [] for node in largest_component_list}
    for r_node, c_node in largest_component_list:
        for nr, nc in find_neighbors_skel(r_node, c_node, skeleton):
            if (nr,nc) in largest_component_set:
                 adj_largest_comp[(r_node, c_node)].append((nr, nc))

    ordered_path = []
    stack = [start_node_trace]
    visited_in_trace = set()

    while stack:
        current_node = stack.pop()
        if current_node in visited_in_trace:
            continue
        visited_in_trace.add(current_node)
        ordered_path.append(current_node)
        
        # Add unvisited neighbors from the largest component to the stack.
        # Sort for some determinism, reverse=True often explores one branch "fully".
        neighbors_to_visit = sorted(
            [n for n in adj_largest_comp.get(current_node, []) if n not in visited_in_trace],
            reverse=True 
        )
        for neighbor in neighbors_to_visit:
            stack.append(neighbor)
            
    if len(ordered_path) != len(largest_component_list):
        print(f"Warning: Path trace visited {len(ordered_path)} waypoints, "
              f"but largest component had {len(largest_component_list)} pixels. Path might be incomplete (e.g. if preferred start was in a smaller disconnected part of the chosen component).")
    else:
        print(f"Path ordering complete. Generated {len(ordered_path)} waypoints from the largest component.")
    
    return ordered_path


def mark_ordered_path(map_to_modify, ordered_waypoints, start_value):
    if not ordered_waypoints:
        print("No waypoints to mark.")
        return
    for i, (r, c) in enumerate(ordered_waypoints):
        if 0 <= r < map_to_modify.shape[0] and 0 <= c < map_to_modify.shape[1]:
            map_to_modify[r, c] = start_value + i
    print(f"Marked {len(ordered_waypoints)} waypoints, starting from value {start_value}.")


# --- Main execution block for the path planner ---
def save_optimal_path(input_map,output_map,output_waypoint,start_point = (28, 78) ):
    # Input: The JSON map file saved by robot controller
    # This should be a map populated with 0s, 1s (path), and 2s (free space)
    input_json_map_file = input_map

    # Output files
    output_map_with_path_file = output_map
    output_waypoints_file = output_waypoint

    print(f"Loading map from: {input_json_map_file}")
    map_data_array, map_resolution = load_map_from_json(input_json_map_file)

    if map_data_array is not None and map_resolution is not None:
        print(f"Map loaded. Shape: {map_data_array.shape}, Resolution: {map_resolution}")

        

        centerline_pts = find_centerline_path(
            map_data_array, 
            PATH_MARKER, 
            FREE_SPACE_MARKER,
            preferred_start_rc=start_point # Optional: specify a preferred start point for path ordering
        )

        if centerline_pts:
            map_with_centerline = np.copy(map_data_array)
            mark_ordered_path(map_with_centerline, centerline_pts, SKELETON_PATH_WAYPOINT_START)
            
            # Save the map with the visualized ordered path
            save_map_json(map_with_centerline, map_resolution, output_map_with_path_file)

            waypoints_for_json = []
            for r_np, c_np in centerline_pts:
                waypoints_for_json.append((int(r_np), int(c_np))) # conversion to standard int from numpy int

            # Ensure map_shape elements are also standard Python ints due to typeError which was raised
            map_shape_for_json = [int(s) for s in map_data_array.shape]

            waypoints_data_to_save = {
                "map_resolution": map_resolution, 
                "map_size_cells": map_shape_for_json, 
                "path_start_value_in_map": SKELETON_PATH_WAYPOINT_START, 
                "ordered_waypoints_rc": waypoints_for_json 
            }
            try:
                with open(output_waypoints_file, "w") as f_wp:
                    json.dump(waypoints_data_to_save, f_wp, indent=2) 
                print(f"Ordered waypoints saved to {output_waypoints_file}")
            except TypeError as te: 
                print(f"A TypeError occurred during waypoint saving: {te}. "
                      "Please check all data types in 'waypoints_data_to_save'.")
            except Exception as e:
                print(f"Error saving ordered waypoints: {e}")
            
            print("\nPath planning complete. You can now visualize:") # etc.