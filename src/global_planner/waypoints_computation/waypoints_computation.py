import os
import numpy as np
from skimage import io, morphology, filters
from scipy.ndimage import distance_transform_edt
from scipy.spatial.distance import cdist
import cv2
import yaml
import math

# TODO: use ament share package instead of src folders
# TODO: use map name from map.yaml config file instead of parameter

# PARAMETERS
PACKAGE_NAME = 'global_planner' # Package name
MAPS_DIR = os.path.join(os.getcwd(), 'src', PACKAGE_NAME, 'maps') # Path of maps directory
CENTERLINE_DIR = os.path.join(os.getcwd(), 'src', PACKAGE_NAME, 'waypoints_computation') # Path of centerline directory
MAP_NAME = 'Spielberg' # Map name
WP_STEP = 10 # 10 # Keep every WP_STEP-th waypoint 
REJECT_WP = 0.00 # 0.09 # Accept each waypoint with norm over that threshold
CURVE_SPEED = 0.5
STRAIGHT_SPEED = 0.8

# PATHS
track_path = os.path.join(MAPS_DIR, MAP_NAME, MAP_NAME + '_track.png')
map_parameter_path = os.path.join(MAPS_DIR, MAP_NAME, MAP_NAME + '_map.yaml')
cl_path = os.path.join(CENTERLINE_DIR, MAP_NAME + '_centerline.png')
wp_path = os.path.join(os.getcwd(), 'src', PACKAGE_NAME, 'config', 'waypoints.yaml')


# EXTRACT CENTERLINE
if os.path.exists(track_path):
    # Open track image from path
    track_image = io.imread(track_path, as_gray=True)
    # Thresholding to create binary image
    track_image = track_image > filters.threshold_otsu(track_image)
    # Remove small objects
    track_image = morphology.remove_small_objects(track_image, min_size=500)
    # Smoothing the edges
    track_image = filters.gaussian(track_image, sigma=1.0)
    # Skeletonize
    track_image = morphology.skeletonize(track_image > 0.5)
    # Save resulting centerline as image
    io.imsave(cl_path, track_image.astype(np.uint8) * 255)
else:
    print('Track does not exist.')


# WAYPOINT COMPUTATION
if os.path.exists(cl_path):
    # Open centerline image from path
    centerline_image = cv2.imread(cl_path, cv2.IMREAD_GRAYSCALE)
    # Flip and rotate centerline image
    centerline_image = cv2.rotate(centerline_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    centerline_image = cv2.flip(centerline_image, 0)
    centerline_image = cv2.flip(centerline_image, 1)
    # Load map parameters from yaml file. Otherwise set default values for map resolution and origin.
    if os.path.exists(map_parameter_path):
        with open(map_parameter_path, 'r') as file:
            map_parameter = yaml.safe_load(file)
            map_resolution = map_parameter['resolution']
            map_origin = map_parameter['origin']
    else:
        map_resolution = 0.05
        map_origin = np.array([0.0, 0.0, 0.0])
    # Find all coordinates of white pixels
    white_pixels = np.column_stack(np.where(centerline_image))
    # Transform white pixel coordinates into waypoints (x,y) using map resolution and origin
    wp = np.zeros(shape=(len(white_pixels),2))
    for i in range(len(wp)):
        wp[i] = white_pixels[i]*map_resolution + map_origin[0:2]
    
    # Sort waypoints in order of the following the racetrack
    def sort_waypoints(wp:np.ndarray) -> np.ndarray:
        sorted_wp = np.zeros(shape=wp.shape)
        current_wp = wp[0]
        sorted_wp[0] = current_wp
        wp = wp[1:]
        index = 1
        while len(wp) > 0:
            distances = cdist([current_wp], wp)
            nearest_index = np.argmin(distances)
            nearest_point = wp[nearest_index]
            wp = np.delete(wp, nearest_index, 0)
            sorted_wp[index] = nearest_point
            index += 1
            current_wp = nearest_point
        return sorted_wp
    sorted_wp = sort_waypoints(wp)
    
    # Reducing the number of waypoints. 
    # Keep only every WP_STEP-th waypoint.
    n = int((sorted_wp.shape[0] - (sorted_wp.shape[0]%WP_STEP))/WP_STEP) + 1
    reduced_wp = np.zeros(shape=(n, sorted_wp.shape[1]))
    for i in range(len(sorted_wp)):
        if i % WP_STEP == 0:
            reduced_wp[int(i/WP_STEP)] = sorted_wp[i]
            
    # Keep only relevant waypoints that change in the angle between the previous and following direction vector.
    # Reject every waypoint with angle < REJECT_WP.
    wp_with_dv = np.zeros(shape=(reduced_wp.shape[0], 2, 2))
    # Compute the direction vector for each waypoint
    for idx, wp in enumerate(reduced_wp):
        if idx == len(reduced_wp)-1:
            wp_next = reduced_wp[0]
        else:
            wp_next = reduced_wp[idx+1]
        dv = wp_next - wp
        wp_with_dv[idx][0] = wp
        wp_with_dv[idx][1] = dv
    # Compute the index of the rejected waypoints
    idx_rejected_wp = []
    for idx, wp in enumerate(wp_with_dv):
        if idx == 0:
            a = wp_with_dv[-1][1]
        else:
            a = wp_with_dv[idx-1][1]
        b = wp[1]
        # scalar product
        ab = np.dot(a,b)
        # norm
        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)
        # angle 
        angle = np.arccos(ab / (na*nb))
        # mark the rejected waypoints
        if angle < REJECT_WP:
            idx_rejected_wp.append(idx)
    # Remove all rejected waypoints 
    accepted_wp = np.delete(reduced_wp, idx_rejected_wp, axis=0)
    
    # Compute the new direction vector for each accepted waypoint 
    accepted_wp_with_dv = np.zeros(shape=(accepted_wp.shape[0], 2, 2))
    for idx, wp in enumerate(accepted_wp):
        if idx == len(accepted_wp)-1:
            wp_next = accepted_wp[0]
        else:
            wp_next = accepted_wp[idx+1]
        dv = wp_next - wp
        accepted_wp_with_dv[idx][0] = wp
        accepted_wp_with_dv[idx][1] = dv
    
    # Compute theta, angle and linear velocity for each waypoint
    wp_final = np.zeros(shape=(accepted_wp.shape[0], 5))
    for idx, wp in enumerate(accepted_wp):
        wp_final[idx][0] = wp[0]
        wp_final[idx][1] = wp[1]
        # theta
        dv = accepted_wp_with_dv[idx][1]
        ndv = np.linalg.norm(dv)
        dv = dv / ndv
        theta = math.atan2(dv[1], dv[0])
        wp_final[idx][2] = theta
        # angle
        if idx == 0:
            a = accepted_wp_with_dv[-1][1]
        else:
            a = accepted_wp_with_dv[idx-1][1]
        b = wp
        ab = np.dot(a,b)
        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)
        angle = np.arccos(ab / (na*nb))
        wp_final[idx][3] = angle
      
    # Save waypoints as yaml 
    # TODO: add theta, angle, linear velocity
    with open(wp_path, 'w') as file:
        file.write("waypoints:\n")
        for idx, wp in enumerate(wp_final):
            file.write(f'  - x: {wp[0]}\n')
            file.write(f'    y: {wp[1]}\n')
            file.write(f'    theta: {wp[2]}\n')     
            print(idx, '\t', round(wp[0], 1), round(wp[1], 1), round(wp[2], 4), round(wp[3], 4))
    print(f'{wp_final.shape[0]} waypoints computed.')  
else:
    print('Centerline does not exist.')
