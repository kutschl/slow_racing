import yaml
import numpy as np
import pandas as pd
import os 
import math

# Parameter 
map_name = 'Spielberg'
map_resolution = 0.05
origin = [0.0, 0.0, 0.0]
starting_pose_idx = 0

# Centerline for starting pose
centerline_csv_path = os.path.join(os.getcwd(), 'src', 'global_planner', 'maps', map_name, f'{map_name}_centerline.csv')
centerline_csv = pd.read_csv(centerline_csv_path)
x1 = centerline_csv.loc[starting_pose_idx,'x']
y1 = centerline_csv.loc[starting_pose_idx,'y']
x2 = centerline_csv.loc[starting_pose_idx+5,'x']
y2 = centerline_csv.loc[starting_pose_idx+5,'y']
yaw = math.atan2(y2 - y1, x2 - x1)
starting_pose = [np.float64(x1)*map_resolution,np.float64(y1)*map_resolution,np.float64(yaw)]
starting_pose = [float(val) for val in starting_pose]

# Creating yaml file
data = {
    'image': f'{map_name}_map.png',
    'negate': 0,
    'occupied_thresh': 0.75,
    'free_thresh': 0.25,
    'resolution': map_resolution, 
    'origin': origin,
    'starting_pose': starting_pose
}
yaml_path = os.path.join(os.getcwd(), 'src', 'global_planner', 'maps', map_name, f'{map_name}_map.yaml')
with open(yaml_path, 'w') as file:
    yaml.dump(data, file, default_flow_style=None, sort_keys=False)

print(f'Configs for Map "{map_name}" created successfully!')



