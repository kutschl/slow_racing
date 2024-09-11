import os
import cv2 

map_name = 'HRL'

map_image_path = os.path.join(os.getcwd(), 'src', 'global_planner', 'maps', map_name, f'{map_name}_map.png')
map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
_, map_image = cv2.threshold(map_image, 250, 255, cv2.THRESH_BINARY)
map_image_path = os.path.join(os.getcwd(), 'src', 'global_planner', 'maps', map_name, f'{map_name}_map_bw.png')
map_image = cv2.imwrite(map_image_path, map_image)
