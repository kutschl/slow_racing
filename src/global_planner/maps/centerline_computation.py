import cv2
import numpy as np 
import os 
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import pandas as pd 


# Parameter 
MAP_NAME = 'Spielberg'

# Import 
image_path = os.path.join(os.getcwd(), 'src', 'global_planner', 'maps', MAP_NAME, f'{MAP_NAME}_gimp.png')
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Binary image
_, binary_image = cv2.threshold(image, 5, 255, cv2.THRESH_BINARY)

# Flip (needed because of rviz etc...)
binary_image = cv2.flip(binary_image, 0)

# Erode
kernel = np.ones((3, 3), np.uint8)
binary_image = cv2.erode(binary_image, kernel)

# Skeleton fpr initial centerline 
skeleton = binary_image.copy()
kernel = np.ones((3, 3), np.uint8)

for i in range(100):
    print(i) # TODO print msg
    skeleton = cv2.dilate(skeleton, kernel, iterations=10)
    skeleton = cv2.ximgproc.thinning(skeleton)

num_labels, labels_im = cv2.connectedComponents(skeleton)

if num_labels == 2:
    sk = np.zeros_like(binary_image)
    sk[labels_im == 1] = 255
else:
    print('fehler') # TODO beende skript
    
# Edge detection for race track borders
edges = cv2.Canny(binary_image, 100, 200)
num_labels, labels_im = cv2.connectedComponents(edges)
if num_labels == 3:
    b1 = np.zeros_like(edges)
    b2 = np.zeros_like(edges)
    b1[labels_im == 1] = 255
    b2[labels_im == 2] = 255
    
else:
    print("fehler") # TODO beende skript 
    
# Extract ordered centerline points 
contours, _ = cv2.findContours(sk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
centerline_points_f = [pt[0] for pt in max(contours, key=cv2.contourArea)]

# Extract ordered racetrack border points
contours, _ = cv2.findContours(b1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
b1_points = [pt[0] for pt in max(contours, key=cv2.contourArea)]
contours, _ = cv2.findContours(b2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
b2_points = [pt[0] for pt in max(contours, key=cv2.contourArea)]


# Centerline optimization
SPLINE_INTERPOLATION = False
STEP_CENTERLINE = 10

# Keep every 20th point from the centerline
centerline_points = centerline_points_f[::STEP_CENTERLINE]

# Create an output image
output_image = np.zeros((binary_image.shape[0], binary_image.shape[1], 3), dtype=np.uint8)

# Draw the contours
output_image[b1 == 255] = [255, 0, 255]
output_image[b2 == 255] = [255, 0, 255]
output_image[sk == 255] = [255, 0, 255]

# Convert points to NumPy arrays
centerline_points = np.array(centerline_points)
b1_points = np.array(b1_points)
b2_points = np.array(b2_points)

# Backup copy of the centerline points
cur_centerline_points = centerline_points.copy()

itt = 10
# Iteratively optimize the centerline
for i in range(itt):
    # List to hold optimized centerline points
    optimized_centerline_points = []

    # Draw normal vectors and find the closest points on splines
    for idx, cp1 in enumerate(cur_centerline_points):
        # Determine the next point for the tangent calculation
        if idx == len(cur_centerline_points) - 1:
            cp2 = cur_centerline_points[0]
        else:
            cp2 = cur_centerline_points[idx + 1]

        # Calculate the tangent and normal vectors
        T = cp2 - cp1  # Tangent vector
        if np.linalg.norm(T) < 1e-6:  # Check for near-zero length to avoid NaNs
            N = np.array([0, 1])  # Default normal if T is zero
        else:
            N = np.array([-T[1], T[0]])  # Normal vector
            N = N / np.linalg.norm(N)  # Normalize the normal vector

        # Calculate the endpoint for the normal vector and convert to integer tuple
        start_point = (int(cp1[0]), int(cp1[1]))
        end_point = (int(cp1[0] + N[0] * 20), int(cp1[1] + N[1] * 20))  # Length of the normal vector is 20 pixels

        # Draw the normal vector in red in the last iteration
        if i == itt - 1:
            cv2.arrowedLine(output_image, start_point, end_point, (0, 0, 255), 1)

        b1d = np.linalg.norm(b1_points-cp1, axis=1)
        b1d_idx = np.argsort(b1d)
        b1_points_s = b1_points[b1d_idx[:20]]
        bd = []
        for bidx, b1p in enumerate(b1_points_s):
            if N[0] == 0 and N[1] != 0:
                t = (b1p[1] - cp1[1])/N[1]
            elif N[0] != 0:
                t = (b1p[0] - cp1[0])/N[0]
            else:
                pass
            
            cp1_schnittpunkt = cp1 + t*N
            bd.append(np.linalg.norm(b1p - cp1_schnittpunkt))
        bd = np.array(bd)
        b1min = b1_points_s[np.argmin(bd)]
        
        b2d = np.linalg.norm(b2_points-cp1, axis=1)
        b2d_idx = np.argsort(b2d)
        b2_points_s = b2_points[b2d_idx[:20]]
        bd = []
        for bidx, b2p in enumerate(b2_points_s):
            if N[0] == 0 and N[1] != 0:
                t = (b2p[1] - cp1[1])/N[1]
            elif N[0] != 0:
                t = (b2p[0] - cp1[0])/N[0]
            else:
                pass
            
            cp1_schnittpunkt = cp1 + t*N
            bd.append(np.linalg.norm(b2p - cp1_schnittpunkt))
        bd = np.array(bd)
        b2min = b2_points_s[bd.argmin()]
        
        if i == itt-1:
            cv2.line(output_image, b1min, b2min, (0, 255, 255), 1)  # Yellow line between points
            
        x = int((b1min[0] + b2min[0])/2)
        y = int((b1min[1] + b2min[1])/2)
        
        optimized_centerline_points.append([x,y])
    
    optimized_centerline_points.append(optimized_centerline_points[0])
    optimized_centerline_points = np.array(optimized_centerline_points)
    
    if SPLINE_INTERPOLATION:
        # spline interpolation
        cx = optimized_centerline_points[:,0]
        cy = optimized_centerline_points[:,1]
        
        spline_cx = CubicSpline(np.arange(len(cx)), cx, bc_type='periodic')
        spline_cy = CubicSpline(np.arange(len(cy)), cy, bc_type='periodic')
        
        ct_values = np.linspace(0, len(cx) - 1, len(optimized_centerline_points))  # 1000 points for smooth curve
        c_spline = np.array([spline_cx(ct_values), spline_cy(ct_values)]).T
        
        print(c_spline[0])
        print(type(c_spline[0]), type(c_spline[0][0]))
        
        # maske aus spline erstellen
        cmask = np.zeros((output_image.shape[0], output_image.shape[1]), dtype=np.uint8)
        for j in range(len(c_spline)):
            p1 = tuple(c_spline[j].astype(int))
            if j == len(c_spline)-1:
                p2 = tuple(c_spline[0].astype(int))
            else:
                p2 = tuple(c_spline[j+1].astype(int))
            cv2.line(cmask, p1, p2, 255, 1)
            if i == itt-2:
                cv2.line(output_image, p1, p2, [0,255,0], 1)
    
    # better
    else:
        # maske aus spline erstellen
        cmask = np.zeros((output_image.shape[0], output_image.shape[1]), dtype=np.uint8)
        
        for j in range(len(optimized_centerline_points)):
            p1 = tuple(optimized_centerline_points[j].astype(int))
            if j == len(optimized_centerline_points)-1:
                p2 = tuple(optimized_centerline_points[0].astype(int))
            else:
                p2 = tuple(optimized_centerline_points[j+1].astype(int))
            cv2.line(cmask, p1, p2, 255, 1)
            if i == itt-2:
                cv2.line(output_image, p1, p2, [0,255,0], 1)
    
    if i < itt:
        # connected component
        contours, _ = cv2.findContours(cmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        loop = max(contours, key=cv2.contourArea)
        centerline_points = [pt[0] for pt in loop]
        centerline_points = centerline_points[::STEP_CENTERLINE]
        cur_centerline_points = np.array(centerline_points)
    
cx = optimized_centerline_points[:,0]
cy = optimized_centerline_points[:,1]
        
spline_cx = CubicSpline(np.arange(len(cx)), cx, bc_type='periodic')
spline_cy = CubicSpline(np.arange(len(cy)), cy, bc_type='periodic')

ct_values = np.linspace(0, len(cx) - 1, len(optimized_centerline_points))  # 1000 points for smooth curve
c_spline = np.array([spline_cx(ct_values), spline_cy(ct_values)]).T

print(c_spline[0])
print(type(c_spline[0]), type(c_spline[0][0]))
        
# maske aus spline erstellen
cmask = np.zeros((output_image.shape[0], output_image.shape[1]), dtype=np.uint8)
for j in range(len(c_spline)):
    
    p1 = tuple(c_spline[j].astype(int))
    if j == len(c_spline)-1:
        p2 = tuple(c_spline[0].astype(int))
    else:
        p2 = tuple(c_spline[j+1].astype(int))


    cv2.line(output_image, p1, p2, [255,255,255], 1)
    cv2.line(cmask, p1,p2,255,1)

# # starting point
# racetrack_starting_point = c_spline[4].astype(int)

# Save the output image and mask image
# cv2.imwrite('centerline_with_smoothed_optimized_centerline.png', output_image)


# find contours from cmask
contours, _ = cv2.findContours(cmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
loop = max(contours, key=cv2.contourArea)
cmask_points = [pt[0] for pt in loop]
cmask_points = np.array(cmask_points)


# # relative coordinates
# cmask_points_relative = cmask_points-racetrack_starting_point

# # starting idx to rearrange array
# for idx, cp in enumerate(cmask_points):
#     if (cp==[0,0]).all():
#         starting_idx = idx

# # rearrange array
# cmask_points_relative = np.concatenate((cmask_points_relative[starting_idx:], cmask_points_relative[:starting_idx]))

# # meter
# RESOLUTION = 0.05
# cmask_points_relative_meter = cmask_points_relative*RESOLUTION
# cmask_points = np.round(cmask_points, 2)

# csv file 
centerline_csv = pd.DataFrame(
    columns=['x', 'y']
)
for i in range(len(cmask_points)):
    centerline_csv.loc[len(centerline_csv.index)] = [float(cmask_points[i][0]), float(cmask_points[i][1])]
    
centerline_csv_path = os.path.join(os.getcwd(), 'src', 'global_planner', 'maps', MAP_NAME, f'{MAP_NAME}_centerline.csv')
centerline_csv.to_csv(centerline_csv_path, index=False)