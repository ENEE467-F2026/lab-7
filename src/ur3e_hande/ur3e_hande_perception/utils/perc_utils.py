import json
import numpy as np
import cv2
import os
from sklearn.cluster import KMeans
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from ament_index_python.packages import get_package_share_directory
import spatialmath as spmath
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple, Union
import open3d as o3d

CLASS_NAMES = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ] # object classes

package_name = 'ur3e_hande_perception'
package_share_dir = get_package_share_directory(package_name)

COLOR_JSON = os.path.join(package_share_dir, 'config', 'colornames.json')
COLOR_RGB_JSON = os.path.join(package_share_dir, 'config', 'colornames_with_rgb.json')
MODELS_DIR = os.path.join(package_share_dir, 'models')

def image_size_calc(img):
    """
    Simple script to check image memory footprint.
    Returns image size in kilobytes of the OpenCV image object img.
    """
    size_bytes = img.nbytes
    size_kb = size_bytes / 1024
    return size_kb

def compute_colored_point_cloud(depth_image, rgb_image, K, pack_rgb=True):
    """
    Returns Nx6 point cloud, where each point has (X, Y, Z, R, G, B).

    K is the camera matrix
    """
    # print(depth_image.shape)
    height, width = depth_image.shape
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # Meshgrid of pixel coordinates
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # Backproject pixels to X, Y, Z in world frame
    Z = depth_image.flatten()
    X = ((u - cx) * depth_image / fx).flatten()
    Y = ((v - cy) * depth_image / fy).flatten()

    # Get RGB values and reshape to Nx3 to 'color' cloud
    colors = rgb_image.reshape(-1, 3)

    # Combine into Nx6 point cloud
    points = np.vstack((X, Y, Z)).T
    point_cloud = np.hstack((points, colors.astype(np.float32)))  # shape: (N, 6), N = width*height

    if pack_rgb:
       packed_points = []
       for i in range(point_cloud.shape[0]):
            x, y, z = point_cloud[i, 0:3]
            r, g, b = point_cloud[i, 3:]
            rgb_packed = pack_rgb_to_float(r, g, b)
            packed_points.append([x, y, z, rgb_packed])
       point_cloud = np.array(packed_points, dtype=np.float32)
    return point_cloud

def get_rs_matrix_params(matrix, matrix_type="proj"):
  """
  Method for retrieving the parameters of the camera from the CameraInfo message.

  Note: 
  For stereo cameras, projection matrix combines focal length and principal pioints with distance between the optical centers of the stereo pair; for monocular cameras; Tx = Ty = 0, and p is a 3x3 matrix.
  
  Realsense asserts that Tz = 0. This assumption is sound for a perspective projection camera model; however, it will not work for a pin-hole camera model, since pixels are transformed by a linear map scaled by 1/Tz. 
  #     [fx'  0  cx' Tx]
  # P = [ 0  fy' cy' Ty]
  #     [ 0   0   1   0]

  #     [fx  0 cx]
  # K = [ 0 fy cy]
  #     [ 0  0  1]

  fx and fx' may be different; ditto for other params.
  """
  if matrix_type == "proj":
    return [matrix[0,0], matrix[0,2], matrix[0,3], matrix[1,1], matrix[1, 2], matrix[1,3]]    
  elif matrix_type == "cam":
    return [matrix[0,0], matrix[0,2], matrix[1,1], matrix[1, 2]]  
  else:
    return NotImplementedError
  
def canonical_proj_function(proj_matrix, camera_matrix, m, Rot_W_C, p_W_C):
    """
    Given the camera matrix K, projection matrix P, an observed point m (3D, in world frame),
    the canonical projection function returns the pixel coordinates of m as observed by a camera at position 
    p_W_C and orientation Rot_W_C (rotation from camera to world).
    """
    # Transform point from world to camera frame
    m = np.asarray(m).reshape(3, 1)
    Rot_C_W = Rot_W_C.T  # inverse rotation
    t_C_W = -Rot_C_W @ p_W_C.reshape(3, 1)
    m_C = Rot_C_W @ m + t_C_W  # 3x1

    # Project to pixel coordinates using camera intrinsics
    x, y, z = m_C.flatten()
    if z == 0:
        return None  # avoid division by zero

    u = camera_matrix[0, 0] * x / z + camera_matrix[0, 2]
    v = camera_matrix[1, 1] * y / z + camera_matrix[1, 2]
    return (float(u), float(v))

def bgr_to_hex(bgr):
    b, g, r = bgr  
    return '#{:02x}{:02x}{:02x}'.format(r, g, b)  # min width 2 for hex conversion of each color channel

def hex_to_rgb(hex_code):
    hex_code = hex_code.lstrip('#')
    return tuple(int(hex_code[i:i+2], 16) for i in (0, 2, 4))

def get_dominant_color(img, use_kmeans=True, k=20):
    img_array = np.asarray(img, dtype=np.uint8)
    h, w, c = img_array.shape

    # reshape to a list of pixels
    img_rgb = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)

    # Reshape image to a list of pixels
    img_reshaped = img_rgb.reshape((-1, 3))

    if use_kmeans:
        # Run KMeans to find dominant cluster
        kmeans = KMeans(n_clusters=k, random_state=42, n_init='auto')
        kmeans.fit(img_reshaped)

        # count how many pixels belong to each cluster
        _, counts = np.unique(kmeans.labels_, return_counts=True)

        # get the centroid of the largest cluster)
        dominant_index = np.argmax(counts)
        dominant_color_bgr = tuple(map(int, kmeans.cluster_centers_[dominant_index]))

        return dominant_color_bgr, bgr_to_hex(dominant_color_bgr)
    else:
        # return center pixel color
        center_y, center_x = int(h/2), int(w/2)
        return tuple(int(c) for c in img_array[center_y, center_x, :]), bgr_to_hex(tuple(int(c) for c in img_array[center_y, center_x, :]))

def get_bbox_corners(bbox_xywh):
  x, y, w, h = bbox_xywh
  return [[int(x-0.5*w), int(y-0.5*h)], 
                  [int(x+0.5*w), int(y-0.5*h)],
                  [int(x+0.5*w), int(y+0.5*h)],
                  [int(x-0.5*w), int(y+0.5*h)] ]

def get_bbox_img(img, box_xywh):
  corners = get_bbox_corners(box_xywh)
  img_array = np.asarray(img, dtype=np.uint8)
  return img_array[corners[0][1]:corners[3][1], corners[0][0]:corners[1][0], :]

def get_color_name_from_json(rgb_color, json_file_abs_path=COLOR_RGB_JSON):
    with open(json_file_abs_path, 'r') as f:
        colors = json.load(f)

        color_rgbs = np.array([color['rgb'] for color in colors])
        rgb_color = np.array(rgb_color)

        distances = np.linalg.norm(color_rgbs - rgb_color, axis=1)

        min_idx = np.argmin(distances)
        closest_color = colors[min_idx]

        return closest_color['name'], closest_color['rgb'], closest_color['hex']

def pack_rgb_to_float(r, g, b):
   # pack RGB into a 32 bit float comprising 24-bits for the three channels and upper 8 bits, which are 0 by default since there is no alpha channel
    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
    return np.frombuffer(np.uint32(rgb_int).tobytes(), dtype=np.float32)[0]

def tf_to_matrix(transform_stamped: TransformStamped):
   """
   Converts a TransformStamped ROS2 interface to a 4x4 numpy array
   """
   t = transform_stamped.transform.translation
   q = transform_stamped.transform.rotation
   
   # ROS quaternion: x, y, z, w
   quat = [q.x, q.y, q.z, q.w]
   
   # Convert quaternion to rotation matrix
   rot_matrix = R.from_quat(quat).as_matrix()
   rot_vec = R.from_quat(quat).as_rotvec()

   # Convert translation to numpy array
   t = np.array([t.x, t.y, t.z])
   
   # Create SE3 transformation
#    print(rot_vec)
#    print(t)
   H = spmath.SE3.RTvec(rvec=rot_vec, tvec=t)
   return H

def extract_plane(cloud: o3d.geometry.PointCloud, dist_thresh: float = 0.02, ransac_n: int = 3, num_iterations: int=1000) -> Tuple[np.ndarray, np.ndarray, o3d.geometry.PointCloud]:
        """Segmentation: Extracts a plane from the point cloud using the RANSAC algorithm
        
        Reference: http://www.cse.yorku.ca/~kosta/CompVis_Notes/ransac.pdf

        scalar plane equation: ax + by + cz + d = 0
        """
        plane_model, inliers = cloud.segment_plane(distance_threshold=dist_thresh, 
                                                   ransac_n=ransac_n,
                                                   num_iterations=num_iterations)
        coefficients = plane_model # [a, b, c, d]
        # # Extract points belonging to the plane
        plane_cloud = cloud.select_by_index(inliers)
        # plane_cloud.paint_uniform_color([1.0, 0, 0])
        return inliers, coefficients, plane_cloud

def extract_cloud_clusters(cloud: o3d.geometry.PointCloud, cluster_name: str, cluster_tol:float=0.05, min_cluster_sz: int=80, max_cluster_sz: int=80000) -> Tuple[List[o3d.geometry.PointCloud], List[List[float]], List[List[float]]]:
    """Extracts clusters corresponding to flat surfaces from the point cloud

    Modify open3d's dbscan algorithm to restrict clusters to Euclidean balls of radius cluster_tol

    https://www.open3d.org/html/tutorial/Basic/pointcloud.html#DBSCAN-clustering
    """
    # tree = o3d.geometry.KDTreeFlann(cloud) cant specify search alg

    # extract clusters
    cluster_labels = np.array(cloud.cluster_dbscan(eps=cluster_tol, min_points=min_cluster_sz, print_progress=False)) # ints; -1 indicates noise; max is max_label; 
    # points in the same cluster have the same cluster_idx

    # lists to store object clusters, centroids, and dimensions
    cluster_clouds = []
    cluster_centroids = []
    cluster_dimensions = []

    # process each cluster
    max_label = cluster_labels.max()
    for i in range(max_label + 1): # 0, 1, 2, ..., max_label
        indices = np.where(cluster_labels == i)[0]

        cluster = cloud.select_by_index(indices)
        points = np.asarray(cluster.points)

        # print(points, np.shape(points))

        # skip empty clusters
        if points.shape[0] == 0:
            continue
        centroid = np.mean(points, axis=0)# Calculate centroid

        # Computes the min and max coordinates along each axis 
        min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)
        dimensions = max_coords - min_coords

        # skip spurious objects 
        max_dim = 0.2  # 50 cm max per axis; get a bigger robot!
        min_dim = 0.05  # 3 cm min per axis
        if any(d > max_dim or d < min_dim for d in dimensions) and cluster_name.lower() == "object":
            print(f"\033[93mSkipping cluster {i} due to size: {dimensions}\033[0m")
            continue

        # Append clusters, centroids and dimensions to lists
        cluster_clouds.append(cluster)
        cluster_centroids.append(centroid.tolist())
        cluster_dimensions.append(dimensions.tolist())

        # Log cluster information
        num_points = len(indices)
        # print(f"\033[92m{cluster_name} cluster {i + 1} has {num_points} points.\033[0m")
        print(f"\033[92mCentroid of {cluster_name} cluster {i + 1}: {centroid}\033[0m")
        # print(f"\033[92mDimensions of {cluster_name} cluster {i + 1}: {dimensions}\033[0m")

    # Check if any clusters have been extracted
    if not cluster_clouds:
        print(f"\033[91mNo {cluster_name} clusters extracted...\033[0m")

    # write clusters to disk
    # o3d.io.write_point_cloud(os.path.abspath("/clouds/clusters"), cluster_clouds)
    # Return the filtered surface clusters, centroids and cluster dimensions
    return cluster_clouds, cluster_centroids, cluster_dimensions

def filter_cloud(cloud: o3d.geometry.PointCloud, max_x_dist: float, min_height: float, max_height: float) -> Union[o3d.geometry.PointCloud, None]:
    """Filter a point cloud"""
    points = np.asarray(cloud.points)
    mask = (points[:, 0] <= max_x_dist) & (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    indices = np.where(mask)[0]
    return cloud.select_by_index(indices)
    
def load_model_point_cloud(class_name, model_dir=MODELS_DIR):
    """
    Load a point cloud model for the given class name.
    Assumes the model is stored in a .npy file with the class name.

    TODO: I'll explore this route later. Module is not used yet.
    """
    model_path = f"{model_dir}/{class_name}.npy"
    try:
        model_pc = np.load(model_path)
        return model_pc
    except FileNotFoundError:
        raise ValueError(f"Model for class '{class_name}' not found at {model_path}")