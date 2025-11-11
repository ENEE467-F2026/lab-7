#!/usr/bin/env python3

"""
# Node for pose estimation using YOLO and way too many pointcloud tricks.

# IMPORTANT: The code assumes that the depth and color images have the same resolution, and that the infrared, color, depth, and pointcloud streams are enabled. You can ensure this by running the realsense2_camera rs_launch.py launch file like so:
# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 pointcloud.enable:=true align_depth.enable:=true pointcloud.ordered_pc:=true enable_infra:=true enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=640x480x30

Author: Clinton Enwerem.
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""
import rclpy
import os
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile, DurabilityPolicy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from ultralytics import YOLO
import spatialmath as spmath
from realsense2_camera_msgs.msg import Extrinsics
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
import open3d as o3d
from ament_index_python.packages import get_package_share_directory
import importlib.util
from transforms3d.euler import euler2quat
from typing import List, Dict, Union
from visualization_msgs.msg import MarkerArray, Marker
from scipy.spatial.transform import Rotation as R
import struct

# custom interfaces (msg)
from ur3e_hande_perception.msg import ObjectUId, InferenceResult, YoloInference, ObjectMetaData, DetectedSurfaces, DetectedObjects

# custom interfaces (action)
from rclpy.action import ActionServer
from ur3e_hande_perception.action import GetTargetObjPose

# transforms
from tf2_ros import LookupException, TransformException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

# utils
package_name = 'ur3e_hande_perception'
package_share_dir = get_package_share_directory(package_name)

# add workspace path to sys
utils_file_path = os.path.join(package_share_dir, 'utils', 'perc_utils.py')
spec = importlib.util.spec_from_file_location("perc_utils", utils_file_path)
utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utils)

# pclpy
pcl_file_path = os.path.join(package_share_dir, 'utils', 'perc_utils.py')
spec = importlib.util.spec_from_file_location("perc_utils", utils_file_path)
utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utils)

# configure QoS profile for publishing and subscribing
QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class YoloPoseEstimationActionServer(Node):
    """
    Class constructor for pose estimation
    """
    def __init__(self, node_name="yolo_pc_pose_estimator", queue_size = 10, qos_profile=QOS_PROFILE):
        self.node_name = node_name
        super().__init__(self.node_name)
        self.queue_size = queue_size
        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0), node=self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # param descriptors
        self.bridge_color_img_enc_desc = ParameterDescriptor(description="Image encoding to use for casting color image message to OpenCV-acceptable format. Options: 'bgr8', or one from the list of strings in include/sensor_msgs/image_encodings.h")
        self.bridge_depth_img_enc_desc = ParameterDescriptor(description="Image encoding to use for casting depth image message to OpenCV-acceptable format. Options: '16UC1', or one from the list of strings in include/sensor_msgs/image_encodings.h")    
        self.yolo_model_desc = ParameterDescriptor(description="The name of the YOLO model to use for object detection. Default is the v8n model, but you can adapt as you see fit.")
        self.world_to_cam_desc = ParameterDescriptor(description="The camera pose in the world frame expressed as a list of space-separated FLOATING-POINT values in the following order: [roll pitch yaw x y z]. \n Eliminates need to refactor code if hardware setup changes. \n Can also be read from tf_static if tree is valid.")

        # Realsense params
        self.declare_parameter("rs_color_info_topic", value="/camera/camera/color/camera_info")
        self.declare_parameter("rs_depth_info_topic", value="/camera/camera/color/depth_info")
        self.declare_parameter("rs_color_topic", value="/camera/camera/color/image_raw")
        self.declare_parameter("rs_depth_topic", value="/camera/camera/aligned_depth_to_color/image_raw") 
        self.declare_parameter("rs_pc_topic", value="/camera/camera/depth/color/points")
        self.declare_parameter("rs_infra_left_topic", value="/camera/camera/infra1/camera_info")
        self.declare_parameter("rs_infra_right_topic", value="/camera/camera/infra2/camera_info") # for actual non-zero intrinsic params; other topics will give funny intrinsics
        self.declare_parameter('num_pcd_points', value=20)
        self.declare_parameter('surface_thickness', value=0.05)
        self.declare_parameter('offset_x', value=0.05)

        # Pointcloud processing params
        self.declare_parameter('max_x_dist', value=0.45)    # around 0.5 Vention table width
        self.declare_parameter('min_height_sf', value=0.02)
        self.declare_parameter('max_height_sf', value=0.15)
        self.declare_parameter('min_height_obj', value=0.15)
        self.declare_parameter('max_height_obj', value=0.3)

        # tf 
        self.declare_parameter('rs_pc_frame_id', value="camera_depth_optical_frame")
        self.declare_parameter('rs_cbs_frame_id', value="camera_bottom_screw_frame")
        self.declare_parameter('world_frame_id', value="world")
        self.declare_parameter('camera_link_frame_id', value="camera_link")
        self.declare_parameter('world_to_camera_tf', value=[0.0, 0.698131, 0.0, -0.522625, 0.542925, 0.34005], descriptor=self.world_to_cam_desc)

        # OpenCV params
        self.declare_parameter("bridge_color_img_enc", value="bgr8", descriptor=self.bridge_color_img_enc_desc)
        self.declare_parameter("bridge_depth_img_enc", value="16UC1", descriptor=self.bridge_depth_img_enc_desc)
        self.declare_parameter("yolo_model", value="yolov8n.pt", descriptor=self.yolo_model_desc)
        self.declare_parameter('bbox_rect_line_width', value=3)

        # pubber params
        self.declare_parameter('annotated_img_topic', value="/annotated_image")
        self.declare_parameter('yolo_detections_topic', value="/yolo_detections")
        self.declare_parameter('object_cloud_topic', value='/object_cloud')
        self.declare_parameter('object_pose_topic', value='/object_pose')
        self.declare_parameter('object_metadata_topic', value="/object_metadata")

        # get everything
        self.rs_color_topic = self.get_parameter("rs_color_topic").value
        self.rs_depth_topic = self.get_parameter("rs_depth_topic").value
        self.rs_color_info_topic = self.get_parameter("rs_color_info_topic").value
        self.rs_pc_topic = self.get_parameter("rs_pc_topic").value
        self.rs_infra_left_topic = self.get_parameter("rs_infra_left_topic").value
        self.rs_infra_right_topic = self.get_parameter("rs_infra_right_topic").value
        self.bridge_color_img_encoding = self.get_parameter('bridge_color_img_enc').value
        self.bridge_depth_img_encoding = self.get_parameter('bridge_depth_img_enc').value
        self.yolo_model = YOLO(self.get_parameter('yolo_model').value)
        self.rs_pc_frame_id = self.get_parameter("rs_pc_frame_id").value  
        self.rs_cbs_frame_id = self.get_parameter("rs_cbs_frame_id").value  
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.camera_link_frame_id = self.get_parameter('camera_link_frame_id').value
        self.annotated_img_topic = self.get_parameter('annotated_img_topic').value
        self.yolo_detections_topic = self.get_parameter('yolo_detections_topic').value
        self.object_cloud_topic = self.get_parameter('object_cloud_topic').value
        self.object_pose_topic = self.get_parameter('object_pose_topic').value
        self.object_metadata_topic = self.get_parameter('object_metadata_topic').value
        self.num_pcd_points = self.get_parameter('num_pcd_points').value
        self.surface_thickness = self.get_parameter('surface_thickness').value
        self.offset_x = self.get_parameter('offset_x').value
        self.max_x_dist = self.get_parameter('max_x_dist').value
        self.min_height_sf = self.get_parameter('min_height_sf').value
        self.max_height_sf = self.get_parameter('max_height_sf').value
        self.min_height_obj = self.get_parameter('min_height_obj').value
        self.max_height_obj = self.get_parameter('max_height_obj').value

        # subscribers
        self.create_subscription(Image, self.rs_color_topic, self.rs_color_callback, self.queue_size)
        self.create_subscription(Image, self.rs_depth_topic, self.rs_depth_callback, self.queue_size)
        self.create_subscription(CameraInfo, self.rs_color_info_topic, self.rs_infra_right_info_callback, self.queue_size)
        self.create_subscription(Image, self.rs_color_topic, self.yolo_and_depth_callback, self.queue_size)
        self.create_subscription(Image, self.rs_depth_topic, self.yolo_and_depth_callback, self.queue_size)
        self.create_subscription(PointCloud2, self.rs_pc_topic, self.publish_sf_obj_pcd_det_cb, self.queue_size)

        # publishers
        self.object_uid_publishers = {} # cache publishers to avoid calling the same publisher
        self.annotated_img_pubber_ = self.create_publisher(Image, self.annotated_img_topic, self.queue_size)
        self.yolo_detections_pubber_ = self.create_publisher(YoloInference, self.yolo_detections_topic, self.queue_size)
        self.object_cloud_pubber_ = self.create_publisher(PointCloud2, self.object_cloud_topic, self.queue_size)
        self.object_pose_pubber_ = self.create_publisher(PoseStamped, self.object_pose_topic, self.queue_size)
        self.object_metadata_pubber_ = self.create_publisher(ObjectMetaData, self.object_metadata_topic, self.queue_size)
        self.surface_pub = self.create_publisher(MarkerArray, 'surface_marker_array', self.queue_size)
        self.surface_detected_pub = self.create_publisher(DetectedSurfaces, 'surface_detected', self.queue_size)
        self.objects_pub = self.create_publisher(MarkerArray,'object_markers', self.queue_size)
        self.object_detected_pub = self.create_publisher(DetectedObjects,'object_detected', self.queue_size)
        self.object_marker_arr_pubbers_ = {}
        self.pose_marker_pub = self.create_publisher(MarkerArray, "/object_centroid_markers", self.queue_size)


        # tf_broadcaster init
        self.latest_object_pose = None
        self.latest_object_uid = None
        self.latest_object_color = 'unknown'
        self.latest_bbox_img_msgs = {int: Image()}
        self.dep_col_ext_tf = None

        # clocks
        self.object_cloud_pub_timer = self.create_timer(0.01, self.publish_object_cloud)
        self.object_pose_tf_pub_timer = self.create_timer(0.01, self.publish_object_tf)
        self.world_camera_static_tf_pub_timer = self.create_timer(0.01, self.publish_world_camera_static_tf)
        self.get_tf_timer = self.create_timer(0.01, self.get_transform)

        # sanity checks
        self.color_img_size = [0, 0] # width, height
        self.depth_img_size = [0, 0] # height, width

        # intrinsics
        # P
        self.projection_matrix = np.zeros((3, 4)) 
        self.rs_fx = 0
        self.rs_fy = 0
        self.rs_cx = 0
        self.rs_cy = 0
        self.rs_Tx = 0
        self.rs_Ty = 0

        # K
        self.camera_matrix = np.zeros((3, 3))
        self.rs_fx_k = 0
        self.rs_fy_k = 0
        self.rs_cx_k = 0
        self.rs_cy_k = 0
        self.baseline = 0 # should be about 50 mm

        # extrinsics
        self.world_to_camera_link_tf_vec = self.get_parameter("world_to_camera_tf").value
        self.H_world_to_camera_link = spmath.SE3.RTvec(rvec=self.world_to_camera_link_tf_vec[:3], tvec=self.world_to_camera_link_tf_vec[3:])
        self.Rot_world_camera_link = np.array(self.H_world_to_camera_link.eul('rad'))
        self.p_world_camera_link = np.array(self.H_world_to_camera_link.t)

        # Outside for dyn
        self.world_to_camera_link_static_tf = None
        self.world_to_rs_pc_frame_tf = None
        
        # pc stuff
        self.detected_object_centroids = []
        self.detected_object_dimensions = []

        # messages
        self.yolo_inference = YoloInference()
        self.yolo_inference.detections = []
        self.object_uids = []
        self.object_metadata_arr = []

        # object pose handling
        self.object_poses = []
        self.latest_color_image_msg = None
        self.latest_depth_image_msg = None

        # color and depth image buffer
        self.color_and_depth_msgs = {'color': {}, 'depth': {}}  

        # actions
        self.get_obj_pose_action_ = ActionServer(
            self,
            GetTargetObjPose,
            'get_target_obj_pose',
            self.execute_pose_estimation_callback
        )
        
    def publish_world_camera_static_tf(self):
        if self.world_to_camera_link_static_tf is not None:
            self.world_camera_static_tf_pub_timer.cancel()
            self.get_logger().info('Got tf successfully. Stopping timer.')
            return

        try:
            now = rclpy.time.Time()
            self.world_to_camera_link_static_tf = self.tf_buffer.lookup_transform(
            target_frame=self.world_frame_id,
            source_frame=self.camera_link_frame_id,
            time=now,
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
            self.get_logger().info(f"Successfully got transform from {self.camera_link_frame_id} to {self.world_frame_id}")
            self.world_camera_static_tf_pub_timer.cancel()
        except LookupException as e:
            self.get_logger().warning('Failed to get transform {} \n'.format(repr(e)))
            cam_pose = self.world_to_camera_link_tf_vec
            cam_pose_quat = euler2quat(
                cam_pose[0],
                cam_pose[1],
                cam_pose[2],
                'sxyz'
            )
            cam_child_frame_id = self.camera_link_frame_id
            cam_tr = TransformStamped()
            cam_tr.header.stamp = self.get_clock().now().to_msg()
            cam_tr.header.frame_id = self.world_frame_id
            cam_tr.child_frame_id = cam_child_frame_id

            cam_tr.transform.translation.x = cam_pose[3]
            cam_tr.transform.translation.y = cam_pose[4]
            cam_tr.transform.translation.z = cam_pose[5]
            cam_tr.transform.rotation.x = cam_pose_quat[1]
            cam_tr.transform.rotation.y = cam_pose_quat[2]
            cam_tr.transform.rotation.z = cam_pose_quat[3]
            cam_tr.transform.rotation.w = cam_pose_quat[0]

            self.tf_broadcaster.sendTransform(cam_tr)

    def color_depth_extrinsics_tf_bct(self):
        if self.world_to_camera_link_static_tf is not None:
            self.world_camera_static_tf_pub_timer.cancel()
            self.get_logger().info('Got tf successfully. Stopping timer.')
            return

        try:
            now = rclpy.time.Time()
            self.world_to_camera_link_static_tf = self.tf_buffer.lookup_transform(
            target_frame=self.world_frame_id,
            source_frame=self.camera_link_frame_id,
            time=now,
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
            self.get_logger().info(f"Successfully got transform from {self.camera_link_frame_id} to {self.world_frame_id}")
            self.world_camera_static_tf_pub_timer.cancel()
        except LookupException as e:
            self.get_logger().warning('Failed to get transform {} \n'.format(repr(e)))
            # get cam pose (dyn)
            cam_pose = self.world_to_camera_link_tf_vec
            cam_pose_quat = euler2quat(
                cam_pose[0],
                cam_pose[1],
                cam_pose[2],
                'sxyz'
            )
            cam_child_frame_id = self.camera_link_frame_id
            cam_tr = TransformStamped()
            cam_tr.header.stamp = self.get_clock().now().to_msg()
            cam_tr.header.frame_id = self.world_frame_id
            cam_tr.child_frame_id = cam_child_frame_id

            cam_tr.transform.translation.x = cam_pose[3]
            cam_tr.transform.translation.y = cam_pose[4]
            cam_tr.transform.translation.z = cam_pose[5]
            cam_tr.transform.rotation.x = cam_pose_quat[1]
            cam_tr.transform.rotation.y = cam_pose_quat[2]
            cam_tr.transform.rotation.z = cam_pose_quat[3]
            cam_tr.transform.rotation.w = cam_pose_quat[0]

            self.tf_broadcaster.sendTransform(cam_tr)

    def get_transform(self):
        if self.world_to_rs_pc_frame_tf is not None:
            self.tf_timer.cancel()
            self.get_logger().info('Got tf successfully. Stopping timer.')
            return
        if self.latest_object_uid is not None:
            try:
                now = rclpy.time.Time()
                self.world_to_rs_pc_frame_tf = self.tf_buffer.lookup_transform(
                target_frame=self.world_frame_id,
                source_frame=self.latest_object_uid.frame_id,
                time=rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
                self.get_logger().info(f"Successfully got transform from {self.latest_object_uid.frame_id} to {self.world_frame_id}")
            except TransformException as e:
                self.get_logger().warning('Failed to get transform {} \n'.format(repr(e)))

            
    def rs_infra_right_info_callback(self, infra_right_msg):
        rows, cols = self.projection_matrix.shape
        for row_idx in range(rows):
            self.projection_matrix[row_idx, :] = infra_right_msg.p[cols*row_idx:(cols*row_idx)+cols]
            self.camera_matrix[row_idx, :] = infra_right_msg.k[(cols-1)*row_idx:((cols-1)*row_idx)+cols-1]

        self.rs_fx, self.rs_cx, self.Tx, self.rs_fy, self.rs_cy, self.rs_Ty = utils.get_rs_matrix_params(self.projection_matrix, "proj")
        self.rs_fx_k, self.rs_cx_k, self.rs_fy_k, self.rs_cy_k, = utils.get_rs_matrix_params(self.camera_matrix, "cam")
        self.baseline = abs(self.Tx/self.rs_fx)
  
    def rs_color_callback(self, msg):
        """
        Main callback for YOLOv8n object detection.
        """
        self.color_img_size = [msg.width, msg.height]
        color_image_cv = self.bridge.imgmsg_to_cv2(msg, self.bridge_color_img_encoding)
        color_image_cv_rgb = cv2.cvtColor(color_image_cv, cv2.COLOR_BGR2RGB)  # convert to RGB format
        results = self.yolo_model(color_image_cv_rgb, verbose=False)

        self.yolo_inference.header.frame_id = "inference"
        self.yolo_inference.header.stamp = self.get_clock().now().to_msg()

        # Clear previous detections
        self.yolo_inference.detections.clear()
        self.object_uids.clear()
        self.object_metadata_arr.clear()

        # store results in custom interfaces
        for result in results:
            bboxes = result.boxes
            for idx, bbox in enumerate(bboxes):
                inference_result = InferenceResult()
                object_uid = ObjectUId()

                # get YOLO class
                cls_id = int(bbox.cls)
                cls_name = self.yolo_model.names[cls_id].replace(" ", "_").lower() # .strip() will not gut underscores so we're fine
                object_uid.class_name = cls_name
                object_uid.id = idx

                # get color
                bbox_xywh = bbox.xywh[0].to('cpu').detach().numpy().copy()  
                bbox_img = utils.get_bbox_img(color_image_cv, bbox_xywh)
                bbox_img_msg = self.bridge.cv2_to_imgmsg(np.array(bbox_img, dtype=np.uint8), encoding='bgr8')
                self.latest_bbox_img_msgs[idx] = bbox_img_msg
                dominant_color_bgr, obj_color = self.get_dominant_color_rng(idx)
                object_uid.color = obj_color
                object_uid.frame_id = f"object_uid_{obj_color.strip().lower()}_{cls_name.replace(' ', '_').lower()}_id_{idx}"

                # cache object uid publishers to av
                if object_uid.frame_id not in list(self.object_uid_publishers.keys()):
                    self.object_uid_publishers[object_uid.frame_id] = self.create_publisher(
                        ObjectUId, object_uid.frame_id, self.queue_size
                    )
                object_uid_pubber_= self.object_uid_publishers[object_uid.frame_id]
                object_uid_pubber_.publish(object_uid)
                self.object_uids.append(object_uid)

                inference_result.object_uid = object_uid
                inference_result.bbox = bbox_xywh.astype(np.float32).tolist() # [cx, cy, w, h]
                inference_result.conf = float(bbox.conf[0])
                self.yolo_inference.detections.append(inference_result)


                # create annotated image with boxes and labels and publish 
                corners =  utils.get_bbox_corners(bbox_xywh) # bottom-left to top-left in a counterclockwise direction 
                ann_color_image_cv = cv2.rectangle(color_image_cv, 
                                                  corners[0], corners[2], 
                                                  dominant_color_bgr, 
                                                  self.get_parameter('bbox_rect_line_width').value)
                label = f"{cls_name}"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8
                thickness = 2

                # put text near top-left corner of the bbox
                text_pos = (corners[0][0], corners[0][1] - 10)
                cv2.putText(
                  ann_color_image_cv, 
                  label, 
                  text_pos, 
                  font, 
                  font_scale, 
                  dominant_color_bgr, 
                  thickness, 
                  lineType=cv2.LINE_AA
                )
                ann_color_image_cv_rgb = cv2.cvtColor(ann_color_image_cv, cv2.COLOR_BGR2RGB)
                ann_color_image_msg = self.bridge.cv2_to_imgmsg(ann_color_image_cv_rgb, 'rgb8')
                self.latest_color_image_msg = Image()
                self.latest_color_image_msg.header = msg.header
                self.latest_color_image_msg.height = msg.height
                self.latest_color_image_msg.width = msg.width
                self.latest_color_image_msg.encoding = 'rgb8'
                self.latest_color_image_msg.is_bigendian = msg.is_bigendian
                self.latest_color_image_msg.step = msg.step
                self.latest_color_image_msg.data = color_image_cv_rgb.tobytes()
                self.annotated_img_pubber_.publish(ann_color_image_msg)
        
        self.yolo_detections_pubber_.publish(self.yolo_inference)
        self.get_logger().info(f"Published {len(self.yolo_inference.detections)} detections.")

    def get_dominant_color_rng(self, det_idx: int):
        """
        Simpler method for color detection by testing against a known range of basic color values.

        Uses raw Image data to be exact. Detects the dominant color at the center pixel of an RGB image.
        Assumes 'bgr8' encoding.
        """
        img_msg = self.latest_bbox_img_msgs[det_idx]

        if img_msg is None or img_msg.encoding != 'bgr8':
            self.get_logger().warning(f"Unsupported encoding: {img_msg.encoding}")
            return (0, 0, 0), 'unknown'

        width = img_msg.width
        height = img_msg.height
        center_x = width // 2
        center_y = height // 2
        byte_depth = 3 

        # avoid index errors
        if width == 0 or height == 0:
            self.get_logger().warning("Invalid image dimensions")
            return (0, 0, 0), 'unknown'

        index = (center_y * width + center_x) * byte_depth
        data = img_msg.data
        if isinstance(data, list):
            data = bytes(data) # convert to bytes if list

        # check if we have enough in the image stream
        if index + 2 >= len(data):
            self.get_logger().warning("Index out of bounds for image data")
            return (0, 0, 0), 'unknown'

        b = data[index]
        g = data[index + 1]
        r = data[index + 2]

        # define color thresholds and get matching color
        color_thresholds = {
            'red': (r > 160 and g < 80 and b < 80),
            'yellow': (r > 200 and g > 200 and b < 100),
            'green': (r < 100 and g > 180 and b < 100),
            'blue': (r < 100 and g < 150 and b > 165)
        }
        
        detected_color = 'unknown'
        for color_name, condition in color_thresholds.items():
            if condition:
                detected_color = color_name
                break
        
        if detected_color != 'unknown':
            self.latest_object_color = detected_color
        
        self.get_logger().debug(f"RGB values: ({r}, {g}, {b}); detected color: {detected_color}")
        
        return (b, g, r), detected_color
    
    def rs_depth_callback(self, depth_msg):
        """
        Callback for depth info retrieval
        """
        self.depth_img_size = [depth_msg.width, depth_msg.height]
        # self.get_logger().info(f"Depth Image Size: {self.depth_img_size}")
        try:
            assert self.depth_img_size == self.color_img_size

        except Exception as e:
            self.get_logger().warning(f'\n Possible size mismatch. Are the depth and color images aligned?{e}')
        self.latest_depth_image_msg = self.bridge.imgmsg_to_cv2(depth_msg, self.bridge_depth_img_encoding)
        self.latest_depth_image_msg = self.latest_depth_image_msg.astype(np.float32) / 1000.0  # convert to meters
  
    def yolo_and_depth_callback(self, msg):
        if self.latest_color_image_msg is None or self.latest_depth_image_msg is None:
            self.get_logger().warning("No RGB or depth image yet")
            return

        # convert full image to OpenCV format
        rgb_image = self.bridge.imgmsg_to_cv2(self.latest_color_image_msg, desired_encoding='rgb8')
        depth_image = self.latest_depth_image_msg  

        image_height, image_width, _ = rgb_image.shape

        self.color_and_depth_msgs['color'] = {}
        self.color_and_depth_msgs['depth'] = {}

        for idx, inf in enumerate(self.yolo_inference.detections):
            bbox = inf.bbox  # xywh format
            cx, cy, w, h = bbox
            x1 = int(cx - w/2)
            y1 = int(cy - h/2)
            x2 = int(cx + w/2)
            y2 = int(cy + h/2)

            # clamp to image bounds
            x1 = max(0, min(image_width - 1, x1))
            x2 = max(0, min(image_width - 1, x2))
            y1 = max(0, min(image_height - 1, y1))
            y2 = max(0, min(image_height - 1, y2))

            # crop
            cropped_img = rgb_image[y1:y2, x1:x2]
            cropped_depth = depth_image[y1:y2, x1:x2]

            # Store in dictionary using the object's index as key
            self.color_and_depth_msgs['color'][idx] = cropped_img
            self.color_and_depth_msgs['depth'][idx] = cropped_depth

    def publish_object_tf(self):
        if self.latest_object_uid is None:
            # self.get_logger().warning("No object tf available to publish yet!")
            return
        
        object_pose = self.latest_object_pose
        object_uid = self.latest_object_uid
        obj_child_frame_id = object_uid.frame_id
        obj_tr = TransformStamped()
        obj_tr.header.stamp = self.get_clock().now().to_msg()
        obj_tr.header.frame_id = self.rs_pc_frame_id
        obj_tr.child_frame_id = obj_child_frame_id

        obj_tr.transform.translation.x = object_pose.pose.position.x
        obj_tr.transform.translation.y = object_pose.pose.position.y
        obj_tr.transform.translation.z = object_pose.pose.position.z
        obj_tr.transform.rotation.x = object_pose.pose.orientation.x
        obj_tr.transform.rotation.y = object_pose.pose.orientation.y
        obj_tr.transform.rotation.z = object_pose.pose.orientation.z
        obj_tr.transform.rotation.w = object_pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(obj_tr)

    
    def read_byte_stream(self, data: bytes, offset:int) -> float:
        """
        Helper method for returning int byte-stream as float.
        """
        return struct.unpack_from('f', buffer=data, offset=offset)[0]

    def create_marker_array(self, pcd_msg:PointCloud2, points: List[Point], color: tuple) -> MarkerArray:
        """
        Returns a MarkerArray object corresponding to the extracted object cloud
        """
        r, g, b = color
        marker_array = MarkerArray()
        for idx, point in enumerate(points):
            marker = Marker()
            marker.header = pcd_msg.header
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.r = r/255.0
            marker.color.g = g/255.0
            marker.color.b = b/255.0
            marker.color.a = 1.0
            marker.pose.position = point
            marker.id = idx
            marker_array.markers.append(marker)

        return marker_array
    
    def extract_points(self, pcd_msg:PointCloud2):
        """
        Returns a list of Point() objects corresponding to the input PointCloud message.
        """
        points_raw = pcd_msg.data
        pcd_height = pcd_msg.height
        points = []

        # check if pcd msg is ordered or not
        if int(pcd_msg.height) == 1:
            step_size = int(len(points_raw) // self.num_pcd_points) # we divide the byte stream into a number of equal byte parts 
        else:
            step_size = int((len(points_raw) / pcd_height) // self.num_pcd_points)

        
        for i in range(0, len(points_raw), step_size):
            if len(points) >=  self.num_pcd_points:
                break # get only the first self.num_pcd_points points

            idx = i
            px = self.read_byte_stream(points_raw, idx)
            py = self.read_byte_stream(points_raw, idx + 4) # read y
            pz = self.read_byte_stream(points_raw, idx + 8) # read z

            p = Point()
            p.x, p.y, p.z = px, py, pz
            points.append(p)
        return points

    def publish_object_cloud(self):
        """
        Returns the pointcloud corresponding to the detected object.
        """
        try:
            if not self.color_and_depth_msgs['color'] or not self.color_and_depth_msgs['depth']:
                self.get_logger().warning("No cropped color or depth images available.")
                return
            idx = next(iter(self.color_and_depth_msgs['color']))
            rgb_image = self.color_and_depth_msgs['color'][idx]
            depth_image = self.color_and_depth_msgs['depth'][idx]
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
        
        point_cloud = utils.compute_colored_point_cloud(depth_image, rgb_image, self.camera_matrix, pack_rgb=True) # N x 4 due to rgb byte packing

        pc2_msg = self.point_cloud_to_ros_msg(point_cloud, self.rs_pc_frame_id) 
        self.object_cloud_pubber_.publish(pc2_msg)

    def ros_msg_to_point_cloud(self, msg: PointCloud2) -> Union[o3d.geometry.PointCloud, None]:
        """Converts a ROS PointCloud2 message to an o3d point cloud"""
        try:
            transform = self.tf_buffer.lookup_transform('base_link',
                                                        msg.header.frame_id,
                                                        rclpy.time.Time(),
                                                        timeout=rclpy.time.Duration(seconds=1.0))
            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            rotation_quaternion = np.array([transform.transform.rotation.x,
                                            transform.transform.rotation.y,
                                            transform.transform.rotation.z,
                                            transform.transform.rotation.w])

            # convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(rotation_quaternion)

            # convert PointCloud2 msg to numpy array
            point_step = msg.point_step
            num_points = len(msg.data) // point_step
            points = []
            for i in range(num_points):
                start_index = i * point_step
                x_bytes = msg.data[start_index:start_index + 4]
                y_bytes = msg.data[start_index + 4:start_index + 8]
                z_bytes = msg.data[start_index + 8:start_index + 12]
                x = np.frombuffer(x_bytes, dtype=np.float32)[0]
                y = np.frombuffer(y_bytes, dtype=np.float32)[0]
                z = np.frombuffer(z_bytes, dtype=np.float32)[0]
                point = np.array([x, y, z])

                # apply the rotation to the point
                rotated_point = np.dot(rotation_matrix, point)

                # apply the translation to the rotated point to get its position relative to the base_link frame
                relative_point = rotated_point + translation

                points.append(relative_point)

            data = np.array(points, dtype=np.float32)
            assert data.shape[1] == 3, "Number of fields must be 3"
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(data)
            return cloud

        except (TransformException, ConnectivityException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in from_ros_msg: {e}")
            return None

    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Converts a quaternion to a rotation matrix"""
        x, y, z, w = q
        rotation_matrix = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                                    [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                                    [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
        return rotation_matrix

    def point_cloud_to_ros_msg(self, cloud, frame_id):
        header = self.create_header(frame_id)
        fields = [
            self.create_pointfield('x', 0, 7, 1),
            self.create_pointfield('y', 4, 7, 1),
            self.create_pointfield('z', 8, 7, 1),
            self.create_pointfield('rgb', 12, 7, 1)
        ] # 16 bytes per point

        pc2_msg = point_cloud2.create_cloud(
            header=header,
            fields=fields,
            points=cloud.tolist()
        )
        return pc2_msg

    def create_header(self, frame_id):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def create_pointfield(self, name, offset, datatype, count):
        pointfield = PointField()
        pointfield.count = count
        pointfield.datatype = datatype
        pointfield.name = name
        pointfield.offset = offset

        return pointfield
    
    def pub_surface_detected(self, centroids: List[List[float]], dimensions: List[List[float]]) -> None:
        """Publishes the detected surface information"""
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            surface_msg = DetectedSurfaces()
            surface_msg.surface_id = idx
            surface_msg.position.x = centroid[0]
            surface_msg.position.y = centroid[1]
            surface_msg.position.z = centroid[2]
            surface_msg.height = dimension[0]
            surface_msg.width = dimension[1]
            self.surface_detected_pub.publish(surface_msg)
    
    def pub_surface_marker(self, surface_centroids: List[List[float]], surface_dimensions: List[List[float]]) -> None:
        """Publishes the detected plane as cube markers"""
        marker_array = MarkerArray()

        for idx, (centroid, dimensions) in enumerate(zip(surface_centroids, surface_dimensions)):
            length = float(dimensions[0])
            width = float(dimensions[1])

            cube_marker = Marker()
            cube_marker.header.frame_id = "base_link"
            cube_marker.id = idx
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.pose.position.x = centroid[0]
            cube_marker.pose.position.y = centroid[1]
            cube_marker.pose.position.z = centroid[2] - self.surface_thickness / 2  
            cube_marker.pose.orientation.w = 1.0

            cube_marker.scale.x = length
            cube_marker.scale.y = width
            cube_marker.scale.z = self.surface_thickness

            cube_marker.color.r = 0.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0
            cube_marker.color.a = 0.5  
            marker_array.markers.append(cube_marker)

        if marker_array.markers:
            self.get_logger().info(f"Published {len(marker_array.markers)} surface plane markers")
            self.surface_pub.publish(marker_array)
        else:
            self.get_logger().warning("No surface plane markers to publish.")

    def pub_object_marker(self, object_centroids: List[List[float]], object_dimensions: List[List[float]]) -> None:
        """Publishes objects on flat surfaces as markers"""
        marker_array = MarkerArray()

        for idx, (centroid, dimensions) in enumerate(zip(object_centroids, object_dimensions)):

            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid[0]) + self.offset_x
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = dimensions[0]
            marker.scale.y = dimensions[1]
            marker.scale.z = dimensions[2]

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            # object dimension sanity check; fat box bug
            max_dim = 0.2  # max 20cm in any direction
            min_dim = 0.05 # min 5cm in any direction

            if any(d > max_dim or d < min_dim for d in dimensions):
                self.get_logger().warn(f"Skipping object {idx} due to invalid size: {dimensions}")
                continue
            # store centroids and dimensions
            self.detected_object_centroids.append(centroid)
            self.detected_object_dimensions.append(dimensions)

            # create and append marker object
            marker_array.markers.append(marker)

        if marker_array.markers:
            self.get_logger().info(f"Published {len(marker_array.markers)} objects on flat surface markers!")
            self.objects_pub.publish(marker_array)
            self.publish_centroid_pose_markers(self.detected_object_centroids)

        else:
            self.get_logger().warning("No objects on flat surface markers to publish.")

    def pub_object_detected(self, centroids: List[List[float]], dimensions: List[List[float]]) -> None:
        """Publishes the detected surface information"""
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            object_msg = DetectedObjects()
            object_msg.object_id = idx
            object_msg.position.x = centroid[0]
            object_msg.position.y = centroid[1]
            object_msg.position.z = centroid[2]
            object_msg.height = dimension[0]
            object_msg.width = dimension[1]
            object_msg.thickness = dimension[2]
            self.object_detected_pub.publish(object_msg)


    def publish_sf_obj_pcd_det_cb(self, msg):
        pcd = self.ros_msg_to_point_cloud(msg) 
        voxel_size = 0.01
        if pcd is not None:
            downpcd = pcd.voxel_down_sample(voxel_size)

            # filter
            filtered_cloud_plane = utils.filter_cloud(downpcd, self.max_x_dist, self.min_height_sf, self.max_height_sf)

            # segment
            plane_indices, _, plane_cloud = utils.extract_plane(filtered_cloud_plane, dist_thresh=0.02)

            # remove plane points from the full downsampled cloud to get candidate objects
            non_plane_cloud = downpcd.select_by_index(plane_indices, invert=True)

            # filter objects by height on non-plane cloud
            filtered_cloud_objects = utils.filter_cloud(non_plane_cloud, self.max_x_dist, self.min_height_obj, self.max_height_obj)

            # clustering: Identify clusters corresponding to objects placed on top of a flat surface
            _, surface_centroids, surface_dimensions = utils.extract_cloud_clusters(plane_cloud, "Surface")
            _, object_centroids, object_dimensions = utils.extract_cloud_clusters(filtered_cloud_objects, "Object")

            # publish all the things
            self.pub_surface_marker(surface_centroids, surface_dimensions)
            self.pub_object_marker(object_centroids, object_dimensions)
            self.pub_surface_detected(surface_centroids, surface_dimensions)
            self.pub_object_detected(object_centroids, object_dimensions)

    def publish_centroid_pose_markers(self, centroids: List[List[float]], frame_id: str = "base_link"):
        marker_array = MarkerArray()
        axis_length = 0.1  # arrow length
        axis_thickness = 0.01  # arrow diameter
        for idx, centroid in enumerate(centroids):
            x, y, z = centroid[0] + self.offset_x, centroid[1], centroid[2]

            # define RGB direction vectors
            directions = {
                "x": ([1, 0, 0], (1.0, 0.0, 0.0)), 
                "y": ([0, 1, 0], (0.0, 1.0, 0.0)),  
                "z": ([0, 0, 1], (0.0, 0.0, 1.0))   
            }

            for axis_id, (vec, color) in enumerate(directions.values()):
                start_point = [x, y, z]
                end_point = [x + axis_length * vec[0],
                            y + axis_length * vec[1],
                            z + axis_length * vec[2]]

                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"object_pose_axes"
                marker.id = idx * 10 + axis_id  

                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                marker.points = [
                    Point(x=start_point[0], y=start_point[1], z=start_point[2]),
                    Point(x=end_point[0], y=end_point[1], z=end_point[2])
                ]

                marker.scale.x = axis_thickness  # shaft diameter
                marker.scale.y = 2 * axis_thickness  # head diameter
                marker.scale.z = 2 * axis_thickness  # head length

                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0

                marker_array.markers.append(marker)

        self.pose_marker_pub.publish(marker_array)

    def execute_pose_estimation_callback(self, goal_handle):
        """
        Action server that serves the pose of the text-described target object.
        E.g., 'red cup'
        """
        # parse the target object description
        target_description = goal_handle.request.target_obj_description.data.lower().strip()

        tokens = target_description.split()
        result = GetTargetObjPose.Result()
        feedback = GetTargetObjPose.Feedback()
        if len(tokens) < 2:
            self.get_logger().error("Invalid target object description. Expected format: '<color> <class_name>'")
            feedback.target_obj_found = 0
            goal_handle.publish_feedback(feedback)
            result.target_obj_pose = PoseStamped()
            goal_handle.abort() 
            return result

        class_name, color = " ".join(tokens[1:]), tokens[0]

        # find matching detection in set of detections
        found_idx = None
        for idx, obj_uid in enumerate(self.object_uids):
            obj_conf = self.yolo_inference.detections[idx].conf
            # if obj_conf >= 0.6:
            if obj_uid.color.lower() == color and obj_uid.class_name.lower().replace(' ', '_') == class_name:
                found_idx = idx
                break   

        if found_idx is None:
            feedback.target_obj_found = 0
            result.target_obj_pose = PoseStamped()            
            goal_handle.publish_feedback(feedback)
            self.get_logger().warning(f"Target object '{target_description}' not found.")
            goal_handle.abort()
            return result

        # extract object point cloud 
        rgb_crop = self.color_and_depth_msgs['color'].get(found_idx, None)
        depth_crop = self.color_and_depth_msgs['depth'].get(found_idx, None)
        if rgb_crop is None or depth_crop is None:
            feedback.target_obj_found = 0
            result.target_obj_pose = PoseStamped()
            goal_handle.publish_feedback(feedback)
            self.get_logger().warning(f"No RGB or depth images received!")
            goal_handle.abort()
            return result

        # convert cropped RGBD to point cloud in camera frame
        obj_pc = utils.compute_colored_point_cloud(depth_crop, rgb_crop, self.camera_matrix, pack_rgb=False)
        obj_pc_rgb_packed = utils.compute_colored_point_cloud(depth_crop, rgb_crop, self.camera_matrix, pack_rgb=True)
        
        # convert object cloud to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(obj_pc[:, :3])  # Nx3

        # transform points into world frame
        obj_cloud = np.asarray(pcd.points, dtype=np.float32)

        # get original RGB and re-attach it to complete cloud
        rgb = obj_pc_rgb_packed[:, 3]  

        # combine into (N, 4): [x, y, z, rgb]
        packed_obj_cloud = np.hstack((obj_cloud, rgb.reshape(-1, 1)))

        pc2_msg_packed = self.point_cloud_to_ros_msg(packed_obj_cloud, self.rs_pc_frame_id) 
        self.object_cloud_pubber_.publish(pc2_msg_packed)

        # fit oriented bounding box to object cloud o3d PointCloud() object
        obb = pcd.get_oriented_bounding_box()
        obj_t = obb.center
        obj_R = obb.R  

        # tf handles frame transformation math
        H_obj_in_cam = spmath.SE3.RTvec(rvec=spmath.SO3(obj_R).eul('rad'), tvec=obj_t)

        # fill PoseStamped (corresponded depth + OBB method; buggy due to OBB rotation bug: )
        # pose_msg = PoseStamped()
        # pose_msg.header = self.create_header(self.world_frame_id)
        # pose_msg.pose.position.x = float(H_obj_in_cam.t[0])
        # pose_msg.pose.position.y = float(H_obj_in_cam.t[1])
        # pose_msg.pose.position.z = float(H_obj_in_cam.t[2])

        # Use the centroid as the estimated pose (drawback: AABB; not invariant to rotations!)
        if found_idx >= len(self.detected_object_centroids):
            self.get_logger().error("Found index exceeds number of detected object centroids.")
            goal_handle.abort()
            return GetTargetObjPose.Result()

        centroid = self.detected_object_centroids[found_idx]

        pose_msg = PoseStamped()
        pose_msg.header = self.create_header(self.world_frame_id)
        pose_msg.pose.position.x = float(centroid[0]) + self.offset_x
        pose_msg.pose.position.y = float(centroid[1])
        pose_msg.pose.position.z = float(centroid[2])

        pose_msg.pose.orientation.w = 1.0  # this only works for upright objects

        rot = R.from_matrix(H_obj_in_cam.R)
        quat = rot.as_quat() # scalar_first=True fails, order is x, y, z, w by default
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        # return result
        object_metadata = ObjectMetaData()
        object_metadata.object_uid = self.object_uids[found_idx] 
        object_metadata.object_pose = pose_msg
        object_metadata.header = pose_msg.header
        feedback.target_obj_found = 1
        goal_handle.publish_feedback(feedback)
        result.target_obj_pose = pose_msg
        self.latest_object_pose = pose_msg
        self.latest_object_uid = self.object_uids[found_idx]
        goal_handle.succeed()
        self.get_logger().info(f"Found target object '{target_description}' with pose: {pose_msg.pose.position.x}, {pose_msg.pose.position.y}, {pose_msg.pose.position.z}")
        self.object_poses.append(pose_msg)
        self.object_metadata_arr.append(object_metadata)
        self.object_pose_pubber_.publish(pose_msg)
        self.object_metadata_pubber_.publish(object_metadata)
        self.get_logger().info(f"Published pose for target object: {color} {class_name}, pose: {np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])}.")
        return result

def main(args=None):
  rclpy.init(args=args)
  yolo_pose_estimator = YoloPoseEstimationActionServer()
  rclpy.spin(yolo_pose_estimator)
  yolo_pose_estimator.destroy_node()
  rclpy.shutdown()

if __name__=="__main__":
  main()
