#!/usr/bin/env python3

"""
Point Cloud Segmentation Node for UR3e + Hand-E Perception

ros2 run ur3e_hande_perception pc_segmentation_node.py --ros-args \
  -p rs_pc_topic:=/rgbd_camera/points \
  -p rs_pc_frame_id:=camera_depth_frame 
"""
import os, struct, importlib.util
import numpy as np
import open3d as o3d
import pye57
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from sensor_msgs_py import point_cloud2
from typing import List, Union
from tf2_ros import TransformBroadcaster, TransformException, ConnectivityException
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory

#  utils load
package_name = 'ur3e_hande_perception'
share = get_package_share_directory(package_name)
utils_path = os.path.join(share, 'utils', 'perc_utils.py')
spec = importlib.util.spec_from_file_location("perc_utils", utils_path)
utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utils)

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

class PCSegmentationNode(Node):
    def __init__(self):
        super().__init__("pc_segmentation_node")
        self.tf_buffer = None  # disable TF lookups for pure PCD mode

        #  parameters 
        self.declare_parameter('rs_pc_topic', '/rgbd_camera/points')
        self.declare_parameter('rs_pc_frame_id', 'camera_depth_frame')
        self.declare_parameter('voxel_size', 0.5)
        self.declare_parameter('max_x_dist', 0.45)
        self.declare_parameter('min_height_sf', 0.02)
        self.declare_parameter('max_height_sf', 0.15)
        self.declare_parameter('min_height_obj', 0.05)
        self.declare_parameter('max_height_obj', 0.30)
        self.declare_parameter('surface_thickness', 0.05)
        self.declare_parameter('offset_x', 0.05)
        self.declare_parameter('num_pcd_points', 20)  # used by extract_points()

        #  get values 
        self.rs_pc_topic = self.get_parameter('rs_pc_topic').value
        self.rs_pc_frame_id = self.get_parameter('rs_pc_frame_id').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_x_dist = float(self.get_parameter('max_x_dist').value)
        self.min_height_sf = float(self.get_parameter('min_height_sf').value)
        self.max_height_sf = float(self.get_parameter('max_height_sf').value)
        self.min_height_obj = float(self.get_parameter('min_height_obj').value)
        self.max_height_obj = float(self.get_parameter('max_height_obj').value)
        self.surface_thickness = float(self.get_parameter('surface_thickness').value)
        self.offset_x = float(self.get_parameter('offset_x').value)
        self.num_pcd_points = int(self.get_parameter('num_pcd_points').value)

        #  pubs/subs
        self.create_subscription(PointCloud2, self.rs_pc_topic, self.pcd_cb, QOS_PROFILE)
        self.surface_pub = self.create_publisher(MarkerArray, 'surface_marker_array', 10)
        self.objects_pub = self.create_publisher(MarkerArray, 'object_marker_array', 10)
        self.object_pose_pubber_ = self.create_publisher(PoseStamped, "/object_pose", 10)
        self.object_cloud_pubber_ = self.create_publisher(PointCloud2, "/object_cloud", 10)

        # pubs
        self.surface_detected_pub = self.create_publisher(MarkerArray, 'surface_detected_dummy', 10)  # 
        self.object_detected_pub = self.create_publisher(MarkerArray, 'object_detected_dummy', 10)   # 
        self.pose_marker_pub = self.create_publisher(MarkerArray, "/object_centroid_markers", 10)

        #  TF 
        self.tf_broadcaster = TransformBroadcaster(self)

        #  runtime state used by your helpers 
        self.latest_object_pose: Union[PoseStamped, None] = None
        self.latest_object_uid = type("UID", (), {})()
        self.latest_object_uid.frame_id = None

        # for compatibility with your methods
        self.color_and_depth_msgs = {'color': {}, 'depth': {}}  # unused in pure-PCD path
        self.camera_matrix = np.zeros((3, 3))  # unused, but required by signature

        self.detected_object_centroids: List[List[float]] = []
        self.detected_object_dimensions: List[List[float]] = []

        self.get_logger().info(f"PCSegmentationNode listening on {self.rs_pc_topic} (frame: {self.rs_pc_frame_id})")

    # main callback
    def pcd_cb(self, msg: PointCloud2):
        # Convert ROS ---> O3D
        try:
            cloud = self.ros_msg_to_point_cloud(msg)  # must return o3d.geometry.PointCloud in msg frame
        except Exception as e:
            self.get_logger().warn(f"ros_to_o3d conversion error: {e}")
            return

        if cloud is not None:
            if len(cloud.points) == 0:
                self.get_logger().warn("Empty point cloud")
                return

            # Downsample 
            vx = self.voxel_size if self.voxel_size and self.voxel_size > 1e-4 else 0.005
            cloud = cloud.voxel_down_sample(voxel_size=vx)

            # Filter to ROI (x depth band + z height)
            # Here we use a single pass wide band [min_height_sf, max_height_obj], then we’ll split by plane/object later.
            roi_cloud = utils.filter_cloud(cloud, self.max_x_dist, self.min_height_sf, self.max_height_obj)

            if len(roi_cloud.points) == 0:
                self.get_logger().warn("ROI filter removed all points")
                return

            # Segment plane
            plane_idx, plane_eq, plane_cloud = utils.extract_plane(roi_cloud, dist_thresh=0.02)
            non_plane_cloud = roi_cloud.select_by_index(plane_idx, invert=True)

            # Height-slice for surface vs objects
            filtered_cloud_plane = utils.filter_cloud(plane_cloud, self.max_x_dist, self.min_height_sf, self.max_height_sf)
            filtered_cloud_objects = utils.filter_cloud(non_plane_cloud, self.max_x_dist, self.min_height_obj, self.max_height_obj)

            # Cluster both
            _, surface_centroids, surface_dims = utils.extract_cloud_clusters(filtered_cloud_plane, "Surface")
            obj_clusters, object_centroids, object_dims = utils.extract_cloud_clusters(filtered_cloud_objects, "Object", return_clusters=True)

            # Publish markers
            self.pub_surface_marker(surface_centroids, surface_dims)
            self.pub_object_marker(object_centroids, object_dims)

            # Keep state for pose/TF and publish the first object’s cloud
            if object_centroids:
                self.detected_object_centroids = object_centroids
                self.detected_object_dimensions = object_dims

                # choose the largest cluster by size (points)
                best_i = int(np.argmax([len(c.points) for c in obj_clusters]))
                best_centroid = object_centroids[best_i]

                # publish object cloud
                cloud_np = np.asarray(obj_clusters[best_i].points, dtype=np.float32)  # Nx3
                rgb_dummy = np.zeros((cloud_np.shape[0], 1), dtype=np.float32)  # no color -> zeros
                packed = np.hstack((cloud_np, rgb_dummy))  # N x 4 to reuse your point_cloud_to_ros_msg
                self.object_cloud_pubber_.publish(self.point_cloud_to_ros_msg(packed, self.rs_pc_frame_id))

                # build PoseStamped in base frame = rs_pc_frame_id
                pose = PoseStamped()
                pose.header = self.create_header(self.rs_pc_frame_id)
                pose.pose.position.x = float(best_centroid[0]) + self.offset_x
                pose.pose.position.y = float(best_centroid[1])
                pose.pose.position.z = float(best_centroid[2])
                pose.pose.orientation.w = 1.0

                self.latest_object_pose = pose
                self.latest_object_uid.frame_id = f"object_uid_cluster_{best_i}"
                self.object_pose_pubber_.publish(pose)

                # TF broadcast
                self.publish_object_tf()

                # axis markers at centroids
                self.publish_centroid_pose_markers(self.detected_object_centroids, frame_id=self.rs_pc_frame_id)
        else:
            self.latest_object_pose = None
            self.latest_object_uid.frame_id = None

    def publish_object_tf(self):
        if self.latest_object_pose is None or self.latest_object_uid.frame_id is None:
            return
        object_pose = self.latest_object_pose
        obj_child_frame_id = self.latest_object_uid.frame_id
        obj_tr = TransformStamped()
        obj_tr.header.stamp = self.get_clock().now().to_msg()
        obj_tr.header.frame_id = self.rs_pc_frame_id
        obj_tr.child_frame_id = obj_child_frame_id
        obj_tr.transform.translation.x = object_pose.pose.position.x
        obj_tr.transform.translation.y = object_pose.pose.position.y
        obj_tr.transform.translation.z = object_pose.pose.position.z
        obj_tr.transform.rotation = object_pose.pose.orientation
        self.tf_broadcaster.sendTransform(obj_tr)

    def ros_msg_to_point_cloud(self, msg: PointCloud2) -> Union[o3d.geometry.PointCloud, None]:
        """Converts a ROS PointCloud2 message to an Open3D cloud (no TF transform)."""
        try:
            # Convert raw bytes into xyz floats
            point_step = msg.point_step
            num_points = len(msg.data) // point_step
            pts = []
            for i in range(num_points):
                start = i * point_step
                x = np.frombuffer(msg.data[start:start+4], dtype=np.float32)[0]
                y = np.frombuffer(msg.data[start+4:start+8], dtype=np.float32)[0]
                z = np.frombuffer(msg.data[start+8:start+12], dtype=np.float32)[0]
                pts.append([x, y, z])

            pts = np.array(pts, dtype=np.float32)
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(pts)
            return cloud
        except Exception as e:
            self.get_logger().error(f"Error converting PointCloud2: {e}")
            return None


    def load_e57_pointcloud(self, file_path: str, voxel_size: float = 0.01) -> o3d.geometry.PointCloud:
        """
        Loads an .e57 point cloud file using pye57 and returns an Open3D PointCloud.
        """

        e57 = pye57.E57(file_path)
        data = e57.read_scan_raw(0)  # read first scan block

        x = np.array(data["cartesianX"], dtype=np.float32)
        y = np.array(data["cartesianY"], dtype=np.float32)
        z = np.array(data["cartesianZ"], dtype=np.float32)

        # Stack into N x 3 array
        pcd_points = np.column_stack((x, y, z))

        # Create Open3D cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_points)

        # Optional voxel downsampling
        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

        self.get_logger().info(f"Loaded E57 cloud with {len(pcd.points)} points (downsampled).")
        return pcd

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
    
    def read_byte_stream(self, data: bytes, offset:int) -> float:
        return struct.unpack_from('f', buffer=data, offset=offset)[0]

    def create_marker_array(self, pcd_msg:PointCloud2, points: List[Point], color: tuple) -> MarkerArray:
        r, g, b = color
        arr = MarkerArray()
        for idx, p in enumerate(points):
            m = Marker()
            m.header = pcd_msg.header
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.01
            m.color.r = r/255.0; m.color.g = g/255.0; m.color.b = b/255.0; m.color.a = 1.0
            m.pose.position = p
            m.id = idx
            arr.markers.append(m)
        return arr

    def extract_points(self, pcd_msg:PointCloud2):
        points_raw = pcd_msg.data
        pcd_height = pcd_msg.height
        pts = []
        if int(pcd_msg.height) == 1:
            step_size = int(len(points_raw) // self.num_pcd_points)
        else:
            step_size = int((len(points_raw) / pcd_height) // self.num_pcd_points)
        for i in range(0, len(points_raw), step_size):
            if len(pts) >= self.num_pcd_points:
                break
            idx = i
            px = self.read_byte_stream(points_raw, idx)
            py = self.read_byte_stream(points_raw, idx + 4)
            pz = self.read_byte_stream(points_raw, idx + 8)
            p = Point(); p.x, p.y, p.z = px, py, pz
            pts.append(p)
        return pts

    def point_cloud_to_ros_msg(self, cloud, frame_id):
        header = self.create_header(frame_id)
        fields = [
            self.create_pointfield('x', 0, 7, 1),
            self.create_pointfield('y', 4, 7, 1),
            self.create_pointfield('z', 8, 7, 1),
            self.create_pointfield('rgb', 12, 7, 1)
        ]
        return point_cloud2.create_cloud(header=header, fields=fields, points=cloud.tolist())

    def create_header(self, frame_id):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def create_pointfield(self, name, offset, datatype, count):
        f = PointField()
        f.count = count; f.datatype = datatype; f.name = name; f.offset = offset
        return f

    # marker publishers 
    def pub_surface_marker(self, surface_centroids: List[List[float]], surface_dimensions: List[List[float]]) -> None:
        arr = MarkerArray()
        for idx, (c, dims) in enumerate(zip(surface_centroids, surface_dimensions)):
            length, width = float(dims[0]), float(dims[1])
            m = Marker()
            m.header.frame_id = self.rs_pc_frame_id
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = c[0]
            m.pose.position.y = c[1]
            m.pose.position.z = c[2] - self.surface_thickness/2
            m.pose.orientation.w = 1.0
            m.scale.x = length; m.scale.y = width; m.scale.z = self.surface_thickness
            m.color.g = 1.0; m.color.a = 0.5
            arr.markers.append(m)
        if arr.markers:
            self.surface_pub.publish(arr)

    def pub_object_marker(self, object_centroids: List[List[float]], object_dimensions: List[List[float]]) -> None:
        self.detected_object_centroids = []
        self.detected_object_dimensions = []
        arr = MarkerArray()
        for idx, (c, dims) in enumerate(zip(object_centroids, object_dimensions)):
            m = Marker()
            m.header.frame_id = self.rs_pc_frame_id
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(c[0]) + self.offset_x
            m.pose.position.y = float(c[1])
            m.pose.position.z = float(c[2])
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = float(dims[0]), float(dims[1]), float(dims[2])

            max_dim, min_dim = 0.2, 0.05
            if any(d > max_dim or d < min_dim for d in dims):
                continue

            self.detected_object_centroids.append(c)
            self.detected_object_dimensions.append(dims)

            m.color.r = 1.0; m.color.a = 0.5
            arr.markers.append(m)

        if arr.markers:
            self.objects_pub.publish(arr)

    def publish_centroid_pose_markers(self, centroids: List[List[float]], frame_id: str = None):
        if frame_id is None:
            frame_id = self.rs_pc_frame_id
        arr = MarkerArray()
        axis_length = 0.1
        axis_thickness = 0.01
        directions = {
            0: ([1,0,0], (1.0,0.0,0.0)),
            1: ([0,1,0], (0.0,1.0,0.0)),
            2: ([0,0,1], (0.0,0.0,1.0)),
        }
        for idx, c in enumerate(centroids):
            x, y, z = c[0] + self.offset_x, c[1], c[2]
            for axis_id, (vec, col) in directions.items():
                m = Marker()
                m.header.frame_id = frame_id
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "object_pose_axes"
                m.id = idx*10 + axis_id
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.points = [
                    Point(x=x, y=y, z=z),
                    Point(x=x + axis_length*vec[0],
                          y=y + axis_length*vec[1],
                          z=z + axis_length*vec[2])
                ]
                m.scale.x = axis_thickness
                m.scale.y = 2*axis_thickness
                m.scale.z = 2*axis_thickness
                m.color.r, m.color.g, m.color.b = col
                m.color.a = 1.0
                arr.markers.append(m)
        if arr.markers:
            self.pose_marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PCSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
