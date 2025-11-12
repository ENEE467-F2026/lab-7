import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

rclpy.init()
node = Node("tf_listener")
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

try:
    t = tf_buffer.lookup_transform(
        "base_link",                 # target frame
        "camera_depth_optical_frame", # source frame
        rclpy.time.Time(),           # at latest available time
        timeout=rclpy.duration.Duration(seconds=1.0)
    )
    print(t.transform.translation)
    print(t.transform.rotation)
except Exception as e:
    print("TF lookup failed:", e)
