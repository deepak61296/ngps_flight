#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf_transformations
from rclpy.qos import QoSPresetProfiles


class VONode(Node):
    def __init__(self):
        super().__init__('vo_node')
        
        self._cv_bridge = CvBridge()
        self._sift = cv2.SIFT_create(1024)
        self._bf = cv2.BFMatcher(crossCheck=False)
        
        self._cached_ref = None
        self._cached_kps_desc = None
        
        self._tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
        
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('min_matches', 30)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_frame', 'odom')
        
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'vo/pose', 10
        )
        
        self._camera_info = None
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').get_parameter_value().string_value,
            self._camera_info_cb,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self._image_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').get_parameter_value().string_value,
            self._image_cb,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self.get_logger().info('VO node initialized')
    
    def _camera_info_cb(self, msg):
        self._camera_info = msg
    
    def _image_cb(self, msg):
        if self._cached_ref is None:
            self._cached_ref = msg
            return
        
        if self._camera_info is None:
            return
        
        pose = self._compute_pose(msg, self._cached_ref)
        if pose is not None:
            self._pose_pub.publish(pose)
            self._cached_ref = msg
    
    def _compute_pose(self, query, reference):
        qry = self._cv_bridge.imgmsg_to_cv2(query, desired_encoding="mono8")
        ref = self._cv_bridge.imgmsg_to_cv2(reference, desired_encoding="mono8")
        
        kp_qry, desc_qry = self._sift.detectAndCompute(qry, None)
        
        if self._cached_kps_desc is None:
            kp_ref, desc_ref = self._sift.detectAndCompute(ref, None)
        else:
            kp_ref, desc_ref = self._cached_kps_desc
        
        try:
            matches = self._bf.knnMatch(desc_qry, desc_ref, k=2)
        except cv2.error:
            return None
        
        if len(matches) < self.get_parameter('min_matches').get_parameter_value().integer_value:
            return None
        
        threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        good = [m for m, n in matches if m.distance < threshold * n.distance]
        
        if len(good) < self.get_parameter('min_matches').get_parameter_value().integer_value:
            return None
        
        mkp_qry = np.array([kp_qry[m.queryIdx].pt for m in good])
        mkp_ref = np.array([kp_ref[m.trainIdx].pt for m in good])
        
        r, t = self._solve_pnp(mkp_qry, mkp_ref)
        if r is None:
            return None
        
        pose_msg = self._create_pose_msg(query.header.stamp, r, t, mkp_qry, mkp_ref)
        if pose_msg is None:
            return None
        
        self._cached_kps_desc = (kp_qry, desc_qry)
        return pose_msg
    
    def _solve_pnp(self, mkp_qry, mkp_ref):
        k_matrix = np.array(self._camera_info.k).reshape(3, 3)
        dist_coeffs = np.zeros((4, 1))
        
        points_3d = np.hstack((mkp_ref, np.zeros((len(mkp_ref), 1))))
        
        try:
            _, r_vec, t_vec, _ = cv2.solvePnPRansac(
                points_3d, mkp_qry, k_matrix, dist_coeffs,
                useExtrinsicGuess=False, iterationsCount=100
            )
            r_matrix, _ = cv2.Rodrigues(r_vec)
            return r_matrix, t_vec
        except cv2.error:
            return None, None
    
    def _create_pose_msg(self, stamp, r, t, mkp_qry, mkp_ref):
        r_inv = r.T
        camera_pos = -r_inv @ t
        
        try:
            query_time = rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', query_time, rclpy.duration.Duration(seconds=0.2)
            )
            distance_to_ground = transform.transform.translation.z
            
            try:
                transform_cam = self._tf_buffer.lookup_transform(
                    'base_link_stabilized', 'camera_optical', query_time,
                    rclpy.duration.Duration(seconds=0.2)
                )
                q = transform_cam.transform.rotation
                angle_off_nadir = self._angle_off_nadir([q.x, q.y, q.z, q.w])
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                angle_off_nadir = 0.0
            
            fx = self._camera_info.k[0]
            scaling = abs(distance_to_ground / (fx * np.cos(angle_off_nadir))) if np.cos(angle_off_nadir) > 0.1 else abs(distance_to_ground / fx)
            
            meters_per_pixel = self._compute_meters_per_pixel(distance_to_ground, angle_off_nadir)
            
            camera_pos_scaled = np.array([
                camera_pos[0] * meters_per_pixel[0],
                camera_pos[1] * meters_per_pixel[1],
                camera_pos[2] * scaling
            ])
            
            camera_pos_scaled[0] -= meters_per_pixel[0] * self._camera_info.width / 2
            camera_pos_scaled[1] -= meters_per_pixel[1] * self._camera_info.height / 2
            camera_pos_scaled[2] += scaling * fx
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.get_parameter('output_frame').get_parameter_value().string_value
        
        q = tf_transformations.quaternion_from_matrix(np.vstack([np.hstack([r_inv, np.zeros((3, 1))]), [0, 0, 0, 1]]))
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]
        
        pose_msg.pose.pose.position.x = float(camera_pos_scaled[0])
        pose_msg.pose.pose.position.y = float(camera_pos_scaled[1])
        pose_msg.pose.pose.position.z = float(camera_pos_scaled[2])
        
        return pose_msg
    
    def _compute_meters_per_pixel(self, distance, angle_off_nadir):
        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        distance_along_axis = distance / np.cos(angle_off_nadir)
        
        fov_h = 2 * np.arctan(self._camera_info.width / (2 * fx))
        fov_v = 2 * np.arctan(self._camera_info.height / (2 * fy))
        
        width_m = 2 * distance_along_axis * np.tan(fov_h / 2)
        height_m = 2 * distance_along_axis * np.tan(fov_v / 2)
        
        return np.array([width_m / self._camera_info.width, height_m / self._camera_info.height])
    
    def _angle_off_nadir(self, quaternion):
        r_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
        camera_forward = np.array([1, 0, 0])
        camera_forward_base = r_matrix @ camera_forward
        nadir = np.array([0, 0, -1])
        cos_theta = np.dot(camera_forward_base, nadir) / (
            np.linalg.norm(camera_forward_base) * np.linalg.norm(nadir)
        )
        return np.arccos(np.clip(cos_theta, -1.0, 1.0))


def main(args=None):
    rclpy.init(args=args)
    node = VONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
