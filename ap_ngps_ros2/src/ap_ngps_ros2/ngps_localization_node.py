#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float64, Header
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import torch
from PIL import Image as PILImage
from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image_arr, rbd
import gc
import os
from pathlib import Path


class NGPSLocalizationNode(Node):
    
    def __init__(self):
        super().__init__('ngps_localization_node')
        
        self.bridge = CvBridge()
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
        self.extractor = SuperPoint(max_num_keypoints=2048).eval().to(self.device)
        self.matcher = LightGlue(features="superpoint").eval().to(self.device)
        
        self.tot_rot = 0.0
        self.base_x = 1024
        self.base_y = 712
        self.x = 0
        self.y = 0
        self.theta_deg = 0.0
        self.coord = []
        self.frame_count = 0
        
        self.rot_hist = []
        self.max_rot_hist = 10
        self.rot_std_thresh = 15.0
        self.max_rot_change = 30.0
        self.last_valid_rot = 0.0
        
        self.load_reference_image()
        
        self.pose_pub = self.create_publisher(PoseStamped, 'ngps/pose', 10)
        self.position_pub = self.create_publisher(PointStamped, 'ngps/position', 10)
        self.rotation_pub = self.create_publisher(Float64, 'ngps/rotation', 10)
        self.debug_image_pub = self.create_publisher(Image, 'ngps/debug_image', 10)
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.declare_parameter('reference_image_path', '')
        self.declare_parameter('kernel_size', 300)
        self.declare_parameter('match_threshold', 0.5)
        self.declare_parameter('min_matches', 20)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('max_rotation_change', 30.0)
        self.declare_parameter('rotation_std_threshold', 15.0)
        self.declare_parameter('max_rotation_history', 10)
        self.declare_parameter('enable_rotation_smoothing', True)
        self.declare_parameter('enable_rotation_validation', True)
        self.declare_parameter('frame_id', 'map')
        
        print('NGPS Localization Node initialized')
    
    def load_reference_image(self):
        ref_img_path = self.get_parameter('reference_image_path').get_parameter_value().string_value
        
        if not ref_img_path or not os.path.exists(ref_img_path):
            print("No reference image provided, using dummy")
            self.imarray = np.zeros((1440, 1024, 3), dtype=np.uint8)
        else:
            try:
                im = PILImage.open(ref_img_path)
                self.imarray = np.array(im)
                self.imarray = cv.rotate(self.imarray, cv.ROTATE_90_COUNTERCLOCKWISE)
                print(f"Loaded reference image: {ref_img_path}")
            except Exception as e:
                print(f"Failed to load reference image: {e}")
                self.imarray = np.zeros((1440, 1024, 3), dtype=np.uint8)
        
        self.imarrayB = cv.rotate(self.imarray, cv.ROTATE_90_CLOCKWISE)
        self.h, self.w, self.c = self.imarray.shape
    
    def kernel_show(self, center_row, center_col, rot=0.0):
        ksize = self.get_parameter('kernel_size').get_parameter_value().integer_value
        
        if center_row == 0 and center_col == 0:
            center_row = self.h // 2
            center_col = self.w // 2
        
        margin = ksize // 2
        min_row, max_row = margin, self.h - margin
        min_col, max_col = margin, self.w - margin
        
        center_row = np.clip(center_row, min_row, max_row)
        center_col = np.clip(center_col, min_col, max_col)
        
        self.tot_rot += rot
        rot_mat = cv.getRotationMatrix2D((int(center_col), int(center_row)), self.tot_rot, 1)
        imarray = cv.warpAffine(self.imarrayB, rot_mat, (self.imarrayB.shape[1], self.imarrayB.shape[0]))
        
        imarray_seg = imarray[center_row - margin : center_row + margin,
                              center_col - margin : center_col + margin,
                              :]
        imarray_seg = cv.cvtColor(imarray_seg, cv.COLOR_BGR2RGB)
        imarray_seg = cv.resize(imarray_seg, (900, 900), interpolation=cv.INTER_LINEAR)
        
        return imarray_seg
    
    def map_kernel_img(self, x, y, base_x, base_y):
        if base_x == 0 and base_y == 0:
            base_x = self.h // 2
            base_y = self.w // 2
        
        rot_mat = cv.getRotationMatrix2D((int(0), int(0)), self.tot_rot, 1)
        rot3d = np.vstack((rot_mat, np.array([0, 0, 1])))
        inv_rot3d = np.linalg.inv(rot3d)
        corr_pt = np.matmul(inv_rot3d, np.array([y, x, 1]))
        
        new_x = base_x + corr_pt[1]
        new_y = base_y + corr_pt[0]
        
        return new_x, new_y
    
    def calculate_rotation_from_homography(self, M):
        try:
            M_norm = M / M[2, 2]
            R = M_norm[:2, :2]
            
            scale_x = np.linalg.norm(R[:, 0])
            scale_y = np.linalg.norm(R[:, 1])
            
            if scale_x > 0 and scale_y > 0:
                R_norm = R / np.array([[scale_x], [scale_y]])
                theta_rad = np.arctan2(R_norm[1, 0], R_norm[0, 0])
                theta_deg = np.degrees(theta_rad)
            else:
                theta_deg = 0.0
                
        except Exception as e:
            print(f"homography calc failed: {e}")
            theta_deg = 0.0
        
        return theta_deg
    
    def calculate_rotation_from_keypoints(self, m_kpts0, m_kpts1):
        try:
            if len(m_kpts0) < 4:
                return 0.0
            
            kpts0_np = m_kpts0.cpu().numpy()
            kpts1_np = m_kpts1.cpu().numpy()
            
            rots = []
            for i in range(len(kpts0_np)):
                for j in range(i + 1, len(kpts0_np)):
                    v0 = kpts0_np[j] - kpts0_np[i]
                    v1 = kpts1_np[j] - kpts1_np[i]
                    
                    if np.linalg.norm(v0) > 0 and np.linalg.norm(v1) > 0:
                        cos_angle = np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1))
                        cos_angle = np.clip(cos_angle, -1.0, 1.0)
                        angle = np.arccos(cos_angle)
                        rots.append(np.degrees(angle))
            
            if len(rots) > 0:
                return np.median(rots)
            else:
                return 0.0
                
        except Exception as e:
            print(f"keypoint rot failed: {e}")
            return 0.0
    
    def calculate_rotation_from_contour(self, dst):
        try:
            rect = cv.minAreaRect(np.int32(dst))
            theta_deg = rect[2]
            
            if theta_deg < -45:
                theta_deg = 90 + theta_deg
            elif theta_deg > 45:
                theta_deg = -90 + theta_deg
            
            return theta_deg
            
        except Exception as e:
            print(f"contour calc error: {e}")
            return 0.0
    
    def validate_rotation(self, theta_deg):
        if not self.get_parameter('enable_rotation_validation').get_parameter_value().bool_value:
            return theta_deg
        
        max_change = self.get_parameter('max_rotation_change').get_parameter_value().double_value
        std_threshold = self.get_parameter('rotation_std_threshold').get_parameter_value().double_value
        
        if abs(theta_deg - self.last_valid_rot) > max_change:
            print(f"Large rotation change: {theta_deg:.2f} -> {self.last_valid_rot:.2f}")
            return self.last_valid_rot
        
        self.rot_hist.append(theta_deg)
        max_history = self.get_parameter('max_rotation_history').get_parameter_value().integer_value
        if len(self.rot_hist) > max_history:
            self.rot_hist.pop(0)
        
        if len(self.rot_hist) >= 3:
            recent_std = np.std(self.rot_hist[-3:])
            if recent_std > std_threshold:
                print(f"High rotation variance: {recent_std:.2f}")
                return self.last_valid_rot
        
        self.last_valid_rot = theta_deg
        return theta_deg
    
    def smooth_rotation(self, theta_deg):
        if not self.get_parameter('enable_rotation_smoothing').get_parameter_value().bool_value:
            return theta_deg
        
        if len(self.rot_hist) == 0:
            return theta_deg
        
        weights = np.linspace(0.5, 1.0, len(self.rot_hist))
        weights = weights / np.sum(weights)
        
        smoothed_rot = np.average(self.rot_hist, weights=weights)
        return smoothed_rot
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            frame = cv.resize(frame, (0, 0), fx=0.6, fy=0.6, interpolation=cv.INTER_AREA)
            hh, ww = frame.shape[:2]
            
            self.base_x, self.base_y = self.map_kernel_img(self.y, self.x, self.base_x, self.base_y)
            
            img_kl = self.kernel_show(int(self.base_x), int(self.base_y), self.theta_deg)
            
            image0 = load_image_arr(img_kl)
            feats0 = self.extractor.extract(image0.to(self.device))
            
            if frame.ndim == 3:
                frame_tensor = frame.transpose((2, 0, 1))
            elif frame.ndim == 2:
                frame_tensor = frame[None]
            
            image1 = torch.tensor(frame_tensor / 255.0, dtype=torch.float)
            feats1 = self.extractor.extract(image1.to(self.device))
            
            if self.frame_count % 23 == 0:
                print(f"frame {self.frame_count} processed")
            
            matches01 = self.matcher({"image0": feats0, "image1": feats1})
            
            feats0, feats1, matches01 = [rbd(x) for x in [feats0, feats1, matches01]]
            
            kpts0, kpts1 = feats0["keypoints"], feats1["keypoints"]
            matches0 = matches01["matches0"]
            scores0 = matches01["matching_scores0"]
            
            threshold = self.get_parameter('match_threshold').get_parameter_value().double_value
            valid = (scores0 > threshold)
            
            idx0 = torch.nonzero(valid).squeeze()
            idx1 = matches0[valid]
            
            m_kpts0 = kpts0[idx0]
            m_kpts1 = kpts1[idx1]
            
            num_matches = len(m_kpts0)
            if self.frame_count % 10 == 0:
                print(f"matches: {num_matches}")
            
            if num_matches > 100:
                print("lots of matches!")
            
            min_matches = self.get_parameter('min_matches').get_parameter_value().integer_value
            if num_matches < min_matches:
                return
            
            M, mask = cv.findHomography(
                m_kpts1.cpu().numpy(), 
                m_kpts0.cpu().numpy(), 
                cv.RANSAC, 
                5.0
            )
            
            if M is None:
                return
            
            rots = []
            
            theta_hom = self.calculate_rotation_from_homography(M)
            if abs(theta_hom) < 180:
                rots.append(theta_hom)
            
            theta_kpts = self.calculate_rotation_from_keypoints(m_kpts0, m_kpts1)
            if abs(theta_kpts) < 180:
                rots.append(theta_kpts)
            
            pts = np.float32([[0, 0], [ww, 0], [ww, hh], [0, hh]]).reshape(-1, 1, 2)
            dst = cv.perspectiveTransform(pts, M)
            theta_cnt = self.calculate_rotation_from_contour(dst)
            if abs(theta_cnt) < 180:
                rots.append(theta_cnt)
            
            if len(rots) > 0:
                theta_deg = np.median(rots)
                
                theta_deg = self.validate_rotation(theta_deg)
                theta_deg = self.smooth_rotation(theta_deg)
                
                self.theta_deg = theta_deg
                
                if abs(theta_deg) > 50:
                    print(f"large rot detected: {theta_deg:.1f}")
            else:
                return
            
            x, y = np.mean(dst, axis=0).astype(int)[0]
            self.x = (x - 450) // 3
            self.y = (y - 450) // 3
            
            self.coord.append([self.base_x, self.base_y])
            
            if len(self.coord) > 1000:
                self.coord = self.coord[-500:]
            
            self.publish_results()
            
            self.publish_debug_image(img_kl, frame, m_kpts0, m_kpts1, dst)
            
            torch.cuda.empty_cache()
            gc.collect()
            
            self.frame_count += 1
            
        except Exception as e:
            print(f"error in callback: {e}")
    
    def publish_results(self):
        current_time = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(self.base_x)
        pose_msg.pose.position.y = float(self.base_y)
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.z = np.sin(np.radians(self.theta_deg) / 2.0)
        pose_msg.pose.orientation.w = np.cos(np.radians(self.theta_deg) / 2.0)
        self.pose_pub.publish(pose_msg)
        
        position_msg = PointStamped()
        position_msg.header.stamp = current_time
        position_msg.header.frame_id = frame_id
        position_msg.point.x = float(self.x)
        position_msg.point.y = float(self.y)
        position_msg.point.z = 0.0
        self.position_pub.publish(position_msg)
        
        rotation_msg = Float64()
        rotation_msg.data = self.theta_deg
        self.rotation_pub.publish(rotation_msg)
    
    def publish_debug_image(self, img_kl, frame, m_kpts0, m_kpts1, dst):
        try:
            rect = cv.minAreaRect(np.int32(dst))
            box = cv.boxPoints(rect)
            box = np.int32(box)
            img_debug = cv.drawContours(img_kl.copy(), [box], 0, 255, 30, cv.LINE_AA)
            
            x, y = np.mean(dst, axis=0).astype(int)[0]
            img_debug = cv.circle(img_debug, (x, y), 10, (0, 0, 255), -1)
            
            debug_msg = self.bridge.cv2_to_imgmsg(img_debug, "rgb8")
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = "camera"
            
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    node = NGPSLocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
