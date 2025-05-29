#!/usr/bin/env python3

import math

import cv2
import numpy as np 

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from flo_msgs.srv import Enable, GetAnnotationStatus

from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from flo_msgs.msg import OccupancyGrid 

from flo_utils.ros2_utils import spin


class AnnotateCmdvel(Node): 
    def __init__(self):
        super().__init__('annotate_cmdvel')

        self.annotate_cmd_vel_srv = self.create_service(
            Enable,
            'annotate/cmd_vel',
            self.annotate_cmd_vel_cb
        )

        self.annotate_ogm_srv = self.create_service(
            Enable,
            'annotate/ogm',
            self.annotate_ogm_srv_cb
        )

        self.annotate_path_srv = self.create_service(
            Enable,
            'annotate/path',
            self.annotate_path_srv_cb
        )

        self.get_annotation_status_srv = self.create_service(
            GetAnnotationStatus,
            'annotate/status',
            self.annotate_status_cb
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_cb, 
            10
        )

        self.img_raw_sub = self.create_subscription(
            Image, 
            'image_raw', 
            self.img_raw_cb, 
            10
        )

        self.omg_sub = self.create_subscription(
            OccupancyGrid,
            'occupancy_grid',
            self.ogm_raw_cb,
            10
        )

        self.annotated_img_pub = self.create_publisher(
            Image, 
            'image_raw/annotated', 
            10
        )
        self.enable_annotate_cmd_vel = False
        self.enable_annotate_ogm = False
        self.enable_annotate_path = False 

        self.load_params()
        self.cv_ros = CvBridge()

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.rgb_height = 0
        self.rgb_width = 0

        self.ogm_height = 0
        self.ogm_width = 0
        self.ogm = None

    def load_params(self):
        self.declare_parameter('footprint_width', 0.76)
        self.declare_parameter('footprint_offset', 0.3)
        self.declare_parameter(
            'K', 
            [513.663, 0.0, 319.198, 0.0, 513.663, 240.053, 0.0, 0.0, 1.0]
        )
        self.declare_parameter(
            'K_disp', 
            [398.795, 0.0, 315.475, 0.0, 398.795, 513.663, 0.0, 0.0, 1.0]
        )
        self.declare_parameter(
            'R_cam_bf',
            [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0]
        )
        self.declare_parameter(
            't_cam_bf',
            [0.28, 0.0, 0.28]
        )
        self.declare_parameter('min_acceptable_velocity', 0.01)
        self.declare_parameter('distance_to_propagate', 2.5)
        self.declare_parameter('number_of_steps', 20)
        self.declare_parameter('min_radius_of_curvature', 1.0)
        self.declare_parameter('annotation_alpha', 0.35)
        self.declare_parameter(
            'annotation_color',
            [68, 68, 239]
        )

        self.footprint_width = self.get_parameter(
                'footprint_width').get_parameter_value().double_value
        self.footprint_offset = self.get_parameter(
                'footprint_offset').get_parameter_value().double_value
        self.K = np.array(
            self.get_parameter('K').get_parameter_value().double_array_value).reshape(3,3)
        self.K_disp = np.array(
            self.get_parameter('K_disp').get_parameter_value().double_array_value).reshape(3,3)
        self.R_cam_bf = np.array(
            self.get_parameter('R_cam_bf').get_parameter_value().double_array_value).reshape(3,3)
        self.t_cam_bf = np.array(
            self.get_parameter('t_cam_bf').get_parameter_value().double_array_value)
        self.min_acceptable_velocity = self.get_parameter(
                'min_acceptable_velocity').get_parameter_value().double_value
        self.distance_to_propagate = self.get_parameter(
                'distance_to_propagate').get_parameter_value().double_value
        self.number_of_steps = self.get_parameter(
                'number_of_steps').get_parameter_value().integer_value
        self.min_radius_of_curvature = self.get_parameter(
                'min_radius_of_curvature').get_parameter_value().double_value
        self.annotation_alpha = self.get_parameter(
                'annotation_alpha').get_parameter_value().double_value
        self.annotation_color = tuple(
            self.get_parameter('annotation_color').get_parameter_value().integer_array_value)

        self.KR = self.K @ self.R_cam_bf.T
        self.scale_factor = self.K[0,0]/self.K_disp[0,0]

    def annotate_cmd_vel_cb(self, req, res):
        if req.enable:
            self.enable_annotate_cmd_vel = True
            res.message = "cmd_vel annotation enabled"
        else: 
            self.enable_annotate_cmd_vel = False
            res.message = "cmd_vel annotation disabled"
        res.success = True
        return res 

    def annotate_ogm_srv_cb(self, req, res):
        if req.enable:
            self.enable_annotate_ogm = True
            res.message = "OccupancyGrid annotation enabled"
        else: 
            self.enable_annotate_ogm = False
            res.message = "OccupancyGrid annotation disabled"
        res.success = True
        return res 

    def annotate_path_srv_cb(self, req, res):
        if req.enable:
            self.enable_annotate_path = True
            res.message = "Path annotation enabled"
        else: 
            self.enable_annotate_path = False 
            res.message = "Path annotation disabled"
        res.success = True
        return res
    
    def annotate_status_cb(self, _, res):
        res.trajectory = self.enable_annotate_cmd_vel
        res.ogm = self.enable_annotate_ogm
        res.path = self.enable_annotate_path
        return res

    def cmd_vel_cb(self, cmd_vel):
        self.linear_velocity = cmd_vel.linear.x 
        self.angular_velocity = cmd_vel.angular.z 

    def img_raw_cb(self, img_raw_msg): 
        img_raw = self.cv_ros.imgmsg_to_cv2(img_raw_msg, 'bgr8')

        self.rgb_height, self.rgb_width, _ = img_raw.shape
        # start pipeline
        img = self.annotate_cmd_vel(img_raw)
        img = self.annotate_ogm(img)
        img = self.annotate_path(img)
        # end pipeline

        self.annotated_img_pub.publish(self.cv_ros.cv2_to_imgmsg(img,'bgr8'))

    def ogm_raw_cb(self, ogm_msg): 
        self.ogm_height = ogm_msg.height
        self.ogm_width = ogm_msg.width
        ogm_raw = np.array(ogm_msg.data)
        self.ogm = ogm_raw.reshape(self.ogm_height, self.ogm_width)
        
    def annotate_cmd_vel(self, img):
        if not self.enable_annotate_cmd_vel:
            return img
        
        annotate, v, w = self.check_and_normalize(self.linear_velocity, self.angular_velocity)
        if not annotate:
            return img

        state = self.propagate_state(v, w)
        left_corners, right_corners = self.propagate_footprint(state)
        annotated_img = self.annotate_image(img, left_corners, right_corners)
        return annotated_img

    def annotate_ogm(self, img): 
        if not self.enable_annotate_ogm: 
            return img 
        
        middle_index = self.ogm_width/2
        
        points2d = np.column_stack(np.where(self.ogm == 1))
        points2d_pairs = np.repeat(points2d, 4, axis=0) 
        points2d_pairs[1::4,1] = points2d[:,1] + 1
        points2d_pairs[2::4,0] = points2d[:,0] + 1
        points2d_pairs[3::4,0:2] = points2d[:,0:2] + 1
        points2d_pairs[:,1] = middle_index - points2d_pairs[:,1]
        points2d_pairs = points2d_pairs * 0.1

        points3d_pairs = np.hstack((points2d_pairs, \
                                    np.zeros((points2d_pairs.shape[0],1),dtype=np.float32))) 
        
        points3d_pairs[:,0] += self.t_cam_bf[0]
        points3d_pairs[:,1] *= self.scale_factor
        
        p_hat_pairs = np.dot(self.KR, (points3d_pairs - self.t_cam_bf.reshape(1,-1)).T).T

        w_pairs = p_hat_pairs[:,2]
        p_blocks = abs(p_hat_pairs[:,:2]/w_pairs[:,np.newaxis]).astype(int)

        for i in range(0, len(p_blocks) -3, 4):
            pxls = p_blocks[i:i+4]
            if np.all(0 <= pxls[:,0]) and np.all(pxls[:,0] < self.rgb_width) and \
                np.all(0 <= pxls[:,1]) and np.all(pxls[:,1] < self.rgb_height) : 
                pxls[[0,1]] = pxls[[1,0]]
                cv2.fillConvexPoly(img, pxls, color=(0,255,0))
        
        return img
    
    def annotate_path(self, img): 
        if not self.enable_annotate_path: 
            return img 
        
    def check_and_normalize(self, linear_velocity, angular_velocity):
        v = 0.0
        w = 0.0
        if linear_velocity >= 0.0:
            if linear_velocity < self.min_acceptable_velocity:
                linear_velocity = self.min_acceptable_velocity
            if math.fabs(angular_velocity) > self.min_acceptable_velocity:
                roc = linear_velocity/angular_velocity # Radius of curvature
                if math.fabs(roc) < self.min_radius_of_curvature:
                    roc = np.sign(roc) * self.min_radius_of_curvature
                v = 1.0
                w = 1.0/roc
                return True, v, w
            else:
                v = 1.0
                w = 0.0
                return True, v, w
        else:
            return False, v, w

    def propagate_state(self, v, w):
        state = [[0.0, 0.0, 0.0]] # [x, y, theta]
        dt = self.distance_to_propagate/self.number_of_steps
        for i in range(self.number_of_steps):
            x_t = state[-1][0]
            y_t = state[-1][1]
            theta_t = state[-1][2]

            x_tp1 = x_t + (v * math.cos(theta_t) * dt)
            y_tp1 = y_t + (v * math.sin(theta_t) * dt)
            theta_tp1 = theta_t + (w * dt)

            state.append([x_tp1, y_tp1, theta_tp1])
        return state

    def propagate_footprint(self, state):
        left_corners = []
        right_corners = []

        for x, y, theta in state:
            c_th = math.cos(theta)
            s_th = math.sin(theta)
            R = np.array(
                [[c_th, -s_th],
                [s_th, c_th]]
            )

            left_corner = list(np.array([x, y]) +\
                    R @ np.array([self.footprint_offset, self.footprint_width/2]))
            left_corners.append(left_corner)

            right_corner = list(np.array([x, y]) +\
                    R @ np.array([self.footprint_offset, -self.footprint_width/2]))
            right_corners.append(right_corner)

        return left_corners, right_corners

    def hmg_to_px(self, pt_hmg): # hmg: Homogeneous
        px = int(pt_hmg[0]/pt_hmg[2])
        py = int(pt_hmg[1]/pt_hmg[2])
        return px, py

    def annotate_image(self, img, lcs, rcs): # lcs: left corners, rcs: right corners
        img_overlay = cv2.addWeighted(
            img, 1-self.annotation_alpha,
            np.full_like(img, self.annotation_color),
            self.annotation_alpha, 0
        )
        mask = np.zeros_like(img)
        for i in range(len(lcs)-1):
            if lcs[i][0] <= self.footprint_offset or lcs[i+1][0] <= self.footprint_offset:
                continue
            li = self.hmg_to_px(
                self.KR @ (np.array([lcs[i][0], lcs[i][1], 0.0]) - self.t_cam_bf)
            )
            lip1 = self.hmg_to_px(
                self.KR @ (np.array([lcs[i+1][0], lcs[i+1][1], 0.0])- self.t_cam_bf)
            )
            if rcs[i][0] <= self.footprint_offset or rcs[i+1][0] <= self.footprint_offset:
                continue
            ri = self.hmg_to_px(
                self.KR @ (np.array([rcs[i][0], rcs[i][1], 0.0]) - self.t_cam_bf)
            )
            rip1 = self.hmg_to_px(
                self.KR @ (np.array([rcs[i+1][0], rcs[i+1][1], 0.0]) - self.t_cam_bf)
            )
            roi = np.array([li, lip1, rip1, ri], np.int32)
            cv2.fillPoly(mask, [roi], (255, 255, 255))

        annotated_img = np.where(mask==255, img_overlay, img)
        return annotated_img


def main(args=None):
    rclpy.init(args=args)

    annotate_cmdvel = AnnotateCmdvel()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(annotate_cmdvel)
    
    try:
        spin(annotate_cmdvel, executor=multi_thread_executor, hz=100)
    except KeyboardInterrupt:
        print("Killing annotate_cmdvel node!")


if __name__ == "__main__":
    main()
