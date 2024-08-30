import rclpy
from rclpy.node import Node
import logging
# msgs
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header

# add
from ultralytics import YOLO
import numpy as np
from cv_bridge import CvBridge
import cv2
import open3d as o3d
# from ctypes import * # convert float to uint32
from pypcd4 import PointCloud
from builtin_interfaces.msg import Time
import struct

class RGBD2CLOUD(Node):
    def __init__(self):
        super().__init__('RGBD2CLOUD')
        self.init_param()

        # qos_policy = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_image = self.create_subscription(
                            RGBD(), 
                            '/input_rgbd', 
                            self.CB_main,
                            10)
        
        self.pub_cloud =  self.create_publisher(PointCloud2, '/output_cloud', 10)
        self.pub_image =  self.create_publisher(Image, '/output_image', 10)
        self.last_publish_time = self.get_clock().now()

    def CB_main(self, msg):
        current_time = self.get_clock().now()
        time_since_last_publish = (current_time - self.last_publish_time).nanoseconds / 1e9  # 秒に変換

        if time_since_last_publish >= self.pub_interval:
            self.process(msg)
            self.last_publish_time = current_time

    def process(self,msg):
        # bgr -> rgb
            input_rgb = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
            # mm?
            input_d = CvBridge().imgmsg_to_cv2(msg.depth, "passthrough") #.astype(np.int32)
            # resize
            input_rgb = cv2.resize(input_rgb, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)
            input_d = cv2.resize(input_d, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)

            # yolo
            mask_np, obj_num = self.create_mask(input_rgb)

            input_d[mask_np==0] = 0.0

            # publish mask image
            if self.do_publish_image:
                mask_img = mask_np * 255
                msg_out_img = CvBridge().cv2_to_imgmsg(mask_img.astype(np.uint8), encoding="mono8")
                self.pub_image.publish(msg_out_img)

            if obj_num >= 1:
                fx_d = msg.rgb_camera_info.k[0] * self.fxy
                fy_d = msg.rgb_camera_info.k[4] * self.fxy
                cx_d = msg.rgb_camera_info.k[2] * self.fxy
                cy_d = msg.rgb_camera_info.k[5] * self.fxy

                # create cloud
                color = o3d.geometry.Image(input_rgb)
                depth = o3d.geometry.Image(input_d)#.astype(np.uint16))
                height, width = input_d.shape
                pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                    width, height, fx_d,fy_d, cx_d, cy_d
                )
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color, depth, convert_rgb_to_intensity=False
                )
                pcd_o3d = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd, pinhole_camera_intrinsic,
                    project_valid_depth_only=True
                )

                # # Convert to Open3D.PointCLoud:
                pcd_o3d.transform( [[0,0, 1,0], 
                                    [-1, 0, 0, 0], 
                                    [0, -1, 0, 0], 
                                    [0, 0, 0, 1]])

                msg_out = self.convert_o3d_to_ros2(pcd_o3d)

            else:
                if self.do_publish_image:
                    mask_img = mask_np * 255
                    msg_out_img = CvBridge().cv2_to_imgmsg(mask_img.astype(np.uint8), encoding="mono8")
                    self.pub_image.publish(msg_out_img)

                # create cloud
                msg_out = self.create_empty_cloud_msg()

            self.pub_cloud.publish(msg_out)

    def create_empty_cloud_msg(self):
        header = Header()
        now = self.get_clock().now()
        header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        header.frame_id = self.frame_id
        msg_out = PointCloud2()
        msg_out.header = header
        msg_out.height = 1
        msg_out.width = 0
        msg_out.fields =[
                                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                                    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
                                ]
        msg_out.is_bigendian = False
        msg_out.point_step = 16
        msg_out.row_step = 0
        msg_out.is_dense = True
        msg_out.data = bytes([])
        return msg_out

    def init_param(self):
        self.declare_parameter('image_scale', 0.25)
        self.declare_parameter('do_publish_image', False)
        self.declare_parameter('frame_id', "hoge")
        self.declare_parameter('yolo.model_path', "")
        self.declare_parameter('yolo.classes', [0])
        self.declare_parameter('pub_interval', 0.5)

        self.fxy = self.get_parameter("image_scale").get_parameter_value().double_value
        self.do_publish_image = self.get_parameter('do_publish_image').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        yolo_path = self.get_parameter('yolo.model_path').get_parameter_value().string_value
        self.yolo_classes = self.get_parameter('yolo.classes').get_parameter_value().integer_array_value
        self.model = YOLO(yolo_path)
        self.pub_interval = self.get_parameter("pub_interval").get_parameter_value().double_value

    def create_mask(self,img):
        h,w,_ = img.shape
        results = self.model(img,conf=0.25 ,classes=self.yolo_classes)
        if results[0].masks:
            mask = results[0].masks.data.to('cpu').detach().numpy().copy().any(axis=0)
            return cv2.resize(mask.astype(int),(w,h),interpolation=cv2.INTER_NEAREST), len(results[0].masks)
        else:
            return np.zeros((h,w)), 0

    def convert_o3d_to_ros2(self, pcd_o3d):
        # TODO: fix convert_rgb_array_to_float
        try:
            header = Header()
            now = self.get_clock().now()
            header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
            header.frame_id = self.frame_id
            rgb_float_array = self.convert_rgb_array_to_float(pcd_o3d.colors)
            arr = np.concatenate([np.asarray(pcd_o3d.points) ,rgb_float_array.reshape((-1,1))],1)
            pc = PointCloud.from_xyzrgb_points(arr) #PointCloud.from_points(arr, fields, types)
            out_msg = pc.to_msg(header)
        except:
            out_msg = self.create_empty_cloud_msg()

        return out_msg
    
    def convert_rgb_array_to_float(self, rgb_array):
        """
        Convert an array of RGB values (0.0 to 1.0) to an array of single float32 RGB values.
        """
        rgb_float_array = np.apply_along_axis(lambda x: self.rgb_to_float(x[2], x[1], x[0]), 1, rgb_array)
        return rgb_float_array

    def rgb_to_float(self, r, g, b):
        """
        Convert separate R, G, B values (0.0 to 1.0) to a single float32 RGB value.
        """
        # Ensure the RGB values are in the range 0 to 255
        r = int(r * 255.0)
        g = int(g * 255.0)
        b = int(b * 255.0)
        
        # Combine the RGB values into a single 32-bit integer
        rgb_int = (r << 16) | (g << 8) | b
        
        # Pack this integer into a float32
        rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
        return rgb_float


def main():
    rclpy.init()
    node = RGBD2CLOUD()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()