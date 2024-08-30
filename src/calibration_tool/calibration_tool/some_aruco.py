import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
import os
import open3d as o3d
import time
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from std_msgs.msg import String, Bool
import tf2_ros
import tf2_geometry_msgs
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation

class Detector(Node):
    def __init__(self):
        super().__init__('Marker_detector')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.sub2= self.create_subscription(Bool, '/op_detect_marker', self.op_callback, 10)
        #self.camera_info_subscription = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        #self.timer = self.create_timer(1.0, self.publish_marker_tf)  # Timer to save images every 1 second
        self.rgb_image = None
        self.depth_image = None
        self.counter = 0
        self.save_path = './data/'
        self.fxy = 1.0  # Scaling factor, set as needed
        self.depth_range_max = 5000  # Maximum depth range for truncation
        self.flag_once = True
        self.flag_received_pose = False
        self.x_camera = 0.0
        self.y_camera = 0.0
        self.z_camera = 0.0
        self.trans_map2camera = None
        self.flag_detect = False
        self.camera_intrinsics =  o3d.camera.PinholeCameraIntrinsic(
            1280, 720, 908.600, 908.537, 643.698, 382.494
        )
        self.rgbd = None
        self.rgb_image = None
        self.depth_image = None
        self.data = [[],[],[]]


        # Create directory if it does not exist
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

    def listener_callback(self, msg):
        """
        try:
            trans = self.tf_buffer.lookup_transform('map', 'end_effector', rclpy.time.Time())
            self.flag_wait = True

        except Exception as e:
            #self.get_logger().info(f'Error : robot pose not found')
            #time.sleep(3)
            self.flag_wait = False
        """
        #if self.flag_detect == True:

        rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb).astype(np.uint8)
        rgb_image_cv2 = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
        depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
        
        rgb_image = cv2.resize(rgb_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
        rgb_image_cv2 = cv2.resize(rgb_image_cv2, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
        depth_image = cv2.resize(depth_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)

        color = o3d.geometry.Image(rgb_image)
        depth = o3d.geometry.Image(depth_image)

        self.rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        color, depth, depth_trunc=5.0, convert_rgb_to_intensity=False)
        self.rgb_image = rgb_image
        self.depth_image = depth_image

        #self.find_aruco(self.rgb_image, self.depth_image)

        #else :
        #self.print_to_file('output.txt')

        #rgb_height, rgb_width, rgb_channels = rgb_image.shape
        #depth_height, depth_width = depth_image.shape


    def op_callback(self, msg):
        print("find_aruco")
        #self.flag_detect = msg.data
        self.find_aruco(self.rgb_image, self.depth_image)
        self.print_to_file('output.txt')
        """
        #if self.flag_once == True and self.flag_wait == True: 
        if True:
            rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb).astype(np.uint8)
            rgb_image_cv2 = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
            depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
            
            rgb_image = cv2.resize(rgb_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
            rgb_image_cv2 = cv2.resize(rgb_image_cv2, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
            depth_image = cv2.resize(depth_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)

            color = o3d.geometry.Image(rgb_image)
            depth = o3d.geometry.Image(depth_image)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                            color, depth, depth_trunc=5.0, convert_rgb_to_intensity=False)

            rgb_height, rgb_width, rgb_channels = rgb_image.shape
            depth_height, depth_width = depth_image.shape

            # Print the dimensions
            #print(f"RGB Image - Width: {rgb_width}, Height: {rgb_height}, Channels: {rgb_channels}")

            #time.sleep(1)

            #self.find_marker(rgbd)
            self.find_aruco(rgb_image, depth_image)
            #self.publish_marker_tf()
        """

    def find_aruco(self, rgb_image, depth_image):
        # ArUco Marker Detection
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, dictionary)
        rs_camera_intrinsics = self.convert_intrinsics(self.camera_intrinsics)
        target_ids = [0, 1, 2]
        all_detected = False
        

        # 検出されたIDリストとコーナーポイントリストを取得
        if ids is not None:
            ids = ids.flatten()

            # 抽出したIDのコーナーポイントを格納する辞書
            target_corners = {id_: [] for id_ in target_ids}  # ここでリストを使う

            # コーナーポイントの格納
            for idx, id_ in enumerate(ids):
                if id_ in target_ids:
                    target_corners[id_].append(corners[idx])  # 各IDにコーナーポイントを追加

            #print(target_ids)

            # 3D座標の計算とPublish
            for id_ in target_ids:
                corner_points = target_corners[id_]
                if not corner_points:
                    print(f"ID {id_} のコーナーポイントが見つかりません")
                    continue

                point = corner_points[0][0][0]
                x, y = point
                x, y = int(x), int(y)

                # Depth画像の範囲内であることを確認
                depth = depth_image[y, x]
                if depth != 0:
                    point3d = rs.rs2_deproject_pixel_to_point(rs_camera_intrinsics, [x, y], depth)
                    x_camera = point3d[0] / 1000
                    y_camera = point3d[1] / 1000
                    z_camera = point3d[2] / 1000
                    print(f"ID {id_}のコーナーポイント {point} の3D座標: x={x_camera}, y={y_camera}, z={z_camera}")
                    #self.publish_marker_tf(id_, x_camera, y_camera, z_camera)
                    self.data[id_].append([x_camera, y_camera, z_camera])
                else:
                    print(f"ID {id_} のコーナーポイント {point} の深度値が0です")

        # Draw marker and display
        aruco.drawDetectedMarkers(rgb_image, corners, ids, (0, 255, 0))
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        rgb_image_cv2 = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        images = np.hstack((rgb_image_cv2, depth_colormap))
        cv2.imshow("Detected ArUco Markers", images)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
        
        """
        print("\n--- id=0 ---\n")
        print(self.data[0])
        print("\n--- id=1 ---\n")
        print(self.data[1])
        print("\n--- id=2 ---\n")
        print(self.data[2])
        """


        """

        all_detected = all(target_corners[id_] is not None for id_ in target_ids)

        if all_detected:
            # target_corners[1]とtarget_corners[0]の差分を計算
            diff = np.array(target_corners[1]) - np.array(target_corners[0])
            print(f"ID 1とID 0のコーナーポイントの差分: \n {diff}")

            for id_ in target_ids:
                corner_points = target_corners[id_][0]  # マーカーの4つのコーナーポイント
                for point in corner_points:
                    x, y = point
                    depth = depth_image[int(y), int(x)]
                    if depth != 0:
                        point3d = rs.rs2_deproject_pixel_to_point(rs_camera_intrinsics, [x, y], depth)
                        x_camera = point3d[0] / 1000
                        y_camera = point3d[1] / 1000
                        z_camera = point3d[2] / 1000
                        print(f"ID {id_}のコーナーポイント {point} の3D座標: x={x_camera}, y={y_camera}, z={z_camera}")
                        self.publish_marker_tf(id_, x_camera, y_camera, z_camera)
        else:
            print("マーカーが検出されませんでした。")
        """

        
        """
        if len(corners) > 0:
            self.get_logger().info(corners)
            for i, corner in enumerate(corners):
                marker_id = ids[i][0]
                top_left_corner = corner[0][0]
                x, y = top_left_corner
                depth = depth_image[int(y), int(x)]
                if depth != 0:
                    point3d = rs.rs2_deproject_pixel_to_point(rs_camera_intrinsics, [x, y], depth)
                    x_camera = point3d[0] / 1000
                    y_camera = point3d[1] / 1000
                    z_camera = point3d[2] / 1000

                    # Log the 3D position of the marker
                    self.get_logger().info(f"Marker {marker_id}: 3D point: x={x_camera}, y={y_camera}, z={z_camera}")

                    # Publish transform for the detected marker
                    #self.publish_marker_tf(marker_id, x_camera, y_camera, z_camera)

                    # Draw marker and display
                    aruco.drawDetectedMarkers(rgb_image, corners, ids, (0, 255, 0))
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    rgb_image_cv2 = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                    images = np.hstack((rgb_image_cv2, depth_colormap))
                    #cv2.imshow("Detected ArUco Markers", images)
                    #cv2.waitKey(2000)
                    #cv2.destroyAllWindows()
        """
                        
    def convert_intrinsics(self, o3d_intrinsics):
        intrinsics = rs.intrinsics()
        intrinsics.width = o3d_intrinsics.width
        intrinsics.height = o3d_intrinsics.height
        intrinsics.fx = o3d_intrinsics.intrinsic_matrix[0, 0]
        intrinsics.fy = o3d_intrinsics.intrinsic_matrix[1, 1]
        intrinsics.ppx = o3d_intrinsics.intrinsic_matrix[0, 2]
        intrinsics.ppy = o3d_intrinsics.intrinsic_matrix[1, 2]
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0, 0, 0, 0, 0]  # Assuming no distortion
        return intrinsics

    def camera_info_callback(self, msg):
        # Extract intrinsic parameters from CameraInfo message
        if self.camera_intrinsics is None:
            self.camera_intrinsics = msg
            #self.get_logger.info('Camera intrinsics received.')
    
    def publish_marker_tf(self, marker_id, x_camera, y_camera, z_camera):
        try:
            map_trans = np.identity(4)
            # Get the transform from 'end_effector' to 'map'
            trans = self.tf_buffer.lookup_transform('map', 'camera_color_optical_frame', rclpy.time.Time())
            tx,ty,tz = trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z
            rx,ry,rz,rw = trans.transform.rotation.x,trans.transform.rotation.y ,trans.transform.rotation.z,trans.transform.rotation.w
            rot = Rotation.from_quat(np.array([rx,ry,rz,rw]))
            map_trans[:3,:3] = rot.as_matrix()
            map_trans[:3,3] = np.array([tx,ty,tz])

            point = np.array([x_camera, y_camera, z_camera, 1])
            point_on_map = (map_trans @ point)[:3]
            print(point_on_map)


            # Create the TransformStamped object to publish
            trans_map2marker = TransformStamped()
            trans_map2marker.header.stamp = self.get_clock().now().to_msg()
            trans_map2marker.header.frame_id = 'map'
            trans_map2marker.child_frame_id = f'marker_{marker_id}'

            trans_map2marker.transform.translation.x = point_on_map[0]
            trans_map2marker.transform.translation.y = point_on_map[1]
            trans_map2marker.transform.translation.z = point_on_map[2]

            # Set rotation (identity quaternion)
            trans_map2marker.transform.rotation.x = 0.0
            trans_map2marker.transform.rotation.y = 0.0
            trans_map2marker.transform.rotation.z = 0.0
            trans_map2marker.transform.rotation.w = 1.0

            # Publish the transform
            self.tf_broadcaster.sendTransform(trans_map2marker)
            self.get_logger().info(f'Publishing transform for marker {marker_id} in map frame')

        except (tf2_ros.TransformException, tf2_ros.LookupException) as e:
            self.get_logger().error(f'Error: {e}')

    def print_to_file(self, filename):
        with open(filename, 'w') as file:
            file.write("\n--- id=0 ---\n")
            file.write(str(self.data[0]) + '\n')
            file.write("\n--- id=1 ---\n")
            file.write(str(self.data[1]) + '\n')
            file.write("\n--- id=2 ---\n")
            file.write(str(self.data[2]) + '\n')

        


        
rclpy.init(args=None)
rclpy.spin(Detector())
rclpy.shutdown()
