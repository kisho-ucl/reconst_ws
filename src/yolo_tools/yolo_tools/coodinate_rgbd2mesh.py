import rclpy
from rclpy.node import Node
import logging
# msgs
from realsense2_camera_msgs.msg import RGBD
# from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header, Bool, Float32
from geometry_msgs.msg import Point,Pose, Vector3, TransformStamped
from shape_msgs.msg import Mesh, MeshTriangle
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
# from mesh_msgs.msg import MeshGeometryStamped, MeshGeometry, MeshVertexColorsStamped, MeshVertexColors, MeshTriangleIndices

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
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation

import paho.mqtt.client as mqtt
import json
import os
import pyautogui
import copy
import threading

def matrix_to_transform(pose):
    t = TransformStamped()
    matrix = np.copy(pose)

    t.transform.translation.x = matrix[0, 3]
    t.transform.translation.y = matrix[1, 3]
    t.transform.translation.z = matrix[2, 3]

    # 行列からクォータニオンに変換
    rotation_matrix = matrix[:3, :3]
    rotation = Rotation.from_matrix(rotation_matrix)

    q = rotation.as_quat()
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t

def draw_coodinate():
    pcd = o3d.geometry.PointCloud()
    num_points = 100  # 点の数
    colors = np.zeros((num_points, 3))  # (num_points, 3) の配列を作成
    points = np.linspace([0, 0, 0], [1, 0, 0], num=num_points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    colors[:] = [1, 0, 0]  # 全ての点に赤色を設定
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    pcd += point_cloud
    points = np.linspace([0, 0, 0], [0, 1, 0], num=num_points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    colors[:] = [0, 1, 0]  # 全ての点に赤色を設定
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    pcd += point_cloud
    points = np.linspace([0, 0, 0], [0, 0, 1], num=num_points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    colors[:] = [0, 0, 1]  # 全ての点に赤色を設定
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    pcd += point_cloud
    return pcd

def draw_vector(p0, p1):
    #print(p1-p0)
    pcd = o3d.geometry.PointCloud()
    num_points = 100  # 点の数
    colors = np.zeros((num_points, 3))  # (num_points, 3) の配列を作成
    points = np.linspace(p0, p1, num=num_points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    colors[:] = [0, 0, 0]  # 全ての点に赤色を設定
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    pcd += point_cloud
    return pcd

class RGBD2MESH(Node):
    def __init__(self):
        super().__init__('RGBD2MESH')
        self.init_param()

        self.client = mqtt.Client()

        self.client.connect("192.168.207.22", 1883, keepalive=300)

        #self.timer = self.create_timer(2.0, self.timer_callback) 

        # qos_policy = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_image = self.create_subscription(
                            RGBD(), 
                            '/input_rgbd', 
                            self.CB_main,
                            10)
        self.subscription = self.create_subscription(
                            Bool,
                            '/servo_state_move',
                            self.update_stateMove,
                            10)
        self.sub3 = self.create_subscription(
                            Bool,
                            '/op_create_mesh',
                            self.update_op,
                            10)
        self.sub4 = self.create_subscription(
                            Bool,
                            '/op_save_mesh',
                            self.update_op2,
                            10)
        self.sub5 = self.create_subscription(
        Float32,
        '/servo_angle',
        self.update_servo_angle,
        10)
        self.pub_mesh =  self.create_publisher(Marker, '/output_mesh', 10)
        self.tf_publisher = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #self.timer = self.create_timer(1.5, self.publish_ply_data2)

        self.success = False
        self.mesh = o3d.geometry.TriangleMesh()
        self.pcl = o3d.geometry.PointCloud()
        self.pcd = None
        self.model = None
        self.th = 0.0
        self.th_pcd = 0.0

        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.TSDF_voxel_length,
            sdf_trunc=self.TSDF_sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )

        self.correct_matrix = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])

        self.cnt = -1
        self.mesh_id = 0
        self.image_id = 0
        self.id = 0
        self.stateMove = True
        self.op_create_mesh = False
        self.op_save_mesh = False
        self.flag_refined = False

        self.world = o3d.geometry.PointCloud()

        if self.is_display_open3d:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.mesh = o3d.geometry.TriangleMesh()
            self.geom_added = False

    def save_tf(self,trans,fname):
        tx,ty,tz = trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z
        rx,ry,rz,rw = trans.transform.rotation.x,trans.transform.rotation.y ,trans.transform.rotation.z,trans.transform.rotation.w
        T = np.identity(4)
        rot = Rotation.from_quat(np.array([rx,ry,rz,rw]))
        T[:3,:3] = rot.as_matrix()
        T[:3,3] = np.array([tx,ty,tz])
        #np.array([tx,ty,tz,rx,ry,rz,rw])
        np.save(fname, T)
        #self.get_logger().info(f'saved {fname} : {T}\n')
        return T

    


    def CB_main(self, msg):
        #msg_time = msg.header.stamp
        #self.get_logger().info(msg_time)

        if self.op_create_mesh == True:
            self.cnt += 1
            #self.get_logger().info(f"\n{self.cnt}\n")

        if self.cnt % 100 == 0: 
            self.get_logger().info(f"\ncreate mesh\n")
            self.get_logger().info(f"\n{self.cnt}\n")
            #input_rgb = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
            input_rgb0 = CvBridge().imgmsg_to_cv2(msg.rgb).astype(np.uint8)
            input_d0 = CvBridge().imgmsg_to_cv2(msg.depth, "passthrough") #.astype(np.int32)
            input_rgb = cv2.resize(input_rgb0, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)
            input_d = cv2.resize(input_d0, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)
        
            color = o3d.geometry.Image(input_rgb)
            depth = o3d.geometry.Image(input_d)#.astype(np.uint16))

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth, depth_trunc=self.depth_range_max, convert_rgb_to_intensity=False)
            
            fx_d = msg.rgb_camera_info.k[0] * self.fxy
            fy_d = msg.rgb_camera_info.k[4] * self.fxy
            cx_d = msg.rgb_camera_info.k[2] * self.fxy
            cy_d = msg.rgb_camera_info.k[5] * self.fxy
            height, width = input_d.shape
            #self.get_logger().info(f"\n{width,height,fx_d,fy_d,cx_d,cy_d}\n")
            pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                width, height, fx_d,fy_d, cx_d, cy_d
            )

            camera_pose = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])

            
            try:
                #trans = self.tf_buffer.lookup_transform(self.frame_id_ground, self.frame_id_depth, rclpy.time.Time())
                trans1 = self.tf_buffer.lookup_transform('map', 'robot_base', rclpy.time.Time())
                trans2 = self.tf_buffer.lookup_transform('robot_base', 'end_effector', rclpy.time.Time())
                trans3 = self.tf_buffer.lookup_transform('camera_link', 'camera_color_optical_frame', rclpy.time.Time())
                trans4 = self.tf_buffer.lookup_transform('map', 'turn_table', rclpy.time.Time())
                trans5 = self.tf_buffer.lookup_transform('map', 'camera_color_optical_frame', rclpy.time.Time())

                new_pcd0 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
                new_pcd = new_pcd0.voxel_down_sample(0.001) #0.0025
                #o3d.visualization.draw_geometries([new_pcd])

                dir0 = f'./dataset_paramEst/data{self.id}'
                if not os.path.exists(dir0):
                    os.makedirs(dir0)

                T1 = self.save_tf(trans1, f'{dir0}/trans1.npy')
                T2 = self.save_tf(trans2, f'{dir0}/trans2.npy')
                T3 = self.save_tf(trans3, f'{dir0}/trans3.npy')
                T4 = self.save_tf(trans4, f'{dir0}/trans4.npy')
                o3d.io.write_point_cloud(f'{dir0}/new_pcd.ply', new_pcd, write_ascii=True)


                T3[:3,:3]=np.identity(3)
                Td = np.identity(4)
                Td[:3,3] = np.array([0,0,0.005])

                camera_pose = T1@T2@Td@T3
                new_pcd.transform(camera_pose)

                self.id += 1

                if self.pcd != None:
                    th = -self.th
                    dth = th - self.th_pcd
                    angle = np.array([th,dth])
                    np.save(f'{dir0}/angle.npy', angle)
                    self.get_logger().info(f"{dth*180/np.pi}")
                    vc = T4[:3,3]
                    #vc = np.array([0.6538, 0.3105, 0.0415])
                    axis = np.array([0, 0, 1])

                    # 回転行列を作成
                    R = self.pcd.get_rotation_matrix_from_xyz((0, 0, dth))

                    # 回転前に点群を回転中心に平行移動
                    self.pcd.translate(-vc)

                    # 回転を適用
                    self.pcd.rotate(R, center=(0, 0, 0))

                    # 回転後に点群を元の位置に戻す
                    self.pcd.translate(vc)

                    self.th_pcd = th
                    
                    previous_pcd = self.pcd
                    trans_icp = np.identity(4)
                    

                    """
                    # ICPアルゴリズムを使用してポーズの微調整
                    threshold = 0.02 #0.02  #0.01  # 適切な閾値を設定
             
                    icp_result = o3d.pipelines.registration.registration_icp(
                        new_pcd, previous_pcd, threshold, trans_icp,
                        o3d.pipelines.registration.TransformationEstimationPointToPoint())
                    """
                    voxel_radius = [0.04, 0.02, 0.01]
                
                    #max_iter = [50, 30, 14]
                    max_iter = [50, 30, 14] * 2
                    for scale in range(3):
                        iter = max_iter[scale]
                        radius = voxel_radius[scale]
                        print([iter, radius, scale])

                        print("3-1. Downsample with a voxel size %.2f" % radius)
                        #source_down = new_pcd
                        #target_down = previous_pcd
                        source_down = new_pcd.voxel_down_sample(radius)
                        target_down = previous_pcd.voxel_down_sample(radius)
                        print("3-2. Estimate normal.")
                        source_down.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
                        target_down.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

                        print("3-3. Applying colored point cloud registration")
                        icp_result = o3d.pipelines.registration.registration_colored_icp(
                            source_down, target_down, radius, trans_icp,
                            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                                            relative_rmse=1e-6,
                                                                            max_iteration=iter))
                    
                    trans_icp = icp_result.transformation
                    t_vec = trans_icp[:3,3]
                    t = np.linalg.norm(t_vec, ord=2)
                    self.get_logger().info(f"\n norm : {t}\n")
                    new_pcd.transform(trans_icp)

                    self.pcd += new_pcd
                    self.world = draw_coodinate()
                    origin = np.array([0,0,0])
                    center = draw_vector(origin,T4[:3,3])
                    self.world += center
                    self.world += self.pcd
                    self.get_logger().info(f"\nAdded first pcd\n")
                    
                    
                    #self.pcd += new_pcd
                    #self.world += new_pcd
                    self.get_logger().info(f"\nAdded new pcd\n")


                    new_pcd_down = new_pcd.voxel_down_sample(0.002)
                    o3d.io.write_point_cloud(f'{dir0}/new_pcd_mqtt.ply', new_pcd, write_ascii=True)
                    o3d.io.write_point_cloud(f'{dir0}/new_pcd_down_mqtt.ply', new_pcd_down, write_ascii=True)
                    #self.publish_ply_data2()
                else:
                    th = -self.th
                    self.th_pcd = th
                    self.pcd = new_pcd

                    self.world = draw_coodinate()
                    origin = np.array([0,0,0])
                    center = draw_vector(origin,T4[:3,3])
                    self.world += center
                    self.world += new_pcd
                    self.get_logger().info(f"\nAdded first pcd\n")

                    new_pcd_down = new_pcd.voxel_down_sample(0.002)
                    o3d.io.write_point_cloud(f'{dir0}/new_pcd_mqtt.ply', new_pcd, write_ascii=True)
                    o3d.io.write_point_cloud(f'{dir0}/new_pcd_down_mqtt.ply', new_pcd_down, write_ascii=True)

                o3d.visualization.draw_geometries([self.pcd])


                
            except Exception as e:
                # Handle the exception
                self.get_logger().info(f"An error occurred: {e}")
                #return -1
                self.success = False


    def publish_ply_data(self, mesh):
        topic = "ply_data"
        vertices = np.round(np.asarray(mesh.vertices), 4)
        vertex_colors = np.round(np.asarray(mesh.vertex_colors), 4)

        mesh_data = {
            "vertices": vertices.tolist(),
            "vertex_colors": vertex_colors.tolist()
        }
        json_message = json.dumps(mesh_data)
        with open("meshdata.json", 'w') as json_file:
            json.dump(mesh_data, json_file, indent =4)
        self.client.publish(topic, json_message, qos=1)  # qos: Quality of Service (QoS) level
        self.get_logger().info(f"Sent mesh data via MQTT on topic '{topic}'")

    def publish_ply_data2(self):
        if self.pcd != None:
            pcd = self.pcd.voxel_down_sample(0.0043) #0.004
            o3d.io.write_point_cloud(f"./data/pcd_down_{self.mesh_id}.ply", pcd)
            topic = "ply_data"
            vertices = np.round(np.asarray(pcd.points), 4)
            vertex_colors = np.round(np.asarray(pcd.colors), 4)

            mesh_data = {
                "vertices": vertices.tolist(),
                "vertex_colors": vertex_colors.tolist(),
            }
            json_message = json.dumps(mesh_data)
            with open("meshdata.json", 'w') as json_file:
                json.dump(mesh_data, json_file, indent =4)
            self.client.publish(topic, json_message, qos=1)  # qos: Quality of Service (QoS) level
            self.get_logger().info(f"Sent mesh data via MQTT on topic '{topic}'")

    def update_stateMove(self,msg):
        self.stateMove = msg.data
        #self.get_logger().info(f'Received: {self.stateMove}')

    def update_op(self,msg):
        self.op_create_mesh = msg.data
        self.get_logger().info(f'Received: {self.op_create_mesh}')

    def update_op2(self,msg):
        self.op_save_mesh = msg.data
        #self.get_logger().info(f'Received: {self.op_create_mesh}')

    def update_servo_angle(self,msg):
        self.th = msg.data
        #self.get_logger().info(f'{self.th}')


    def init_param(self):
        self.declare_parameter('image_scale', 1.0)
        self.declare_parameter('frame_id_ground', "map")
        #self.declare_parameter('frame_id_depth', "depth_camera_optical_frame")
        self.declare_parameter('frame_id_depth', "camera_color_optical_frame")
        self.declare_parameter('depth_range_max', 4.0)
        self.declare_parameter('is_display_open3d', True)
        self.declare_parameter('marker_ns', "marker")
        self.declare_parameter('TSDF.voxel_length',4.0 / 512.0)
        self.declare_parameter('TSDF.sdf_trunc',0.04)
        
        self.fxy = self.get_parameter("image_scale").get_parameter_value().double_value
        self.frame_id_ground = self.get_parameter('frame_id_ground').get_parameter_value().string_value
        self.frame_id_depth = self.get_parameter('frame_id_depth').get_parameter_value().string_value
        self.depth_range_max = self.get_parameter('depth_range_max').get_parameter_value().double_value
        self.is_display_open3d = self.get_parameter('is_display_open3d').get_parameter_value().bool_value
        self.marker_ns = self.get_parameter('marker_ns').get_parameter_value().string_value
        self.TSDF_voxel_length = self.get_parameter('TSDF.voxel_length').get_parameter_value().double_value
        self.TSDF_sdf_trunc = self.get_parameter('TSDF.sdf_trunc').get_parameter_value().double_value

def main():
    rclpy.init()
    node = RGBD2MESH()
    rclpy.spin(node)
    rclpy.shutdown()
    node.vis.destroy_window()
    del node.vis

if __name__ == '__main__':
    main()