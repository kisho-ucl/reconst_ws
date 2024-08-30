import rclpy
from rclpy.node import Node
import logging
# msgs
from realsense2_camera_msgs.msg import RGBD
# from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header, Bool
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
        self.stateMove = True
        self.op_create_mesh = False
        self.op_save_mesh = False
        self.flag_refined = False

        if self.is_display_open3d:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.mesh = o3d.geometry.TriangleMesh()
            self.geom_added = False

    def CB_main(self, msg):
        """
        input_d = CvBridge().imgmsg_to_cv2(msg.depth, "passthrough") #.astype(np.int32)
        input_d = cv2.resize(input_d, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)
        fx_d = msg.rgb_camera_info.k[0] * self.fxy
        fy_d = msg.rgb_camera_info.k[4] * self.fxy
        cx_d = msg.rgb_camera_info.k[2] * self.fxy
        cy_d = msg.rgb_camera_info.k[5] * self.fxy
        height, width = input_d.shape
        self.get_logger().info(f"\n{width},{height},{fx_d},{fy_d},{cx_d},{cy_d}\n")
        """

        #self.get_logger().info(f"\nwait for operation\n")
        #if self.cnt == 80:
        #if self.op_create_mesh == True:
        #self.get_logger().info(f"\n{self.cnt}\n")
        if self.op_create_mesh == True:
            self.cnt += 1
            #self.get_logger().info(f"\n{self.cnt}\n")

        if self.cnt % 50 == 0: 
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
                trans = self.tf_buffer.lookup_transform('turn_table', 'camera_color_optical_frame', rclpy.time.Time())
                #self.get_logger().info(f"Translation: {trans.transform.translation}")
                #self.get_logger().info(f"Rotation: {trans.transform.rotation}")
                tx,ty,tz = trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z
                rx,ry,rz,rw = trans.transform.rotation.x,trans.transform.rotation.y ,trans.transform.rotation.z,trans.transform.rotation.w
                rot = Rotation.from_quat(np.array([rx,ry,rz,rw]))
                camera_pose[:3,:3] = rot.as_matrix()
                camera_pose[:3,3] = np.array([tx,ty,tz])
                refined_pose = camera_pose
                
                new_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
                #new_pcd = new_pcd0.voxel_down_sample(0.001) #0.0025
                #o3d.io.write_point_cloud(f"/home/realsense/exp/open3d/datasets2/new_pcd_{self.image_id}.ply", new_pcd, write_ascii=True)
                new_pcd.transform(camera_pose)
                #camera_pose = np.dot(self.correct_matrix,camera_pose)
                if self.pcd != None:
                    #previous_pcd = self.pcd
                    previous_pcd = o3d.geometry.PointCloud()
                    mesh = self.volume.extract_triangle_mesh()
                    mesh.compute_vertex_normals() 
                    previous_pcd.points = o3d.utility.Vector3dVector(mesh.vertices)
                    previous_pcd.colors =o3d.utility.Vector3dVector(mesh.vertex_colors)
                    #o3d.visualization.draw_geometries([previous_pcd])

                    #pcd = self.pcd
                    trans_icp = np.identity(4)
                    
                    """
                    # ICPアルゴリズムを使用してポーズの微調整
                    threshold = 0.02 #0.02  #0.01  # 適切な閾値を設定
             
                    icp_result = o3d.pipelines.registration.registration_icp(
                        new_pcd, previous_pcd, threshold, trans_icp,
                        o3d.pipelines.registration.TransformationEstimationPointToPoint())

                    """
                    #voxel_radius = [0.04, 0.02, 0.01]
                    voxel_radius = [0.008, 0.004, 0.001]
                
                    #max_iter = [50, 30, 14]
                    max_iter = [50, 30, 14]
                    for scale in range(2):
                        iter = max_iter[scale]
                        radius = voxel_radius[scale]
                        print([iter, radius, scale])

                        print("3-1. Downsample with a voxel size %.2f" % radius)
                        #source_down = new_pcd
                        #target_down = previous_pcd
                        source_down = new_pcd.voxel_down_sample(radius)
                        target_down = previous_pcd.voxel_down_sample(radius)
                        o3d.visualization.draw_geometries([target_down,source_down])
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
                        
                    

                    #o3d.visualization.draw_geometries([self.pcd])

                    #trans_icp = icp_result.transformation
                    norm =  np.linalg.norm(trans_icp[:3,3] )
                    self.get_logger().info(f"\n{norm}\n")
                    if norm > 0.04: #0.038:
                        raise ValueError("bad trans")
                    #new_pcd.transform(trans_icp)


                    refined_pose = trans_icp @ camera_pose
                    

                    self.pcd += new_pcd
                    self.get_logger().info(f"\nadded new pcd\n")
                    #self.publish_ply_data2()
                else:
                    self.pcd = new_pcd
                    self.get_logger().info(f"\nfirst pcd\n")

                #o3d.visualization.draw_geometries([self.pcd])
                # Destroy the visualization window
                #vis.destroy_window()   
                #time.sleep(1)

                # 更新されたカメラポーズを取得
                #refined_pose = icp_result.transformation
                #camera_pose = refined_pose

                #o3d.io.write_point_cloud(f"/home/realsense/exp/open3d/datasets2/pcd_{self.image_id}.ply", self.pcd, write_ascii=True)
                #np.savetxt(f'/home/realsense/exp/open3d/datasets2/camera_pose_{self.image_id}.txt', camera_pose, delimiter=',')
                #np.savetxt(f'/home/realsense/exp/open3d/datasets2/camera_pose_refined_{self.image_id}.txt', refined_pose, delimiter=',')
                #mesh = self.volume.extract_triangle_mesh()
                #mesh.compute_vertex_normals() 
                #o3d.io.write_triangle_mesh(f"./data/mesh_{self.image_id}.ply", mesh, write_ascii=True)

                self.image_id += 1
                self.success = True

                #if self.op_save_mesh==True:
                #self.correct_matrix = np.dot(refined_pose, np.linalg.inv(camera_pose))
                #self.get_logger().info(f"\nupdated\n")

                #self.get_logger().info(f"\ncamera_pose was refined\n")
                #self.previous_pcd = new_pcd
                
                """
                t = matrix_to_transform(self.refined_pose)
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'map'  # 親フレーム名を設定
                t.child_frame_id = 'camera_link_refined'  # 子フレーム名を設定
                self.publisher.sendTransform(t)
                #camera_pose = refined_pose
                
                self.previous_pcd = new_pcd
                """
                #self.get_logger().info(f"Translation: {trans.transform.translation}")
                #self.get_logger().info(f"Rotation: {trans.transform.rotation}")
                self.volume.integrate(
                rgbd,
                pinhole_camera_intrinsic,
                #camera_pose
                np.linalg.inv(refined_pose)
                )

                
                
            except Exception as e:
                # Handle the exception
                self.get_logger().info(f"An error occurred: {e}")
                #return -1
                self.success = False
            
            #np.savetxt(f'/home/realsense/exp/open3d/datasets/camera_pose_{self.image_id}.txt', camera_pose, delimiter=',')
            #rgb_filename = os.path.join('/home/realsense/exp/open3d/datasets', f'image_rgb_{self.image_id}.png')
            #depth_filename = os.path.join('/home/realsense/exp/open3d/datasets', f'image_depth_{self.image_id}.png')

            # Save the images
            #cv2.imwrite(rgb_filename, input_rgb0)
            #cv2.imwrite(depth_filename, input_d0)

            

            # https://www.open3d.org/docs/release/python_api/open3d.geometry.TriangleMesh.html
          
            #self.op_create_mesh = False

        #if self.cnt % 500 == 0:
        #    o3d.visualization.draw_geometries([self.pcd])
        
        
        if self.cnt % 500 == 0:
            pcd = o3d.geometry.PointCloud()
            mesh = self.volume.extract_triangle_mesh()
            #pcd = self.pcd
            mesh.compute_vertex_normals() 
            o3d.io.write_triangle_mesh(f"./data/mesh_{self.mesh_id}.ply", mesh, write_ascii=True)
            #o3d.io.write_point_cloud(f"./data/pcd_{self.mesh_id}.ply", pcd)
            self.get_logger().info(f"\nsaved\n")

            pcd.points = o3d.utility.Vector3dVector(mesh.vertices)
            pcd.colors =o3d.utility.Vector3dVector(mesh.vertex_colors)
            o3d.visualization.draw_geometries([pcd])
            #o3d.visualization.draw_geometries([self.pcd])

            self.mesh_id += 1

        #if self.op_save_mesh == True:
        #mesh = self.volume.extract_triangle_mesh()
        #mesh.compute_vertex_normals() 
        #pcd = self.pcd
        
        """
        if self.mesh_id > 7:
            mesh = self.volume.extract_triangle_mesh()
            mesh.compute_vertex_normals() 
            o3d.io.write_triangle_mesh(f"./data/mesh_{self.mesh_id}.ply", mesh, write_ascii=True)
            o3d.io.write_point_cloud(f"./data/pcd_{self.mesh_id}.ply", pcd)
        """
        #mesh = self.volume.extract_triangle_mesh()
        #mesh.compute_vertex_normals() 
        #o3d.io.write_triangle_mesh(f"./data/mesh_{self.mesh_id}.ply", mesh, write_ascii=True)
        #self.get_logger().info(f"\n ---saved {self.mesh_id} --- \n")
        #self.mesh_id += 1
        #self.publish_ply_data(mesh)
        #pcd_down = pcd.voxel_down_sample(0.01) #0.004
        #self.publish_ply_data2(pcd_down)
        #self.op_save_mesh = False

    #def timer_callback(self):
    #    self.publish_ply_data(self.mesh)

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