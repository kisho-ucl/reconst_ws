import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import tf2_ros
import geometry_msgs.msg
import json
import open3d as o3d
import numpy as np
from threading import Thread
from scipy.spatial.transform import Rotation as R
import datetime
from builtin_interfaces.msg import Time

# MQTT ブローカーの情報
broker_address = "192.168.207.22"
broker_port = 1883
topic = "new_robot_pose"

class Robot_hand(Node):

    def __init__(self):
        super().__init__('Robot_hand')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        #self.timer = self.create_timer(0.1, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0
        self.time = None
        self.last_time = None
        self.flag_received = False
        self.flag_update = False
        self.a = 0.8
        self.h = 0.10

        # MQTT クライアントのセットアップ
        self.client = mqtt.Client()
        # MQTT ブローカーに接続
        self.client.connect(broker_address, broker_port, 60)
        # メッセージ受信時のコールバック関数を設定
        self.client.on_message = self.on_message
        # MQTT クライアントが指定したトピックをサブスクライブ
        self.client.subscribe(topic)
        # MQTT クライアントのループをスレッドで実行
        self.mqtt_thread = Thread(target=self.client.loop_forever)
        self.mqtt_thread.start()
        self.timer = self.create_timer(0.1, self.publish_tf)
            
    def on_message(self, client, userdata, message):
        #print("Message Received")
        # JSON形式のメッセージをデコード
        nmsg_dict = json.loads(message.payload.decode("utf-8"))
        self.x = float(nmsg_dict['x']) / 1000
        self.y = float(nmsg_dict['y']) / 1000
        self.z = float(nmsg_dict['z']) / 1000
        self.rot_x = float(nmsg_dict['rot_x']) * np.pi/180
        self.rot_y = float(nmsg_dict['rot_y']) * np.pi/180
        self.rot_z = float(nmsg_dict['rot_z']) * np.pi/180
        self.time_str = str(nmsg_dict['time'])
        self.flag_received = True
        self.flag_update =  True
        self.id = 0
        #print(self.time_str,self.x,self.y,self.z)

    def publish_tf(self):
        if self.flag_received == True:
            t = geometry_msgs.msg.TransformStamped()
            if self.flag_update == True:
                dt = datetime.datetime.strptime(self.time_str, '%Y-%m-%d %H:%M:%S.%f')
                timestamp = dt.timestamp()
                sec = int(timestamp)
                nanosec = int((timestamp - sec) * 1e9)
                t.header.stamp = Time(sec=sec, nanosec=nanosec)
                self.get_logger().info('update robot pose')
                #pose = np.array([self.x, self.y, self.z, self.rot_x, self.rot_y, self.rot_z])
                #np.savetxt(f'/home/realsense/poselog/robot_pose_{self.id}.txt', pose, delimiter=',')
                #self.id += 1
            else:
                t.header.stamp = self.get_clock().now().to_msg()
                self.get_logger().info('non new robot pose')
                pose = np.array([self.x, self.y, self.z, self.rot_x, self.rot_y, self.rot_z])
                print(pose)


            t.header.frame_id = 'map'
            t.child_frame_id = 'robot_base'

            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

            t1 = geometry_msgs.msg.TransformStamped()
            t1.header.stamp = self.get_clock().now().to_msg()
            t1.header.frame_id = 'robot_base'
            t1.child_frame_id = 'end_effector'
            
            t1.transform.translation.x = self.x
            t1.transform.translation.y = self.y
            t1.transform.translation.z = self.z
            #rot0 = R.from_euler('xyz', [np.pi/2, 0.0, np.pi/2])
            rot = R.from_euler('xyz', [self.rot_x, self.rot_y, self.rot_z])
            #rot2 = rot0 * rot1
            q = rot.as_quat()
            #q = self.euler_to_quaternion(self.rot_x, self.rot_y, self.rot_z)
            t1.transform.rotation.x = q[0]
            t1.transform.rotation.y = q[1]
            t1.transform.rotation.z = q[2]
            t1.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t1)
            
            # end_effectorからcamera_linkへのTransform
            t2 = geometry_msgs.msg.TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg()
            t2.header.frame_id = 'end_effector'
            t2.child_frame_id = 'camera_link'
            
            t2.transform.translation.x = 0.0  # x座標
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.005 #0.01790 #0.0225 #0.0225 #0.7922  # エンドエフェクタからカメラまでの距離
            
            rot = R.from_euler('xyz', [0.0, -np.pi/2, np.pi/2])
            q = rot.as_quat()
            t2.transform.rotation.x = q[0]
            t2.transform.rotation.y = q[1]
            t2.transform.rotation.z = q[2]
            t2.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t2)

            #test
            t3 = geometry_msgs.msg.TransformStamped()
            t3.header.stamp = self.get_clock().now().to_msg()
            t3.header.frame_id = 'camera_link'
            t3.child_frame_id = 'my_color_optical_frame'
            
            t3.transform.translation.x = 0.0  # x座標
            t3.transform.translation.y = 0.0
            t3.transform.translation.z = 0.0
            
            rot = R.from_euler('xyz', [0.0, 0.0, -np.pi/2])
            q = rot.as_quat()
            t3.transform.rotation.x = q[0]
            t3.transform.rotation.y = q[1]
            t3.transform.rotation.z = q[2]
            t3.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t3)

            self.last_time = self.time
            self.flag_update = False

def main(args=None):
    rclpy.init(args=args)
    hand = Robot_hand()
    try:
        rclpy.spin(hand)
    except KeyboardInterrupt:
        pass
    hand.client.disconnect()
    hand.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
