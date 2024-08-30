#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import paho.mqtt.client as mqtt
import json
from threading import Thread

broker_address = "192.168.207.22"
broker_port = 1883
topic = "webxr/pose"

class KeyboardCmdRot(Node):

    def __init__(self):
        super().__init__('keyboard_cmd_rot')
        # MQTT クライアントのセットアップ
        self.client = mqtt.Client()
        # MQTT ブローカーに接続
        self.client.connect(broker_address, broker_port, 60)
        # メッセージ受信時のコールバック関数を設定
        self.client.on_message = self.on_message
        # MQTT クライアントが指定したトピックをサブスクライブ
        self.client.subscribe(topic)

        self.mqtt_thread = Thread(target=self.client.loop_forever)
        self.mqtt_thread.start()

        self.pub = self.create_publisher(String, '/cmd_rot', 10)
        self.pub2 = self.create_publisher(Bool, '/servo_state_move', 10)
        self.pub3 = self.create_publisher(Bool, '/op_create_mesh', 10)
        self.pub4 = self.create_publisher(Bool, '/op_save_mesh', 10)

        self.b0_value = 0.0
        self.bm_value = 0.0
        self.bm_statePush = False
        self.bA_statePush = False
        self.bm_statePush = False
        self.b0_cnt = 0

        self.current_bm_value = 0.0
        self.last_bm_value = 0.0

        self.current_b0_value = 0.0
        self.last_b0_value = 0.0

        self.state = 0
        self.create_cnt = 0
        self.save_cnt = 0
        self.flag_start = False
        self.flag_save = False
        self.over_threshold = False
        self.over_threshold2 = False
        self.th = 0.7
        self.create_timer(0.1, self.publish_op)
        #self.create_timer(0.1, self.publish_op)
        #self.get_logger().info('Node has been started.')

    def publish_op(self):
        try:
            key = input('r: turn right, l: turn left, i: set init , o: create mesh , p: stop > ')
            msg = String()
            msg2 = Bool()
            msg3 = Bool()
            msg4 = Bool()

            if 'r' in key:
                msg.data = 'right'
            elif 'l' in key:
                msg.data = 'left'
            elif 'i' in key:
                msg.data = 'init'
            elif 'o' in key:
                msg3.data = True
                self.pub3.publish(msg3)
            elif 'p' in key:
                msg3.data = False
            elif 'q' in key:
                msg4.data = True
                self.pub4.publish(msg4)
            else:
                msg.data = 'other'

            msg2.data = True

            self.pub.publish(msg)
            self.pub2.publish(msg2)
            
        except KeyboardInterrupt:
            self.shutdown()



        # 受信したメッセージを処理するコールバック関数
    def on_message(self, client, userdata, message):
        #print("Message Received")
        # JSON形式のメッセージをデコード
        nmsg_dict = json.loads(message.payload.decode("utf-8"))
        #print(nmsg_dict)
        self.b0_value = float(nmsg_dict['pad']['b0'])
        self.bm_value = float(nmsg_dict['pad']['bm'])
        self.bA_statePush = bool(nmsg_dict['pad']['bA'])
        self.bm_statePush = bool(nmsg_dict['pad']['bB'])
        #self.publish_op()

'''
    def publish_op(self):
        msg_rot = String()
        msg_create_mesh = Bool()
        msg_save_mesh = Bool()
        
        #flag_start
        self.current_bm_value = self.bm_value
        if self.last_bm_value <= 0.5 and self.current_bm_value > 0.5:
            self.over_threshold = True

        if self.over_threshold == True and self.current_bm_value <= 0.5:
            self.flag_start = not self.flag_start
            self.over_threshold = False
            print(f"Flag_start:{self.flag_start}")

        self.last_bm_value = self.current_bm_value

        
        msg_save_mesh.data = self.flag_save
        self.last_b0_value = self.current_b0_value
        

        if self.flag_start == True:
            if self.bA_statePush == True:
                msg_rot.data = 'right'
            elif self.bm_statePush == True:
                msg_rot.data = 'left'
            
            #create_mesh
            if self.b0_value > 0.7:
                self.create_cnt += 1
            else:
                self.create_cnt = 0
        
            if self.create_cnt == 15:
                msg_create_mesh.data = True
                self.pub3.publish(msg_create_mesh)

            #else:
            #    msg_create_mesh.data = False

            #save_mesh
            self.current_b0_value = self.b0_value
            if self.last_b0_value > 0.5 and self.current_b0_value <= 0.5:
            #if self.last_b0_value <= 0.5 and self.current_b0_value > 0.5:
                self.flag_save = True
                print(f"Flag_save:{self.flag_save}")
                self.pub4.publish(msg_save_mesh)

            """
            if self.b0_value > 0.7:
                self.save_cnt += 1
            else:
                self.save_cnt = 0
                msg_save_mesh.data = False
            
            if self.save_cnt == 10:
                msg_save_mesh.data = True
            """

        #print(self.b0_cnt)
        #print(self.b0_value)
        #print(self.bm_value)
        #print(self.bA_statePush)

        self.pub.publish(msg_rot)
        #self.pub3.publish(msg_create_mesh)
'''
    

"""
    def timer_callback(self):
        try:
            key = input('r: turn right, l: turn left, i: set init , o: create mesh , p: stop > ')
            msg = String()
            msg2 = Bool()
            msg3 = Bool()
            msg4 = Bool()

            if 'r' in key:
                msg.data = 'right'
            elif 'l' in key:
                msg.data = 'left'
            elif 'i' in key:
                msg.data = 'init'
            elif 'o' in key:
                msg3.data = True
            elif 'p' in key:
                msg3.data = False
            elif 'q' in key:
                msg4.data = True
                self.pub4.publish(msg4)
            else:
                msg.data = 'other'

            msg2.data = True

            self.pub.publish(msg)
            self.pub2.publish(msg2)
            self.pub3.publish(msg3)
            
        except KeyboardInterrupt:
            self.shutdown()
"""

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCmdRot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

