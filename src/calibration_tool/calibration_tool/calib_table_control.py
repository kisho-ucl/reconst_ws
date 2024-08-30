import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String, Bool
import numpy as np
import tf2_ros
import serial
import time
import paho.mqtt.client as mqtt
import json

STRETCH = 0x01
SPEED = 0x02
CURRENT = 0x03
TEMPERATURE = 0x04

def pos2deg(pos):
    deg = (pos - 7500) * 135.0 / 4000
    return deg

def deg2pos(deg):
    pos = deg * 4000 / 135.0 + 7500
    return int(pos)

class Servo(Node):
    def __init__(self):
        super().__init__('control_servo')
        self.krs = serial.Serial('/dev/ttyUSB0', baudrate=115200, parity=serial.PARITY_EVEN, timeout=0.5)
        self.rot_speed = 30
        self.rot_once = 60.0 #45.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.cntStop = 0
        self.stateMove = False
        reData = self.krs_setValue(0, SPEED, 30)
        print(reData)
        self.publisher = self.create_publisher(Bool, '/servo_state_move', 10)
        self.pub2 = self.create_publisher(Bool, '/op_detect_marker', 10)
        #self.pub3 = self.create_pub3(Bool, '/create_mesh', 10)
        self.sub = self.create_subscription(String, '/cmd_rot', self.cmd_rot_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.publish_data)
        self.deg = 0.0
        self.last_deg = 0.0
        self.pos = 7500
        self.flag_find_loc = True
        self.flag_detect = True

        #self.timer = self.create_timer(0.1, self.publish_turntable_angle)

    def cmd_rot_callback(self,msg):
        data = msg.data
        #self.get_logger().info(f'{data}')
        if data == 'right':
            self.turnRight()
        elif data == 'left':
            self.turnLeft()
        elif data == 'init':
            self.setInit()

    def calib_turn(self):
        dur = 5
        time.sleep(dur)
        self.publish_op()
        
        time.sleep(dur)
        self.turnRight()
        time.sleep(dur)
        self.publish_op()
        self.publish_data()
        print(self.deg)

        time.sleep(dur)
        self.turnLeft()
        time.sleep(1)
        self.publish_data()
        self.turnLeft()
        time.sleep(dur)
        self.publish_op()
        self.publish_data()
        print(self.deg)

        self.setInit()
        time.sleep(dur)
        self.publish_data()

        """

        #self.publish_op()
        time.sleep(dur)
        self.turnRight()
        time.sleep(dur)
        self.publish_data()
        print(self.deg)

        #self.publish_op()
        time.sleep(dur)
        self.turnRight()
        time.sleep(dur)
        self.publish_data()
        print(self.deg)
        """

        self.publish_op()


    
    def publish_op(self):
        msg = Bool()
        msg.data = self.flag_detect
        self.flag_detect = not self.flag_detect
        self.pub2.publish(msg)
        

    def find_table_location(self):
        with open('/home/realsense/test_ws/src/calibration_tool/calibration_tool/info/table_location.txt', 'r') as file:
            # 1行目を読み込む
            line = file.readline().strip()
            # カンマで分割し、浮動小数点数に変換
            self.x, self.y, self.z = map(float, line.split(','))
        """
        try:
            trans = self.tf_buffer.lookup_transform('map', 'marker', rclpy.time.Time())
            tx,ty,tz = trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z
            self.x = tx   
            self.y = ty   
            self.z = tz
            self.flag_find_loc = False
            with open('/home/realsense/test_ws/src/calibration_tool/calibration_tool/info/table_location.txt', 'w') as f:
                f.write(f"{tx},{ty},{tz}\n")
            self.get_logger().info(f'found table location.')

        except:
            self.get_logger().info('marker was not found')
        """
        
    def publish_data(self):
        try:
            bl, reData = self.krs_getPos36_CMD(0)
            pos=reData
            #self.pos = pos
        except:
            pass

        if (self.stateMove == True) and (self.pos==pos):
            self.cntStop += 1
        
        if self.cntStop > 20:
            self.stateMove = False
            self.cntStop = 0
            msg = Bool()
            msg.data = False
            self.publisher.publish(msg)

        self.pos = pos
        self.deg = pos2deg(self.pos)

        if self.flag_find_loc == True: self.find_table_location()
 
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'        
        t.child_frame_id = 'turn_table'
        th = -self.deg*3.14/180
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        r = R.from_euler('xyz', [0.0, 0.0, th])
        q = r.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        #msg = Bool()
        #msg.data = self.stateMove
        #self.publisher.publish(msg)
        #self.get_logger().info(f'Published: {msg.data}')
    
    def publish_turntable_angle(self):
        topic="webxr/turntable_angle"
        mesh_data = {
            "x": 0.0,
            "y": 0.0,
            "z": self.deg ,
        }
        json_message = json.dumps(mesh_data)
        self.client.publish(topic, json_message, qos=0)  # qos: Quality of Service (QoS) level
        self.get_logger().info(f"Sent turntable angle via MQTT on topic '{topic}'")
        self.last_deg = self.deg
 

    def krs_setPos_CMD(self, servo_id, pos):
        txCmd = [0x80 | servo_id,
                 pos >> 7 & 0x7f,
                 pos & 0x7f]
        self.krs.write(txCmd)
        rxCmd = self.krs.read(6)
        if len(rxCmd) == 0:
            return False
        else:
            return True, (rxCmd[4] << 7) + rxCmd[5]

    def krs_setFree_CMD(self, servo_id):
        bl, Value = self.krs_setPos_CMD(servo_id, 0)
        return bl, Value

    def krs_getValue_CMD(self, servo_id, sc):
        txCmd = [0xA0 | servo_id,
                 sc]
        self.krs.write(txCmd)
        rxCmd = self.krs.read(5)

        if len(rxCmd) == 0:
            return False, 0
        else:
            return True, rxCmd[4]

    def krs_getPos36_CMD(self, servo_id):
        txCmd = [0xA0 | servo_id,
                 0x05]
        self.krs.write(txCmd)
        rxCmd = self.krs.read(6)
        if len(rxCmd) == 0:
            return False, 0
        else:
            return True, (rxCmd[4] << 7) + rxCmd[5]

    def krs_getPos35_CMD(self, servo_id):
        bl, Value = self.krs_setPos_CMD(servo_id, 0)
        bl, Value = self.krs_setPos_CMD(servo_id, Value)
        return bl, Value

    def krs_setValue(self, servo_id, sc, Value):
        if sc == 0x00:
            return False

        txCmd = [0xC0 | servo_id,
                 sc,
                 Value]

        self.krs.write(txCmd)
        rxCmd = self.krs.read(6)
        if len(rxCmd) == 0:
            return False
        else:
            if rxCmd[5] == Value:
                return True
            else:
                return False

    def krs_getID_CMD(self):
        txCmd = [0xFF,
                 0x00,
                 0x00,
                 0x00]

        self.krs.write(txCmd)
        rxCmd = self.krs.read(5)

        if len(rxCmd) == 0:
            return False, 0
        else:
            return True, rxCmd[4] & 0x1F

    def setInit(self):
        self.stateMove = True
        target = deg2pos(0)
        bl, reData = self.krs_setPos_CMD(0, target)

    def turnRight(self):  # Added self parameter
        self.stateMove = True
        target = deg2pos(self.deg + self.rot_once)
        if target > 11500: target=11500
        bl, reData = self.krs_setPos_CMD(0, target)
    def turnLeft(self):  # Added self parameter
        self.stateMove = True
        target = deg2pos(self.deg - self.rot_once)
        if target < 3500: target = 3500
        bl, reData = self.krs_setPos_CMD(0, target)

    def reviDeg(self):
        time.sleep(1.0)
        bl, reData = self.krs_getPos36_CMD(0)
        self.pos = reData
        self.deg = pos2deg(reData)
        print("servo:", self.deg, "deg")


def main():
    rclpy.init()
    node = Servo()
    node.setInit()
    node.calib_turn()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.krs.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()