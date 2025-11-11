import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import os
import time
import math
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '4.0.0'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280

class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_sync_plan")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription
        
        # Allow configuring serial port and baud via ROS parameters (helps on Raspberry Pi)
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 1000000)
        port_param = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # Auto-detect common serial devices if port parameter is empty
        port = port_param
        if not port:
            # check USB serial devices first
            usb = os.popen("ls /dev/ttyUSB* 2>/dev/null").readline().strip()
            acm = os.popen("ls /dev/ttyACM* 2>/dev/null").readline().strip()
            serial0 = os.popen("ls /dev/serial0 2>/dev/null || true").readline().strip()
            ama = os.popen("ls /dev/ttyAMA* 2>/dev/null").readline().strip()
            candidates = [usb, acm, serial0, ama]
            port = next((p for p in candidates if p), '')

        if not port:
            self.get_logger().error('No serial port found for MyCobot. Set the "port" parameter to the correct device (e.g. /dev/serial0 or /dev/ttyUSB0).')
            raise RuntimeError('No serial port found for MyCobot')

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot280(port, baud)
        time.sleep(0.05)
        if self.mc.get_fresh_mode() == 0:
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)

        # RViz joint order to match published JointState names (include gripper controller)
        self.rviz_order = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
            'gripper_controller'
        ]

    def listener_callback(self, msg):
        # 创建字典将关节名称与位置值关联
        joint_state_dict = {name: msg.position[i] for i, name in enumerate(msg.name)}
        # 根据 RViz 顺序重新排列关节角度
        # Build arm angles (first 6 joints) and gripper value separately to match Pi API
        arm_angles = []
        gripper_value = 0

        for i, joint in enumerate(self.rviz_order):
            if joint in joint_state_dict:
                deg = round(math.degrees(joint_state_dict[joint]), 3)
            else:
                deg = 0.0

            # first 6 entries are arm joints, last is gripper_controller
            if i < 6:
                arm_angles.append(deg)
            else:
                # map gripper joint angle (radians) to gripper percent 0-100
                # same mapping used in slider_control_adaptive_gripper.py
                # gripper URDF limits: min=-0.74, max=0.15
                min_val = -0.74
                max_val = 0.15
                # note: slider_control maps radians to percentage by using the raw radian value
                # here we expect msg.position to be radians; use that mapping
                rad = joint_state_dict.get(joint, 0.0)
                mapped_value = (rad - min_val) / (max_val - min_val) * 100
                gripper_value = int(round(max(0.0, min(100.0, mapped_value)), 0))

        self.get_logger().info('sending arm angles: {} gripper: {}'.format(arm_angles, gripper_value))
        try:
            # send arm angles async and gripper separately (Pi API)
            self.mc.send_angles(arm_angles, 25, _async=True)
            # set_gripper_value(value, speed, gripper_type=1)
            self.mc.set_gripper_value(gripper_value, 80, gripper_type=1)
        except Exception as e:
            self.get_logger().error(f'Failed to send angles to MyCobot: {e}')


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
