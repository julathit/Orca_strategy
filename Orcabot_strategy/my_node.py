#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from grsim_ros_bridge_msgs.msg import SSL
from Orcabot_strategy.component import Robot
from Orcabot_strategy.component import C_manulDrive
from Orcabot_strategy.component import C_vission_handler

from krssg_ssl_msgs.msg import SSLDetectionFrame
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 


vision_qos_profile = QoSProfile(
    # C++ uses .best_effort(), so we must match it!
    reliability=ReliabilityPolicy.BEST_EFFORT, 
    # C++ uses KeepLast(10)
    history=HistoryPolicy.KEEP_LAST, 
    depth=10 
)

class TestSSL(Node):
    def __init__(self):
        super().__init__('test_ssl')
        Id = 0
        team = "blue"
        self.publisher = self.create_publisher(SSL, f"/robot_{team}_{Id}/cmd", 10)
        self.sub = self.create_subscription(SSLDetectionFrame, "/vision_n", self.subscription_callback, vision_qos_profile )
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.my_robot = Robot(self)
        self.manulD = C_manulDrive(self)
        self.vission = C_vission_handler()
        
    def timer_callback(self):

        self.manulD.execute()
        print(self.vission.robot_tBlue[0])
        # self.my_robot.sendCommand(5.0,0.0,0.0)
        
    def subscription_callback(self,msg):
        self.vission.data_pack(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestSSL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()