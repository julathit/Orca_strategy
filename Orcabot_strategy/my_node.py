#! /usr/bin/env python3


import rclpy
from rclpy.node import Node
from grsim_ros_bridge_msgs.msg import SSL
from Orcabot_strategy.component import Robot
from Orcabot_strategy.component import C_manulDrive

class TestSSL(Node):
    def __init__(self):
        super().__init__('test_ssl')
        self.publisher = self.create_publisher(SSL, '/robot_blue_0/cmd', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.my_robot = Robot(self,"blue",0)
        self.manulD = C_manulDrive(self,"blue",0)
        
    def timer_callback(self):

        self.manulD.execute()
        # self.my_robot.sendCommand(5.0,0.0,0.0)


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