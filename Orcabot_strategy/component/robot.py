import numpy as np
import rclpy
from rclpy.node import Node
from grsim_ros_bridge_msgs.msg import SSL

class Robot:
    def __init__(self,team:str, rid: int):
        self.id: int = rid
        self.role = None
        self.team = team
        
        self.ssl_msg: SSL = SSL()
        
    def getRole(self):
        return self.role

    def getPosition(self) -> np.array:
        ...

    def getOrientation(self) -> float:
        ...

    def __distanceToPoint(self, point: np.array) -> float:
        ...

    def __angToPoint(self, point: np.array) -> float:
        ...
            
    def __distance(self, origins: np.array , points: list) -> list:
        ...
            
    def sendCommand(self,TestSSL: Node ,x: float, y: float, z: float, kickPower = False, dribbler = False):
        self.ssl_msg.cmd_vel.angular.z = z
        self.ssl_msg.cmd_vel.linear.x = x
        self.ssl_msg.cmd_vel.linear.y = y
        self.ssl_msg.kicker = kickPower
        self.ssl_msg.dribbler = dribbler
        # print(self.ssl_msg)
        TestSSL.publisher.publish(self.ssl_msg)
        TestSSL.get_logger().info('Publishing: %s' % self.ssl_msg)