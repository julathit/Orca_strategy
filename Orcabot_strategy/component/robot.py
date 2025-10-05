import numpy as np
import rclpy
from rclpy.node import Node
from grsim_ros_bridge_msgs.msg import SSL

from Orcabot_strategy.component.vission_handler import vission_handler as C_vission_handler

class Robot:
    def __init__(self,Test_ssl: Node,team: str,Id: int):
        self.role = None
        self.Test_ssl = Test_ssl
        
        self.team = team
        self.Id = Id
        
        self.ssl_msg: SSL = SSL()
        
    def getRole(self):
        return self.role

    def getPosition(self) -> np.array:
        
        if self.team == "blue":
            data = C_vission_handler().robot_tBlue
        else:
            data = C_vission_handler().robot_tYellow
        
        x,y = data[self.Id].x, data[self.Id].y
        
        return np.array([x,y])

    def getOrientation(self) -> float:
        
        if self.team == "blue":
            data = C_vission_handler().robot_tBlue
        else:
            data = C_vission_handler().robot_tYellow
            
        orientation = data[self.Id].orientation
        return orientation

    def __distanceToPoint(self, point: np.array) -> float:
        return np.linalg.norm(self.getPosition() - point)

    def __angToPoint(self, point: np.array) -> float:
        current_pos = self.getPosition()
    
        delta = point - current_pos
        
        dy = delta[1]
        dx = delta[0]
        
        angle_radians = np.arctan2(dy, dx)
        
        return angle_radians            
            
    def sendCommand(self,x: float, y: float, z: float, kickPower = False, dribbler = False):
        self.ssl_msg.cmd_vel.angular.z = z
        self.ssl_msg.cmd_vel.linear.x = x
        self.ssl_msg.cmd_vel.linear.y = y
        self.ssl_msg.kicker = kickPower
        self.ssl_msg.dribbler = dribbler
        # print(self.ssl_msg)
        self.Test_ssl.publisher.publish(self.ssl_msg)
        # self.Test_ssl.get_logger().info('Publishing: %s' % self.ssl_msg)