import numpy as np
import rclpy
from rclpy.node import Node
from grsim_ros_bridge_msgs.msg import SSL
from Orcabot_strategy.utils.simpleMovementTools import SToV as SToVCal
from Orcabot_strategy.utils.simpleMovementTools import PID as PIDcal
from Orcabot_strategy.utils.a_star import a_star as f_a_star
from Orcabot_strategy.utils.a_star import h_euclidean as h
from Orcabot_strategy.utils.a_star import initial_node as f_initial_node
from Orcabot_strategy.utils.state import state as State
from Orcabot_strategy.utils.a_star import reconstruct_path as f_reconstruct_path

import time

from Orcabot_strategy.component.vission_handler import vission_handler as C_vission_handler

class Robot:
    def __init__(self,Test_ssl: Node,team: str,Id: int):
        self.role = None
        self.Test_ssl = Test_ssl
        
        self.team = team
        self.Id = Id
        
        self.ssl_msg: SSL = SSL()
        
        # handle pid
        if self.team == "blue":
            data = C_vission_handler().robot_tBlue
        else:
            data = C_vission_handler().robot_tYellow
        self.data = data
        x,y = data[self.Id].x, data[self.Id].y
        o = data[self.Id].orientation
        self.oldPosition = np.array([x,y,o])
        self.oldTime = time.time()
        # self.pid = PIDcal(Kp=np.array([1,1,2]),Ki=np.array([0.005,0.005,0.0]),Kd=np.array([0.4,0.4,0.05]))
        self.pid = PIDcal(Kp=np.array([2,2,2]),Ki=np.array([0.005,0.005,0.0]),Kd=np.array([0.4,0.4,0.05]))
        # self.pid = PIDcal(Kp=np.array([0,0,2]),Ki=np.array([0,0,0.0]),Kd=np.array([0,0,0.05]))
        
        #a_star 
        # self.pid = PIDcal(Kp=np.array([1.8,1,1]),Ki=np.array([0.04,0,0]),Kd=np.array([1.5,0,0.0]))
        self.radius = 150
        self.ds = 50
        
    def getRole(self):
        return self.role

    def getPosition(self) -> np.array:
        
        if self.team == "blue":
            data = C_vission_handler().robot_tBlue
        else:
            data = C_vission_handler().robot_tYellow
        # print(self.Id,"key")
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
        
        
    def nearPoint(self, point: np.array, threshold: int = 20) -> bool:
        # print("dis to", point, "is", self.__distanceToPoint(point))
        if self.__distanceToPoint(point) < threshold:
            return True
        return False
    
    def faceToPoint(self, point: np.array) -> bool:
        headingAngToBall = self.__angToPoint(point) - self.getOrientation()

        if headingAngToBall > np.pi:
            headingAngToBall -= 2 * np.pi

        elif headingAngToBall < -np.pi:
            headingAngToBall += 2 * np.pi

        if abs(headingAngToBall) >= 0.1:
            self.sendCommand(0, 0, 3 * headingAngToBall, False)
            return False
        else:
            return True
        
    def moveToPoint(self,point: np.array):
        dx, dy = point - self.getPosition() 
        o = self.getOrientation()
        dom = self.__angToPoint(point) - o
        S = np.array([dx,dy,dom])
        V = SToVCal(S,o)/5
        vx, vy, yaw = V
        self.sendCommand(vx,vy,yaw*10)
        
    #movement with pid
    def MotionMapping(self,point: np.array, angle = None) -> np.array:
        dx, dy = point - self.getPosition() 
        o = self.getOrientation()
        if angle == None:
            dom = self.__angToPoint(point) - o
        else:
            dom = angle*np.pi/180 - o

        if dom > np.pi:
            dom -= 2 * np.pi

        elif dom < -np.pi:
            dom += 2 * np.pi

        S = np.array([dx,dy,dom])
        
        V = SToVCal(S,o)/5
        vx, vy, yaw = V
        return np.array([vx,vy,yaw*20])

    def messureSpeed(self) -> np.array:
        x,y = self.getPosition()
        o = self.getOrientation()
        currentPos = np.array([x,y,o])
        currentTime = time.time()
        delS = currentPos - self.oldPosition
        dt = currentTime - self.oldTime
        v = delS/dt
        self.oldPosition = currentPos
        self.oldTime = currentTime
        return v, dt
    
    def MoveToPointWithPID(self,point: np.array,angle = None): 
        messure, dt = self.messureSpeed()
        setpoint = self.MotionMapping(point,angle)
        output = self.pid.compute(setpoint,messure,dt)
        vx, vy, vz = output
        vxs, vys, vzs = setpoint

        if self.nearPoint(point,self.ds):
            self.sendCommand(0.0,0.0,0.0)
        else:
            self.sendCommand(vx,vy,vz)
            
    def moveToPointWithA_star(self, point: np.array, angle=None):
        #NOTE: this algorithm need multitrade and cut of depth because when obs on in way or ray unable to hit robot stack nate so fix it 
        current_pos = self.getPosition()
        
         # NOTE: Using 'hasattr' here is fine, but initializing in __init__ is cleaner.
        if not hasattr(self, '_a_star_path'):
            self._a_star_path = []
        if not hasattr(self, '_target_point_cache'):
            self._target_point_cache = None

        Obs_list = []
        dataB = C_vission_handler().robot_tBlue
        dataY = C_vission_handler().robot_tYellow
        for i in range(len(dataB)):
            if (i == self.Id) and (self.team == "blue"):
                continue
            Obs_list.append(np.array([dataB[i].x, dataB[i].y]))
        for i in range(len(dataY)):
            if (i == self.Id) and (self.team == "yellow"):
                continue
            Obs_list.append(np.array([dataY[i].x, dataY[i].y]))

        target_changed = not np.array_equal(self._target_point_cache, point)
        
        if target_changed or not self._a_star_path:
            
            self._target_point_cache = point
            
            first_state = State(current_pos, point)
            
            node, _ = f_a_star(first_state, h, Obs_list, self.radius, point, self.ds)
            self._a_star_path = f_reconstruct_path(node)
            
        WAYPOINT_TOLERANCE = self.ds * 3

        if len(self._a_star_path) > 1:
            
            while len(self._a_star_path) > 1 and np.linalg.norm(current_pos - self._a_star_path[1].pos) < WAYPOINT_TOLERANCE:
                self._a_star_path.pop(0)
                
            if len(self._a_star_path) > 1:
                next_waypoint_position = self._a_star_path[1].pos
            else:
                next_waypoint_position = point 
        
        elif len(self._a_star_path) == 1:
            next_waypoint_position = point
            self._target_point_cache = None
            self._a_star_path = []
            
        else:
            next_waypoint_position = current_pos
            
        path_vector = next_waypoint_position - current_pos
        path_distance = np.linalg.norm(path_vector)

        if path_distance > 1e-6:
            rot = np.arctan2(path_vector[1], path_vector[0])
            
            checker_state = State(current_pos, next_waypoint_position)
            
            rayHit = checker_state.is_cut_obstacle_CORRECTED(
                Obs_list, 
                path_distance,
                self.radius, 
                rot
            )

            if rayHit:
                self._a_star_path = []
                next_waypoint_position = current_pos
        
        self.MoveToPointWithPID(next_waypoint_position, angle)
        
    # def moveToPointWithA_star(self,point: np.array,angle = None):
    #     first_state = State(self.getPosition(),point)
    #     robot_lange = list(range(C_vission_handler().num_of_robot ))
    #     robot_lange.remove(self.Id)
    #     Obs_list = []
    #     for i in robot_lange:
    #         Obs_list.append(np.array([self.data[i].x,self.data[i].y]))
    #     path = []
        
    #     node, _ = f_a_star(first_state,h,Obs_list,self.radius,point,self.ds)
    #     path = f_reconstruct_path(node)
        
    #     if len(path) > 2:
    #         position = path[1].pos
    #     else:
    #         position = point
    #     self.MoveToPointWithPID(position,angle)