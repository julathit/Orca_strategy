import numpy as np
from heapq import heappush, heappop, heapify
from typing import List, Tuple, Optional, Any, Callable
import matplotlib.pyplot as plt

class state:
    def __init__(self,pos: np.array,goal_pos: np.array):
        self.pos = pos
        self.goal_pos = goal_pos
        delta_pos = goal_pos - self.pos
        current_heading = np.arctan2(delta_pos[1], delta_pos[0])
        self.rotation = current_heading
        self.fullRotate = 2*np.pi
        
        # config
        self.ang = np.pi/4
        self.step_size = 250
    
    def is_goal(self,goal: np.array,ds: float) -> bool:
        dis = np.linalg.norm(self.pos - goal)
        if dis < ds:
            return True
        return False
    
    def __hash__(self):
        return hash((round(self.pos[0], 4), round(self.pos[1], 4), round(self.rotation, 4)))
    
    def __repr__(self):
        return f"({self.pos[0]},{self.pos[1]})"
    
    def __eq__(self, obj: any):
        if not isinstance(obj, state):
            return False
        pos_equal = np.allclose(self.pos, obj.pos)
        rot_equal = np.isclose(self.rotation, obj.rotation) 
        return pos_equal and rot_equal
    
    def is_cut_obstacle_CORRECTED(self, obstacle: list, step: float, radius: float,rotation: float) -> bool:
        P_start = self.pos
        unitV = np.array([np.cos(rotation), np.sin(rotation)])
        V_path = step * unitV
        path_len_sq = step * step
        radius_sq = radius * radius 
        
        for obs_pos in obstacle:
            V_obs = obs_pos - P_start
            current_distance_sq = np.sum(V_obs**2)
            if current_distance_sq < radius_sq:
                continue

            if path_len_sq == 0:
                t = 0
            else:
                t = np.dot(V_obs, V_path) / path_len_sq
            t_clamped = np.clip(t, 0.0, 1.0)
            P_closest = P_start + t_clamped * V_path
            distance_sq = np.sum((obs_pos - P_closest)**2)
            if distance_sq < radius**2:
                return True 
        return False
    
    def suc(self, obstacle: list, radius: float):
        suc = []
        max_turn = np.pi/2
        num_increments = int(round(max_turn / self.ang + 1e-9)) 
        
        distance_to_goal = np.linalg.norm(self.goal_pos - self.pos)
        
        step_size = min(self.step_size, distance_to_goal)
        if step_size < 1e-6:
            return suc
        current_heading = self.rotation

        for k in range(-num_increments, num_increments + 1):
            # Calculate the new rotation
            new_rotation = (current_heading + k * self.ang) % self.fullRotate
            
            # 1. Calculate the new position
            dx = step_size * np.cos(new_rotation)
            dy = step_size * np.sin(new_rotation)
            displacement = np.array([dx, dy])
            
            new_pos = self.pos + displacement
            
            is_colliding = self.is_cut_obstacle_CORRECTED(
            obstacle, step_size, radius, rotation=new_rotation
            )
            if not is_colliding:
                successor_node = state(new_pos,self.goal_pos)
                
                suc.append((step_size, successor_node))
            

        return suc      