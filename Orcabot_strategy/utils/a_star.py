import numpy as np
from heapq import heappush, heappop, heapify
from typing import List, Tuple, Optional, Any, Callable
import matplotlib.pyplot as plt

from Orcabot_strategy.utils.state import state

class Node:
    def __init__(self, current: state, parent: state, path_cost: int, depth: int) -> None:
        self.current = current
        self.parent = parent
        self.path_cost = path_cost
        self.depth = depth
    def __lt__(self, other: state) -> bool:
        return self.path_cost < other.path_cost
    
def h_euclidean(current_pos: np.array, goal_pos: np.array) -> float:
    return np.linalg.norm(current_pos - goal_pos)

def reconstruct_path(node: Node) -> List[state]:
    path = []
    current = node
    while current:
        path.append(current.current)
        current = current.parent
    return path[::-1] 

def initial_node(node: Node) -> np.array:
    current_node = node
    while current_node:
        current_node = current_node.parent
    return current_node.parent.pos

def a_star(init: 'state', h: Callable[[np.array, np.array], float], obstacle: list, radius: float, goal: np.array, ds: float) -> Tuple['Node', int]:
    init_node = Node(init, None, 0, 0)
    
    g_scores = {init: 0.0}
    
    f_value = 0.0 + h(init.pos, goal)
    
    frontier = [(f_value, init_node)]
    heapify(frontier)
    
    n_visits = 0
    
    while frontier:
        
        n_visits += 1
        
        f_value, node = heappop(frontier)
        current_state = node.current
        
        if node.path_cost > g_scores.get(current_state, float('inf')):
             continue

        if current_state.is_goal(goal, ds):
            return node, n_visits
        
        for step_cost, child_state in current_state.suc(obstacle,radius):
            
            new_g_score = node.path_cost + step_cost
            if new_g_score < g_scores.get(child_state, float('inf')):
                
                g_scores[child_state] = new_g_score
                
                child_node = Node(child_state, node, new_g_score, node.depth + 1)
                
                new_score = new_g_score + h(child_state.pos, goal)
                # print(new_score,child_node.current,"next")
                heappush(frontier, (new_score, child_node))
                
    return None, n_visits