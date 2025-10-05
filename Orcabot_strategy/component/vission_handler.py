from krssg_ssl_msgs.msg import SSLDetectionFrame, SSLDetectionRobot, SSLDetectionBall

class vission_handler:
    _instance = None 
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(vission_handler, cls).__new__(cls)
            cls._instance.initialized = False 
        return cls._instance
    
    def __init__(self):
        # Only run initialization code once
        if self.initialized:
            return

        self.num_of_robot = 6
        self.robot_tBlue = {i: SSLDetectionRobot() for i in range(self.num_of_robot)}
        self.robot_tYellow = {i: SSLDetectionRobot() for i in range(self.num_of_robot)}
        self.initialized = True # Mark as initialized

    def data_pack(self,d_frame: SSLDetectionFrame) -> None:
                    
        for robot in d_frame.robots_blue:
            id_robot = robot.robot_id
            if 0 <= id_robot < self.num_of_robot:
                self.robot_tBlue[id_robot] = robot

        # Add yellow robots for completeness
        for robot in d_frame.robots_yellow:
            id_robot = robot.robot_id
            if 0 <= id_robot < self.num_of_robot:
                self.robot_tYellow[id_robot] = robot