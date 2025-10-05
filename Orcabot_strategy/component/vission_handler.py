from krssg_ssl_msgs.msg import SSLDetectionFrame, SSLDetectionRobot, SSLDetectionBall

class vission_handler:
    def __init__(self):
        self.num_of_robot = 6
        self.robot_tBlue = {i: SSLDetectionRobot() for i in range(self.num_of_robot)}
        self.robot_tYellow = {i: SSLDetectionRobot() for i in range(self.num_of_robot)}

    def data_pack(self,d_frame: SSLDetectionFrame) -> None:
        for i in range(0,len(d_frame.robots_blue)):
            id_robot = d_frame.robots_blue[i].robot_id
            for index in range(self.num_of_robot):
                if id_robot == index:
                    self.robot_tBlue[index] = d_frame.robots_blue[i]