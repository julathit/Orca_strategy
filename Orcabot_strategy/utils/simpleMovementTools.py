import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, setpoint: np.array, measurement: np.array, dt: float) -> np.array:
        error = setpoint
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output
    

# S is vector that tell distace to go and oretation to make <dx,dy,dom>
# V is vector speed each compo is <vx,vy,om> speed of robot
# <fox,fxy> is front vector <sox,soy> is side vector
# M is unit orthogonal matrix of [[fox,sox,0],
#                                 [foy,s0y,0], 
#                                 [0,  0  ,1]]
# Remid yoy if M is unit orthogonal MT = M^-1
#o is ang of robot


def SToV(S: np.array,o: np.array) -> np.array:
    fox,foy = np.cos(o),np.sin(o)
    sox, soy = (-foy), (fox)
    M = np.array([[fox,sox,0],
                 [foy,soy,0],
                 [0  ,0  ,1]])
    Mmal = lambda x, y: np.matmul(x, y)
    MT = np.transpose(M)
    V = Mmal(MT,S)
    return V