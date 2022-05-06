from controller import Robot
import numpy as np
import math
# Robot is defined
robot = Robot()
# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

v_len = np.linalg.norm # Length of vector
pi = math.pi

class driver:
    def __init__(self, angle1, angle2, angle3):
        self.motor_1 = robot.getDevice('wheel1')
        self.motor_2 = robot.getDevice('wheel2')
        self.motor_3 = robot.getDevice('wheel3')
        self.angle_1 = angle1
        self.angle_2 = angle2
        self.angle_3 = angle3

    def driver(self, distance, angle):
        dir = self.polar_to_cartesian(angle, distance)
        m1 = self.polar_to_cartesian(self.angle_1, 1)
        m2 = self.polar_to_cartesian(self.angle_2, 1)
        m3 = self.polar_to_cartesian(self.angle_3, 1)
        v1 = self.vec_pos(m1, dir)
        v2 = self.vec_pos(m2, dir)
        v3 = self.vec_pos(m3, dir)
        self.motor_1.setVelocity(v_len(v1))
        self.motor_2.setVelocity(v_len(v2))
        self.motor_3.setVelocity(v_len(v3))

    def polar_to_cartesian(self, angle, length):
        return np.array([math.cos(angle) * length, math.sin(angle) * length])

    def vec_projection(self, a,b):
        # print('floats: {}{}'.format(a,b))
        return (np.dot(a,b)/v_len(b)**2)*b

    def vec_pos(self, a, b):
        return -1*self.vec_projection(a, b)+b

    def print_var(self):
        print(self.motor_1)
        print(self.angle_1)
        print(self.motor_2)
        print(self.angle_2)
        print(self.motor_3)
        print(self.angle_3)

#insedires
# v = -10
ds = robot.getDevice('position_sensor_wheel1')
ds.enable(timestep)
run = driver(0, 2*pi/3, 4*pi/3)

# Main loop:
# - perform simulation steps until Webots is stopping the controller


while robot.step(timestep) != -1:
   ###################### - Big wheels - ###########################
    run.motor_1.setPosition(0)
    run.motor_2.setPosition(0)
    run.motor_3.setPosition(0)
    #motor1.setVelocity(v)
    #motor2.setVelocity(-v)
    #motor3.setVelocity(0.0)
    #print("hi")
    run.driver(10,0*pi)
    run.print_var()

    ###################### - Small whells - #####################
    """ 10 is the max-speed of the small wheels"""
    """ Velorcity of the big wheels until  """
    #motor1.setVelocity(10) #small wheels
    #motor2.setVelocity(10) #small wheels
    #motor3.setVelocity(10) #small wheels


    # Deprecated functions written without use of numPy:

    # def dot(self, a,b):
    #     return a[0]*b[0]+a[1]*b[1]

    # def vec_len(self, a):
    #     return math.sqrt(a[0]**2+a[1]**2)

    # def vec_plus(self, a,b):
    #     return [a[0]+b[0],a[1]+b[1]]

    # def vec_scalar(self, a ,vet):
    #     return [a*vet[0], a*vet[1]]
