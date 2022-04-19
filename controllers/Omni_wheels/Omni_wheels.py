from controller import Robot
import math
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

class driver:
    def __init__(self, motor1, angle1, motor2, angle2, motor3, angle3):
        self.motor_1 = motor1
        self.angle_1 = angle1
        self.motor_2 = motor2
        self.angle_2 = angle2
        self.motor_3 = motor3
        self.angle_3 = angle3

    def driver(self, speed, angle):
        print("tisse 2")
        dir = self.polar_to_cartesian(angle, speed) # Asbjørn, speed er ordinært et 'distance' parameter
        print("tisse 1")
        m1 = self.polar_to_cartesian(self.angle_1, 1)
        m2 = self.polar_to_cartesian(self.angle_2, 1)
        m3 = self.polar_to_cartesian(self.angle_3, 1)
        v1 = self.vec_pos(m1, dir)
        v2 = self.vec_pos(m2, dir)
        v3 = self.vec_pos(m3, dir)
        self.motor_1.setVelocity(vec_len(v1))
        self.motor_2.setVelocity(vec_len(v2))
        self.motor_3.setVelocity(vec_len(v3))

    def polar_to_cartesian(self, angle, length):
        return ([math.cos(angle) * length,math.sin(angle) * length])

    def dot(self, a,b):
        return a[0]*b[0]+a[1]*b[1]

    def vec_scalar(self, a ,vet):
        return [a*vet[0], a*vet[1]]

    def vec_plus(self, a,b):
        return [a[0]+b[0],a[1]+b[1]]

    def vec_projection(self, a,b):
        print('floats: {}{}'.format(a,b))
        return self.vec_scalar((self.dot(a,b))/(self.vec_len(b)),b)

    def vec_pos(self, a, b):
        return self.vec_plus(b, self.vec_scalar(-1, self.vec_projection(a, b)))

    def vec_len(self, a):
        return math.sqrt(a[0]**2+a[1]**2)

    def print_var(self):
        print(self.motor_1)
        print(self.angle_1)
        print(self.motor_2)
        print(self.angle_2)
        print(self.motor_3)
        print(self.angle_3)

#insedires
v = -10
motor1 = robot.getDevice('wheel1')
motor2 = robot.getDevice('wheel2')
motor3 = robot.getDevice('wheel3')
ds = robot.getDevice('position_sensor_wheel1')
ds.enable(timestep)

run = driver(motor1 , 0, motor2 , 2*math.pi/3, motor3, 4*math.pi/3)

# Main loop:
# - perform simulation steps until Webots is stopping the controller


while robot.step(timestep) != -1:
   ###################### - Big wheels - ###########################
    motor1.setPosition(float('INF'))
    motor2.setPosition(float('INF'))
    motor3.setPosition(float('INF'))
    #motor1.setVelocity(v)
    #motor2.setVelocity(-v)
    #motor3.setVelocity(0.0)
    print("hej")
    run.driver(1,math.pi)

    ###################### - Small whells - #####################
    """ 10 is the max-speed of the small wheels"""
    """ Velorcity of the big wheels until  """
    #motor1.setVelocity(10) #small wheels
    #motor2.setVelocity(10) #small wheels
    #motor3.setVelocity(10) #small wheels
