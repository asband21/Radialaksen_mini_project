from controller import Robot
import math
robot = Robot() 
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

class driver:
    def __init__(self, moter1, angel1, moter2, angel2, moter3, angel3):
        self.moter_1 = moter1
        self.angel_1 = angel1
        self.moter_2 = moter2
        self.angel_2 = angel2
        self.moter_3 = moter3
        self.angel_3 = angel3
    def driver(self,speed, angel):
        print("tisse 2")
        ret = self.poler_til_kartien(angel, speed)
        print("tisse 1")
        m1 = poler_til_kartien(angel_1, 1)  
        m2 = poler_til_kartien(angel_2, 1)
        m3 = poler_til_kartien(angel_3, 1)
        v1 = vec_pas(m1,ret);
        v2 = vec_pas(m2,ret);
        v4 = vec_pas(m3,ret);
        self.moter_1.setVelocity(vec_len(v1))
        self.moter_2.setVelocity(vec_len(v2))
        self.moter_3.setVelocity(vec_len(v3))

    def dot(self, a,b):
        return [a[0]*b[0]+a[1]*b[1]]

    def poler_til_kartien(self, angel, length):
        return [math.cos(angel)*length,math.sin(angel)*length]

    def vec_skal(self, a ,vet):
        return [a*vet[0], a*vet[1]]

    def vec_plus(self, a,b):
        return [a[0]+b[0],a[1]+b[1]]

    def vec_prosekojon(self, a,b):
        return vec_skal((dot(a,b))/(dot(b,b)),b)

    def vec_pas(self, a,b):
        return vec_plus(b,vec_skal(-1,vec_prosekojon(a,b)))

    def vec_len(self, a):
        return math.sqrt(a[0]*a[0]+a[1]*a[1])

    def print_var(self):
        print(self.moter_1)
        print(self.angel_1)
        print(self.moter_2)
        print(self.angel_2)
        print(self.moter_3)
        print(self.angel_3)

#inseretes 
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
    run.driver(0, 0)

    ###################### - Small whells - #####################
    """ 10 is the max-speed of the small wheels""" 
    """ Velorcity of the big wheels until  """
    #motor1.setVelocity(10) #small wheels
    #motor2.setVelocity(10) #small wheels 
    #motor3.setVelocity(10) #small wheels 
