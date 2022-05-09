from controller import Robot, DistanceSensor
from matplotlib.gridspec import GridSpec
from matplotlib import pyplot as plt
from numpy import pi
import pandas as pd
import numpy as np
import math




plt.style.use('fivethirtyeight')

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

#plot object and method:
class dataDisplay:
    def __init__(self, ):
        self.max_value = 1000
        self.r_offset = -100

    def rosePlot(self, value_array):
        data = pd.DataFrame({'value': value_array,
                             'bearing': range(360, 0, -60),
                             'compass': ['S1', 'S12', 'S2', 'S23', 'S3', 'S31']})
        data.index = data['bearing'] * 2 * pi / 360
        # print(data.index)
        # print(data['value'])
        fig = plt.figure(figsize=(8, 3))
        gs = GridSpec(nrows=1, ncols=2, width_ratios=[1, 1])

        ax1 = fig.add_subplot(gs[0, 0])
        ax1.bar(x=data['compass'], height=data['value'], width=1)
        ax1.set_ylim(0, 1000)
        ax1.spines['bottom'].set_position(('data', 0))  # Move the X axis up
        ax1.spines['top'].set_position(('axes', 0))
        ax1.spines['top'].set_linestyle(':')

        ax2 = fig.add_subplot(gs[0, 1], projection='polar')
        ax2.set_theta_zero_location('S')
        ax2.set_theta_direction(-1)

        max_value = 1000
        r_offset = -100

        ax2.set_rlim(0, max_value)
        ax2.set_rorigin(r_offset)

        r2 = max_value - r_offset
        alpha = r2 - r_offset
        v_offset = r_offset**2 / alpha
        forward = lambda value: ((value + v_offset) * alpha)**0.5 + r_offset
        reverse = lambda radius: (radius - r_offset) ** 2 / alpha - v_offset

        ax2.set_yscale('function', functions=(
            lambda value: np.where(value >= 0, forward(value), value),
            lambda radius: np.where(radius > 0, reverse(radius), radius)))

        ax2.set_xticks(data.index, labels=data['compass'])
        ax2.set_xticklabels(data.compass)
        ax2.set_rgrids([0, 250, 500, 750, 1000])
        ax2.bar(x=data.index,align='center', height=data['value'], width=pi/6)

        plt.show()




class distance_sensor:
    # distance sensor class is defined

    def __init__(self, ID):
        self.ID = ID
        super().__init__()
        self.name = 'distance sensor({})'.format(ID+1)
        self.sensor = robot.getDevice(self.name)
        self.activate()
        self.position = range(360, 0, -60)[ID] * 2 * pi / 360

    def get_data(self):
        return self.sensor.getValue()

    def activate(self):
        self.sensor.enable(timestep)
        print("sensor {} activated!".format(self.ID+1))

dist_sensor= []
dist_sensor_objects = []
for i in range(6):
    dist_sensor_objects.append(distance_sensor(i))
ds = robot.getDevice('position_sensor_wheel1')
ds.enable(timestep)
run = driver(0, 2*pi/3, 4*pi/3)
data_object = dataDisplay()

# Main loop:
# - perform simulation steps until Webots is stopping the controller

global_sensor_values = []
k = 0
h = 0
while robot.step(timestep) != -1:
   ###################### - Big wheels - ###########################
    run.motor_1.setPosition(0)
    run.motor_2.setPosition(0)
    run.motor_3.setPosition(0)



    # run.print_var()

    current_sensor_values = []
    for i in range(len(dist_sensor_objects)):
        # print(dist_sensor_objects[i].sensor)
        dist_sensor_objects[i].get_data()
        current_sensor_values.append(dist_sensor_objects[i].get_data())
    # mAngle = angle of max value distance sensor (Max Angle)
    mAngle = dist_sensor_objects[current_sensor_values.index(max(current_sensor_values))].position

    if dist_sensor_objects[current_sensor_values.index(max(current_sensor_values))].get_data() >= 200:
        run.driver(9.5,pi+mAngle)
    else:
        if h == 0:
            run.driver(9.5,0*pi)
            h =+ 1
        else:
            pass
    global_sensor_values.append(current_sensor_values)
    #print(current_sensor_values)

    if k%500 == 1:

        data_object.rosePlot(current_sensor_values)
        pass
    k += 1
    ###################### - Small whells - #####################
    """ 10 is the max-speed of the small wheels"""
    # Deprecated functions written without use of numPy:

    # def dot(self, a,b):
    #     return a[0]*b[0]+a[1]*b[1]

    # def vec_len(self, a):
    #     return math.sqrt(a[0]**2+a[1]**2)

    # def vec_plus(self, a,b):
    #     return [a[0]+b[0],a[1]+b[1]]

    # def vec_scalar(self, a ,vet):
    #     return [a*vet[0], a*vet[1]]
