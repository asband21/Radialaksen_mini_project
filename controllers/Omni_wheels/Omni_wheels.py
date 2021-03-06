from controller import Robot, DistanceSensor
from matplotlib.gridspec import GridSpec
from matplotlib import pyplot as plt
from numpy import pi
import pandas as pd
import numpy as np
import math
# Plotting style
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
        # Dir is inputted distance and angle of direction of movement, desired. It is converted to cartesian, just as m1...m3
        dir = self.polar_to_cartesian(angle, distance)
        # Motor positional vectors convert to Cartesian unit vectors
        m1 = self.polar_to_cartesian(self.angle_1, 1)
        m2 = self.polar_to_cartesian(self.angle_2, 1)
        m3 = self.polar_to_cartesian(self.angle_3, 1)
        rot_matrix = np.array([[0,-1],[1,0]])  # Rotation array, used for determining
                                               # direction of vecocity vectors below in the following if statements
        v1 = self.vec_pos(dir, m1)
        v2 = self.vec_pos(dir, m2)
        v3 = self.vec_pos(dir, m3)

        print("dir: {} m1: {} m2: {} m3: {}v1: {} v2: {} v3: {} ".format(dir,m1,m2,m3,v1,v2,v3))

        # These if-staments are inner produkt of two vectors. m_n is motor postions and v_n is velocity of motor (n is number of particular motor).
        # It is determined if the two vectors are parallel and in direction relations are between them.
        # If the inner product between m_n and v_n vectors are -1, then the vectors are parrale but point in opposite directions.
        # When the inner product is 1, the two vectors point in the same direction.
        if  0 != v_len(v1):
            self.motor_1.setVelocity(v_len(v1)*(rot_matrix@m1@(v1[0]/v_len(v1))))
            # print("v1 flipspeed:"+str(rot_matrix@m2@(v1[0]/v_len(v1))))
        else :
            self.motor_1.setVelocity(0)
        if 0 != v_len(v2):
            self.motor_2.setVelocity(v_len(v2)*(rot_matrix@m2@(v2[0]/v_len(v2))))
            # print(rot_matrix@m2@(v2[0]/v_len(v2)))
        else :
            self.motor_2.setVelocity(0)
        if 0 != v_len(v3):
            self.motor_3.setVelocity(v_len(v3)*((rot_matrix@m3)@(v3[0]/v_len(v3))))
            # print((rot_matrix@m2)@(v3[0]/v_len(v3)))
        else :
            self.motor_3.setVelocity(0)

    def polar_to_cartesian(self, angle, length):
        return np.array([math.cos(angle) * length, math.sin(angle) * length])

    def vec_projection(self, a,b):
        return (np.dot(a,b)/v_len(b)**2)*b

    def vec_pos(self, a, b):
        return [a-self.vec_projection(a, b)]

    def print_var(self):
        print(self.motor_1)
        print(self.angle_1)
        print(self.motor_2)
        print(self.angle_2)
        print(self.motor_3)
        print(self.angle_3)

#plot object and method:
class data_display:
    def __init__(self, ):
        self.max_value = 1000   # integer parameters used for determining size of roseplot.
        self.r_offset = -100

    def rosePlot(self, value_array):
        data = pd.DataFrame({'value': value_array,
                             'bearing': range(360, 0, -60),
                             'compass': ['S1', 'S12', 'S2', 'S23', 'S3', 'S31']})
        data.index = data['bearing'] * 2 * pi / 360                         # Conversion to radians

        fig = plt.figure(figsize=(8, 3))                                    # Size of window plotting and initialisation
        gs = GridSpec(nrows=1, ncols=2, width_ratios=[1, 1])                # Plot two plots in a column of same width along each other horizontally.

        ax1 = fig.add_subplot(gs[0, 0]) # plot                              # Bar plot on left.
        ax1.bar(x=data['compass'], height=data['value'], width=1)           # XY defined
        ax1.set_ylim(0, self.max_value)
        ax1.spines['bottom'].set_position(('data', 0))  # Move the X axis up
        ax1.spines['top'].set_position(('axes', 0))
        ax1.spines['top'].set_linestyle(':')

        ax2 = fig.add_subplot(gs[0, 1], projection='polar')                 # ax2 is a polar plot (roseplot)
        ax2.set_theta_zero_location('S')                                    # First sensor is in South direction of polar plot
        ax2.set_theta_direction(-1)

        ax2.set_rlim(0, self.max_value)
        ax2.set_rorigin(self.r_offset)

        alpha = self.max_value - 2*self.r_offset   # Polar plot is made 200 wider, to account for 0 starting at radius 100, and leaving 100 extra space.
        v_offset = self.r_offset**2 / alpha
        forward = lambda value: ((value + v_offset) * alpha)**0.5 + self.r_offset
        reverse = lambda radius: (radius - self.r_offset) ** 2 / alpha - v_offset

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
        self.position = range(0, 360, 60)[ID] * 2 * pi / 360
        self.activate()

    def get_data(self):
        return self.sensor.getValue()

    def activate(self):
        self.sensor.enable(timestep)
        print("sensor {} activated!".format(self.ID+1))


dist_sensor= []
dist_sensor_objects = []
for i in range(6):
    dist_sensor_objects.append(distance_sensor(i))
    # print(dist_sensor_objects[i].position)
ds = robot.getDevice('position_sensor_wheel1')
ds.enable(timestep)
run = driver(0, 2*pi/3, 4*pi/3)
data_object = data_display()

# Main loop:
# - perform simulation steps until Webots is stopping the controller

global_sensor_values = []
# two arbitrarily declared variables used for one time operations,
# such as initial movement before spotting targetbot.
k = 0
h = 0
while robot.step(timestep) != -1:
   ###################### - Big wheels - ###########################
    run.motor_1.setPosition(float('inf'))  # Motors set to always be adjustable in speed
    run.motor_2.setPosition(float('inf'))
    run.motor_3.setPosition(float('inf'))


    # run.print_var()

    current_sensor_values = []
    for i in range(len(dist_sensor_objects)):
        # print(dist_sensor_objects[i].sensor)
        dist_sensor_objects[i].get_data()
        current_sensor_values.append(dist_sensor_objects[i].get_data())
    # mAngle = angle of max value distance sensor (Max Angle)
    mAngle = dist_sensor_objects[current_sensor_values.index(max(current_sensor_values))].position
    sum_angle = [0,0]
    for i in range(6):
        sum_angle = sum_angle + run.polar_to_cartesian(dist_sensor_objects[i].position,dist_sensor_objects[i].get_data())
    print("sum_angle: {} , \nlen: {}".format(sum_angle,v_len(sum_angle)))


    if v_len(sum_angle) >= 100:
        run.driver(10,math.atan2(sum_angle[1], sum_angle[0]))
        # print("mAngle:"+str(math.atan2(sum_angle[1], sum_angle[0])))
    else:
        if h == 0:
            run.driver(10,2*pi/3)
            h =+ 1
        else:
            pass
    global_sensor_values.append(current_sensor_values)

    if k%500 == 1:

        data_object.rosePlot(current_sensor_values) #When out-comment the plots are disabled.
        pass
    k += 1
