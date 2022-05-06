from controller import Robot, DistanceSensor
from matplotlib.gridspec import GridSpec
from matplotlib import pyplot as plt
from numpy import pi
import pandas as pd
import numpy as np
import math


plt.style.use('fivethirtyeight')

robot = Robot()

# define variable for later incremental function
##increment = count()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#inseretes
v = -10
motor1 = robot.getDevice('wheel1')
motor2 = robot.getDevice('wheel2')
motor3 = robot.getDevice('wheel3')
# class dist_sensor(self, ID):
#     def __init__():
#         self.ID = ID
#     def

class distance_sensor:
    # distance sensor class is defined

    def __init__(self, ID):
        self.ID = ID
        super().__init__()
        self.name = 'distance sensor({})'.format(ID+1)
        self.sensor = robot.getDevice(self.name)
        self.activate()
        self.position = range(0 ,360, 60)[ID]

    def get_data(self):
        return self.sensor.getValue()

    def activate(self):
        self.sensor.enable(timestep)
        print("sensor {} activated!".format(self.ID+1))


dist_sensor= []
dist_sensor_objects = []
for i in range(6):
    dist_sensor_objects.append(distance_sensor(i))
    # dist_sensor.append(dist_sensor_objects[i].sensor())

#distance_sensor_array creation (objects)
# for i in range(len(dist_sensor_objects)):
    # dist_sensor.append(robot.getDevice(dist_sensor_objects[i]))
    # dist_sensor[i].enable(timestep)



ds = robot.getDevice('position_sensor_wheel1')
ds.enable(timestep)


#plot function:
def rosePlot(valueArray):
    data = pd.DataFrame({'value': valueArray,
                         'bearing': range(0, 360, 60),
                         'compass': ['S1', 'S12', 'S2', 'S23', 'S3', 'S31']})

    data.index = data['bearing'] * 2*pi / 360
    print(data.index)
    print(data['value'])
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


# Main loop:
# - perform simulation steps until Webots is stopping the controller

global_sensor_values = []
k = 0
while robot.step(timestep) != -1:
   ###################### - Big wheels - ###########################
    motor1.setPosition(float('INF'))
    motor2.setPosition(float('INF'))
    motor3.setPosition(float('INF'))
    motor1.setVelocity(0*v)
    motor2.setVelocity(0*v)
    motor3.setVelocity(0.4*v)
    current_sensor_values = []
    for i in range(len(dist_sensor_objects)):
        # print(dist_sensor_objects[i].sensor)
        dist_sensor_objects[i].get_data()
        current_sensor_values.append(dist_sensor_objects[i].get_data())
    global_sensor_values.append(current_sensor_values)
    print(current_sensor_values)

    if k%500 == 1:
        rosePlot(current_sensor_values)
        pass
    k += 1




    #print(global_sensor_values)
    # class DistanceSensor (Device):
    #     def enable(self, samplingPeriod):
    #     def disable(self):
    #     def getSamplingPeriod(self):
    #     def getValue(self):





    ###################### - Small whells - #####################
    #""" 10 is the max-speed of the small wheels"""
    #""" Velorcity of the big wheels until  """
    #motor1.setVelocity(10) #small wheels
    #motor2.setVelocity(10) #small wheels
    #motor3.setVelocity(10) #small wheels
