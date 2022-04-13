from controller import Robot, DistanceSensor
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#inseretes
v = -10
motor1 = robot.getDevice('wheel1')
motor2 = robot.getDevice('wheel2')
motor3 = robot.getDevice('wheel3')
ps = []
psNames = ['distance sensor(1)','distance sensor(2)','distance sensor(3)','distance sensor(4)','distance sensor(5)']#,'distance sensor(6)']  #distance_sensor_array
for i in range(len(psNames)):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

ds = robot.getDevice('position_sensor_wheel1')
ds.enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller


while robot.step(timestep) != -1:
   ###################### - Big wheels - ###########################
    motor1.setPosition(float('INF'))
    motor2.setPosition(float('INF'))
    motor3.setPosition(float('INF'))
    motor1.setVelocity(0*v)
    motor2.setVelocity(0*v)
    motor3.setVelocity(0.0*v)
    psValues = []
    for i in range(len(ps)):
        psValues.append(ps[i].getValue())

    # class DistanceSensor (Device):
    #     def enable(self, samplingPeriod):
    #     def disable(self):
    #     def getSamplingPeriod(self):
    #     def getValue(self):




    ###################### - Small whells - #####################
    """ 10 is the max-speed of the small wheels"""
    """ Velorcity of the big wheels until  """
    #motor1.setVelocity(10) #small wheels
    #motor2.setVelocity(10) #small wheels
    #motor3.setVelocity(10) #small wheels
