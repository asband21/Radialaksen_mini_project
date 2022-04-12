from controller import Robot, DistanceSensor
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#inseretes
v = -10
motor1 = robot.getDevice('wheel1')
motor2 = robot.getDevice('wheel2')
motor3 = robot.getDevice('wheel3')
# dist1  = robot.getDevice('
ds = robot.getDevice('position_sensor_wheel1')
ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller


while robot.step(timestep) != -1:
   ###################### - Big wheels - ###########################
    motor1.setPosition(float('INF'))
    motor2.setPosition(float('INF'))
    motor3.setPosition(float('INF'))
    motor1.setVelocity(0.80*v)
    motor2.setVelocity(-0.80*v)
    motor3.setVelocity(0.30*v)




    ###################### - Small whells - #####################
    """ 10 is the max-speed of the small wheels"""
    """ Velorcity of the big wheels until  """
    #motor1.setVelocity(10) #small wheels
    #motor2.setVelocity(10) #small wheels
    #motor3.setVelocity(10) #small wheels
