from controller import Robot
robot = Robot() 

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#inseretes 
motor1 = robot.getDevice('wheel1') 
motor2 = robot.getDevice('wheel2') 
motor3 = robot.getDevice('wheel3')
ds = robot.getDevice('position_sensor_wheel1')
ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
   ###################### - Big wheels - ###########################
    motor1.setPosition(100.0) 
    motor2.setPosition(100.0)
    motor3.setPosition(100.0)   
    
    ###################### - Small whells - #####################
    """ 10 is the max-speed of the small wheels""" 
    """ Velorcity of the big wheels until  """
    #motor1.setVelocity(10) #small wheels
    #motor2.setVelocity(10) #small wheels 
    #motor3.setVelocity(10) #small wheels 