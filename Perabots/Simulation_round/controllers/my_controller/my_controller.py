from controller import Robot

def setup(robot):
    time_step =32
# Initialize motors
    left_motor = robot.getDevice('left motor')
    right_motor = robot.getDevice('right motor')
    
    left_motor.setPosition(float('inf'))  # Velocity control mode
    right_motor.setPosition(float('inf'))
    
    # Start stopped
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    # Initialize IR sensors (assumed 2 sensors: left and right)
    left_ir = robot.getDevice('left_ir')
    right_ir = robot.getDevice('right_ir')
    
    left_ir.enable(timestep)
    right_ir.enable(timestep)
    
    MAX_SPEED = 6.28
            # Obstacle ahead: stop or go backward
    left_motor.setVelocity(4)
    right_motor.setVelocity(4)

if __name__ =="__main__":
    robot = Robot()
    setup(robot)
   
    
