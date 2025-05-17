from controller import Robot
import cv2


TIME_STEP = 32
MAX_SPEED = 6.28  # Adjust if your robot's max speed is different

robot = Robot()

left_motor = robot.getDevice('left motor')
right_motor = robot.getDevice('right motor')

#get the camera device
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)


left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(MAX_SPEED)
right_motor.setVelocity(MAX_SPEED)

while robot.step(TIME_STEP) != -1:
   
    ret, frame = camera.getImage()
    if ret:
        # Convert the image to a format suitable for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Display the image using OpenCV
        cv2.imshow('Camera Feed', frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()