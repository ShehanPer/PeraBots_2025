import cv2
import numpy as np
from time import sleep
from config import *
from movemap import *
from matrix_map import *
from path_finder import save_optimal_path
from visualize import visualize_map


def stopwhenredSeeRed(frame):
    isdet=isredDetected(frame)
    if isdet:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        return True
    return False




isredSeen=0
counter=0
while robot.step(TIME_STEP) != -1:
    counter=counter+1
    if(counter>20):
        print("Over")
        break
    left_motor.setVelocity(3)
    right_motor.setVelocity(3)
    update_position()
    update_map()


while robot.step(TIME_STEP) != -1:

    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    # Convert Webots BGRA image to BGR
    img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    frame = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
    height, width = frame.shape[:2]
    middlestrip,bottomstrip,blacktestStrip,frame_with_line=get_middle_horizontal_strip(frame)

    blacktestStripArray=getcolorArray(blacktestStrip)
    middlecolArray=getcolorArray(middlestrip)
    bottomcolArray=getcolorArray(bottomstrip)

    midzeroIndexes=find_longest_zero_run(middlecolArray)
    bottomzeroIndexes=find_longest_zero_run(bottomcolArray)

    line_y_top = 0
    line_y_bottom = frame_with_line.shape[0] 
    cv2.line(frame_with_line, (midzeroIndexes[0], line_y_top), (midzeroIndexes[0], line_y_bottom), (0, 255, 0), 2)
    cv2.line(frame_with_line, (midzeroIndexes[1], line_y_top), (midzeroIndexes[1], line_y_bottom), (0, 255, 0), 2)

    cv2.line(frame_with_line, (bottomzeroIndexes[0], line_y_top), (bottomzeroIndexes[0], line_y_bottom), (255, 0, 0), 2)
    cv2.line(frame_with_line, (bottomzeroIndexes[1], line_y_top), (bottomzeroIndexes[1], line_y_bottom), (255, 0, 0), 2)
    cv2.rectangle(frame_with_line, (20, height//2), (280, height), (0, 0, 0), 2)
    cv2.rectangle(frame_with_line, ((width // 2)-25, (height//2)-10), ((width // 2)+25, (height//2) +70), (0, 0, 0), 2)

    midVal=int((midzeroIndexes[0]+midzeroIndexes[1])/2)
    midlength=midzeroIndexes[1]-midzeroIndexes[0]
    bottomval=int((bottomzeroIndexes[0]+bottomzeroIndexes[1])/2)
    bottomlength=bottomzeroIndexes[1]-bottomzeroIndexes[0]
    difval=midVal-bottomval
    
    if(not isredDetected(frame) and isredSeen==1):
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break
    elif(stopwhenredSeeRed(frame)):
        isredSeen=1
        decideDirection(150,width)
    elif(midzeroIndexes==(-1,-1) or midzeroIndexes==(-2,-2)):
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
    else:
        if((blacktestStripArray[20]==1 or blacktestStripArray[280]==1 )and bottomzeroIndexes not in [(-1, -1), (-2, -2)] ):
            decideDirection(bottomval-difval/2,width)
        elif(bottomzeroIndexes not in [(-1, -1), (-2, -2)] and midlength > 90 ):
            decideDirection(bottomval,width)
        else:
            decideDirection(midVal,width)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow("Middle Strip", frame_with_line)


    update_position()
    update_map()
 




cv2.destroyAllWindows()

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

#saving json. THis will be automated to run once robot coplete travelling in the maze once.
print(f"Sim time: {robot.getTime():.2f}s - Saving map...")
save_map_json(MAP, MAP_RES, f"robot_map.json")
sleep(1)
print("Saving optimal path to JSON file...")
save_optimal_path("robot_map.json","optimal_path_map.json","waypoints.json")
sleep(1)
print("Generating optimal path map image...")
visualize_map("optimal_path_map.json")

