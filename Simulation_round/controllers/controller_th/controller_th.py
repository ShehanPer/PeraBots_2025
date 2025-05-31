import cv2
import numpy as np
from config import *
from movemap import *
from matrix_map import *
from path_finder import save_optimal_path




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

    midVal=int((midzeroIndexes[0]+midzeroIndexes[1])/2)
    midlength=midzeroIndexes[1]-midzeroIndexes[0]
    bottomval=int((bottomzeroIndexes[0]+bottomzeroIndexes[1])/2)
    bottomlength=bottomzeroIndexes[1]-bottomzeroIndexes[0]
    difval=midVal-bottomval
    print(f"MidVal: {midVal}, BottomVal: {bottomval}, MidLength: {midlength}, BottomLength: {bottomlength}, DifVal: {difval}")

    if(midzeroIndexes==(-1,-1) or midzeroIndexes==(-2,-2)):
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
    update_map_with_all_sensors()
    update_map()
    current_simulation_time = robot.getTime()
    if current_simulation_time % 5 == 0:
        print(f"Sim time: {robot.getTime():.2f}s - Saving map...")
        # Ensure MAP_RES (from matrix_map.py) is accessible or pass it
        save_map_json(MAP, MAP_RES, f"robot_map_final.json")
cv2.destroyAllWindows()


