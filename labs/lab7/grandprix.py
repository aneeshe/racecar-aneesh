# Added the following 2 lines to get logging imports
import os 
import logging.config

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

# Added the following 2 lines to set up a log file
dir_path = os.path.dirname(os.path.realpath(__file__))
logfile = dir_path + "all9.log"
logging.basicConfig(filename=logfile,
                            filemode='a',
                            format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                            datefmt='%H:%M:%S',
                            level=logging.DEBUG)

colors = [    # defining the Hue Saturation Value Ranges for all of the colors
    ((85, 200, 200), (120, 255, 255), "blue"), 
    ((40, 50, 50), (80, 255, 255), "green"), 
    ((170, 50, 50), (10, 255, 255), "red"),
    ((125, 245, 215), (145, 255, 255), "purple_l"),
    ((5, 245, 215), (25, 255, 255), "orange_l")
]

blue_l = ((85, 200, 200), (120, 255, 255), "blue_l") 
red_l =  ((170, 50, 50), (10, 255, 255), "red_l")
green_l =  ((40, 50, 50), (80, 255, 255), "green_l"), 
purple_l = ((125, 245, 215), (145, 255, 255), "purple_l")
orange_l = ((5, 245, 215), (25, 255, 255), "orange_l")  
class slalom(IntEnum): # creating a state machine for the section that the racecar has to slalom around cones
    search  = 0
    approach_red = 1
    approach_blue = 2
    turn_red = 3
    turn_blue = 4
    stop = 5
curr_state_slalom: slalom = slalom.search
    
rc = racecar_core.create_racecar()
logging.info("Created racecar instance as rc")
speed = 0 
angle = 0
counter = 0
BLUE_S = ((100, 150, 150), (130, 255, 255))  # The HSV range for the color blue
RED_S  = ((165, 0, 0),(179, 255, 255)) 
MIN_CONTOUR_AREA_S = 800  # determining the minimum area for something to be recognised and detected, otherwise there will be too much noise
DIST_S = 80
# Cone recovery
RECOVER_BLUE_S = False
RECOVER_RED_S = False
#Speed constants
APPROACH_SPEED_S = 1
TURN_SPEED_S = 0.65
#Angle constants
RECOVER_ANGLE_S = 0.95
TURN_ANGLE_S =  1
Slalom_counter = 0
between_counter = 0
line_counter = 0
idle_counter = 0

def get_contour(HSV, MIN_CONTOUR_AREA_S = 30): # for a given HSV(find the contour area, if its size is greater than 30)
    image = rc.camera.get_color_image()# function to retrieve color image
    if image is None:
        return None
    else:
        contours = rc_utils.find_contours(image, HSV[0], HSV[1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA_S)
        return contour
class State (IntEnum): # creating a state machine for every task the racecar has to complete. It transitions between tasks with AR Markers
    idle = 0
    between_lines= 1
    wall_follow_right = 2
    line_follow_blue = 3
    line_follow_red = 4
    line_follow_green = 5
    slalom = 6
curr_state: State = State.idle
# Add any global variables here

########################################################################################
# Functions
########################################################################################
def line_follower(color):# function for how the racecar will folow a certain color line
    logging.info("Inside line_follower function using the color camera for color detection and following color priority")
    global line_counter
    global curr_state
    print("this is line counter", line_counter)
    MIN_CONTOUR_AREA = 150
    CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
    BLUE =[colors[0][0],colors[0][1]] # The HSV range for the color blue
    RED  = [colors[2][0], colors[2][1]]
    GREEN = [colors[1][0], colors[1][1]]
    
    contour_center = None
    contour_area = 0
    
    image = rc.camera.get_color_image()
    priority_mode = 0
    if color == "blue": 
        priority_mode = 3
    elif color == "red":
        priority_mode = 1
    else:
        priority_mode = 2

    order = []
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)

        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # Find all of the color contours
        red_contours = rc_utils.find_contours(image, RED[0], RED[1])
        blue_contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
        green_contours = rc_utils.find_contours(image, GREEN[0], GREEN[1])
        purple_contours = rc_utils.find_contours(image,(125, 245, 215), (145, 255, 255))
        orange_contours = rc_utils.find_contours(image, (5, 245, 215), (25, 255, 255))
        print("this is the lenght of orange contours", len(orange_contours))
    if priority_mode == 1:
        order = [red_contours,green_contours,blue_contours,orange_contours,purple_contours,]
    if priority_mode == 2:
        order = [green_contours,red_contours,blue_contours,orange_contours,purple_contours] 
        # print ("Priority: green > red > blue")
    if priority_mode == 3:
        order = [blue_contours,green_contours, red_contours,orange_contours] 
        # print ("Priority: green > blue > red")
    for contours_color in order:
        if contours_color is not None:
            contour = rc_utils.get_largest_contour(contours_color, MIN_CONTOUR_AREA)
            if contour is None: continue
            break

    if contour is not None:
        # Calculate contour information
        contour_center = rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)

        # Draw contour onto the image
        rc_utils.draw_contour(image, contour)
        rc_utils.draw_circle(image, contour_center)

    else:
        contour_center = None
        contour_area = 0
        return 0

    # Display the image to the screen
    # rc.display.show_color_image(image)
    print("this is contourcenter",contour_center[1])
    angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1], 
                                                    rc.camera.get_width(), 
                                                    rc.camera.get_width() / 2 - 1, 1, 0) * 2, -1, 1)
    if line_counter > 20:
        curr_state = State.idle
    line_counter += rc.get_delta_time()
    return angle

def between_lines(color):# function for how racecar will go between lines
    logging.info("Inside between_lines function using the color camera for color detection and tracking")
    global between_counter
    print("this is between counter", between_counter)
    MIN_CONTOUR_AREA = 150
    CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
    BLUE =[colors[0][0],colors[0][1]] # The HSV range for the color blue
    RED  = [colors[2][0], colors[2][1]]
    GREEN = [colors[1][0], colors[1][1]]
    
    contour_center = None
    contour_area = 0
    
    image = rc.camera.get_color_image()
    priority_mode = 0
    if color == "blue": 
        priority_mode = 3
    elif color == "red":
        priority_mode = 1
    else:
        priority_mode = 2

    order = []
    if image is None:
        contour_center = None
        contour_area = 0
    else:
    

        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # Find all of the color contours
        red_contours = rc_utils.find_contours(image, RED[0], RED[1])
        blue_contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
        green_contours = rc_utils.find_contours(image, GREEN[0], GREEN[1])
        purple_contours = rc_utils.find_contours(image,(125, 245, 215), (145, 255, 255))
        orange_contours = rc_utils.find_contours(image, (5, 245, 215), (25, 255, 255))
        print("this is the length of orange contours", len(orange_contours))
        print("this is the length of purple contours", len(purple_contours))
    if priority_mode == 1:
        order = [red_contours,green_contours,blue_contours,orange_contours,purple_contours,]
    if priority_mode == 2:
        order = [green_contours,red_contours,blue_contours,orange_contours,purple_contours] 
        # print ("Priority: green > red > blue")
    if priority_mode == 3:
        order = [blue_contours,green_contours, red_contours,orange_contours] 
        # print ("Priority: green > blue > red")
    for contours_color in order:
        if contours_color is not None:
            contour = rc_utils.get_largest_contour(contours_color, MIN_CONTOUR_AREA)
            if contour is None: continue
            break

    if contour is not None:
        # Calculate contour information
        contour_center = rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)

        # Draw contour onto the image
        rc_utils.draw_contour(image, contour)
        rc_utils.draw_circle(image, contour_center)

    else:
        contour_center = None
        contour_area = 0
        return 0

    # Display the image to the screen
    rc.display.show_color_image(image)
    print("this is contourcenter",contour_center[1])
    if between_counter < 5.5:
        angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1]-216, 
                                                    rc.camera.get_width(), 
                                                    rc.camera.get_width() / 2 - 1, 1, 0) * 2, -1, 1)
    else:
         angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1]+63, 
                                                    rc.camera.get_width(), 
                                                    rc.camera.get_width() / 2 - 1, 1, 0) * 2, -1, 1)
    #elif between_counter > 4 and between_counter < 22:
       # angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1]+50, 
        #                                            rc.camera.get_width(), 
         #                                           rc.camera.get_width() / 2 - 1, 1, 0) * 2, -1, 1)
    #elif between_counter > 21.5 and between_counter < 23.7:
     #   angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1]+250, 
      #                                              rc.camera.get_width(), 
       #                                             rc.camera.get_width() / 2 - 1, 1, 0) * 2, -1, 1)
   # else:
    #    angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1]-50, 
                 #                                  rc.camera.get_width(), 
                 #                                   rc.camera.get_width() / 2 - 1, 1, 0) * 2, -1, 1)
    between_counter += rc.get_delta_time()
    return angle
def start():
    logging.info("Inside start function")
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 5 - AR Markers")


def update():# function that is run 60 times per second and based on the state follows a course of action.
    global angle,speed, RECOVER_BLUE_S, RECOVER_RED_S
    global curr_state, curr_state_slalom, counter
    global DIST_S, RED_S, BLUE_S, MIN_CONTOUR_AREA_S
    blue_l = ((85, 200, 200), (120, 255, 255), "blue_l") 
    red_l =  ((170, 50, 50), (10, 255, 255), "red_l")
    green_l =  ((40, 50, 50), (80, 255, 255), "green_l") 
    purple_l = ((125, 245, 215), (145, 255, 255), "purple_l")
    orange_l = ((5, 245, 215), (25, 255, 255), "orange_l")
    global Slalom_counter, between_counter, idle_counter
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    
    for marker in markers: # determining what the Ar Marker says and switching to the corresponding state
        print("i am printing this ", marker.get_id())
        if marker.get_id() == 0 and Slalom_counter > 26.5:
            print("you made it here")
            marker.detect_colors(color_image, [blue_l, red_l, green_l])
            print(marker.get_color())
            if marker.get_color() == "red_l": # The color of the AR marker is going to be the line that we follow
                logging.info("Detecting AR Marker which tell us to switch to line following")
                logging.info("Color on Perimeter of AR Marker tells us the priority of the Lines(Red Now)")
                print("you made it here red")
                curr_state = State.line_follow_red
            elif marker.get_color() == "green_l":
                logging.info("Detecting AR Marker which tell us to switch to line following")
                logging.info("Color on Perimeter of AR Marker tells us the priority of the Lines(Green Now)")
                curr_state = State.line_follow_green
            elif marker.get_color() == "blue_l":
                logging.info("Detecting AR Marker which tell us to switch to line following")
                logging.info("Color on Perimeter of AR Marker tells us the priority of the Lines(Blue Now)")
                curr_state = State.line_follow_blue
        elif marker.get_id() == 1 and idle_counter > 24.5:
            logging.info("Detecting AR Marker which tell us to switch to going between Lines")
            curr_state = State.between_lines
        elif marker.get_id() == 3 and between_counter > 29.5:
            logging.info("Detecting AR Marker which tell us to switch to avoid Natural colored obstacles")
            curr_state = State.idle
        elif marker.get_id() == 199:
            print(marker.get_orientation())
            if marker.get_orientation().value == 1:
                curr_state = State.idle
            else:
                curr_state = State.idle
        
        elif marker.get_id() == 2 and idle_counter > 56.3:
            logging.info("Detecting AR Marker which tell us to switch to Slalom")
            print("beginning Slalom")
            curr_state = State.slalom
    
    scan = rc.lidar.get_samples() 
    if curr_state == State.between_lines:
        angle = between_lines("purple")# NOW we call the line follow methods that we already made
    elif curr_state == State.line_follow_green: 
        angle = line_follower("green")
    elif curr_state == State.line_follow_blue: 
        angle = line_follower("blue")
    elif curr_state == State.line_follow_red:
        angle = line_follower("red")
    elif curr_state == State.slalom: # I didn't make a slalom method so I run this in the update function, time complexity is still the same
        logging.info("Inside Slalom funciton, by using color camera for cone recognition, and depth camera for distance")
        print("This is Slalom_counter",Slalom_counter) 
        print("camera height",rc.camera.get_height())
        image = rc.camera.get_color_image()
        CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        depth_image_original = (rc.camera.get_depth_image() - 0.01) % 10000
        depth_image_original= cv.GaussianBlur(depth_image_original,(3,3),0)
    
        red_contour = get_contour(RED_S, MIN_CONTOUR_AREA_S)
        blue_contour = get_contour(BLUE_S, MIN_CONTOUR_AREA_S)
    
        red_center = rc_utils.get_contour_center(red_contour) if red_contour is not None else None
        blue_center = rc_utils.get_contour_center(blue_contour) if blue_contour is not None else None
    
        red_depth = depth_image_original[red_center[0]][red_center[1]] if red_contour is not None else 0.0
        blue_depth = depth_image_original[blue_center[0]][blue_center[1]] if blue_contour is not None else 0.0
    
        # we want to identify the closest object
    
        if curr_state_slalom == slalom.search:
            speed = 1
            angle = -.05
            if RECOVER_RED_S: 
                angle = -RECOVER_ANGLE_S
            if RECOVER_BLUE_S:
                angle = RECOVER_ANGLE_S
            if red_depth < blue_depth and red_depth!=0:
                curr_state_slalom = slalom.approach_red
            if blue_depth < red_depth and blue_depth!=0:
                curr_state_slalom  = slalom.approach_blue
            elif red_depth != 0:
                curr_state_slalom = slalom.approach_red
            elif blue_depth !=0:
                curr_state_slalom= slalom.approach_blue
            else:
                speed = 1

        if curr_state_slalom == slalom.approach_red:
            if RECOVER_BLUE_S: RECOVER_BLUE_S = False
            if red_depth == 0.0: 
                curr_state_slalom = slalom.search
            elif red_depth < DIST_S: 
                curr_state_slalom = slalom.turn_red  
            else:
              #  rc_utils.draw_circle(image,red_center)
                angle = rc_utils.remap_range(red_center[1], 0, rc.camera.get_width(), -1,1, True) 
    
        if curr_state_slalom == slalom.approach_blue:
            if RECOVER_RED_S: RECOVER_RED_S = False
            if blue_depth == 0.0: 
                curr_state_slalom = slalom.search
            elif blue_depth < DIST_S: 
                curr_state_slalom = slalom.turn_blue
            else:
              #  rc_utils.draw_circle(image, blue_center)
                angle = rc_utils.remap_range(blue_center[1], 0, rc.camera.get_width(), -1,1,True) 
    
        if curr_state_slalom == slalom.turn_red:
            counter += rc.get_delta_time()
            if counter < 0.85:
                angle = TURN_ANGLE_S
            elif counter <1:
                angle = 0
            else:
                counter = 0
                RECOVER_RED_S = True
                curr_state_slalom = slalom.search

        if curr_state_slalom == slalom.turn_blue:
            counter += rc.get_delta_time()
            if counter < 0.85:
                angle = -TURN_ANGLE_S
            elif counter < 1:
                angle = 0
            else:
                counter = 0
                RECOVER_BLUE_S = True
                curr_state_slalom = slalom.search
    
        if curr_state_slalom == (slalom.approach_blue or slalom.approach_red or slalom.search):
            speed = APPROACH_SPEED_S
        else:
            speed = TURN_SPEED_S
        
        rc.drive.set_speed_angle(speed, angle)
        rc.display.show_color_image(image)

        #######################################
        ###############Debug###################
        #######################################
        if Slalom_counter > 22.8 and Slalom_counter < 25.1:
            angle = -1
        elif Slalom_counter >25.1 and Slalom_counter <26:
            angle = 1
        #print(f"S:{curr_state_slalom} V{speed:.2f} Angle: {angle:.2f} Rec R:{RECOVER_RED_S} Rec B:{RECOVER_BLUE_S}")
        #print(f"Red depth:{red_depth:.2F} Blue depth:{blue_depth:.2F}")
        Slalom_counter += rc.get_delta_time()
    elif curr_state == State.idle: # WE start in the base state which avoids obstacles so if no AR markers are detected we want this state
        logging.info("Inside Avoiding Walls function by using Lidar")
        print("this is idlecounter",idle_counter)
        pass
        scan = rc.lidar.get_samples()

        _, left_point = rc_utils.get_lidar_closest_point(scan, (-48, -42))
        _, right_point = rc_utils.get_lidar_closest_point(scan, (42, 48))
        print (left_point, right_point)
        speed = 1
        midpoint = right_point - left_point
        kp = 0.02
        angle = kp * midpoint
        angle = rc_utils.clamp(angle, -1, 1)
        if idle_counter > 26 and idle_counter < 56.1:
            logging.info("Detecting AR Marker which tell us to switch to avoid Natural colored obstacles")
        if -0.2 < angle < 0.2:
            angle = 0
        if idle_counter> 22 and idle_counter < 24.5:
            angle = .05
        if idle_counter > 32 and idle_counter < 33:
            angle = -.6
        if idle_counter> 44 and idle_counter<45:
            angle = -.8
        if idle_counter > 53 and idle_counter < 56.3:
            speed = .4
        if idle_counter > 76.3 and idle_counter < 80:
            speed = 1
            angle = .01
        if idle_counter >80 and idle_counter < 81.3:
            speed = .3
            angle = 1
        if idle_counter > 81.3 and idle_counter < 82.8:
            speed = .3
            angle = -1
        if idle_counter > 82.8 and idle_counter < 88.3:
            speed = 1
            angle = 0
        rc.drive.set_speed_angle(speed, angle)
        idle_counter += rc.get_delta_time()
   
    print(angle)
    # TODO: If we see a marker with ID 199, turn left if the marker faces left and right
    # if the marker faces right

    # TODO: If we see a marker with ID 2, follow the color line which matches the color
    # border surrounding the marker (either blue or red). If neither color is found but
    # we see a green line, follow that instead.
    
    ## debug
    # rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    # lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    # speed = rt-lt
    # angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    
    print(curr_state)

    speed = 0.85

    rc.drive.set_speed_angle(speed, angle)
    
# Shut down the logger
    logging.shutdown()

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()