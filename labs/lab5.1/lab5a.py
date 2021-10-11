"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Lab 5 - AR Markers
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

colors = [
    ((85, 200, 200), (120, 255, 255), "blue"), 
    ((40, 50, 50), (80, 255, 255), "green"), 
    ((170, 50, 50), (10, 255, 255), "red")
]



rc = racecar_core.create_racecar()
speed = 0 
angle = 0

class State (IntEnum):
    idle = 0
    wall_follow_left = 1
    wall_follow_right = 2
    line_follow_blue = 3
    line_follow_red = 4
    line_follow_green = 5
curr_state: State = State.idle
# Add any global variables here

########################################################################################
# Functions
########################################################################################
def line_follower(color):
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
    
    if priority_mode == 2:
        order = [green_contours,red_contours,blue_contours] 
        # print ("Priority: green > red > blue")
    if priority_mode == 3:
        order = [green_contours,blue_contours,red_contours] 
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
    
    angle = rc_utils.remap_range(contour_center[1],0,rc.camera.get_width(),-1,1, True)
    return angle

def wall_follower(scan, right):
    FRONT_WINDOW = (-10, 10)
    FRONT_RIGHT_WINDOW = (25, 35)
    FRONT_LEFT_WINDOW = (325, 335)
    RIGHT_WINDOW = (65, 70)
    LEFT_WINDOW = (285, 290)
    _, fr_dist = rc_utils.get_lidar_closest_point(scan, FRONT_RIGHT_WINDOW)
    _, fl_dist = rc_utils.get_lidar_closest_point(scan, FRONT_LEFT_WINDOW)
    _, r_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW)
    _, l_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW)
    

    if right: 
        if fr_dist > 80:
            angle = 0.45
            if r_dist > 70:
                angle = -1
        elif fl_dist > 80:
            angle = -0.5
            if l_dist > 70:
                angle = 1
        else:
            angle = 0
    else:
        if fl_dist > 80:
            angle = -0.5
            if l_dist > 70:
                angle = 1
        elif fr_dist > 80:
            angle = 0.45
            if r_dist > 70:
                angle = -1
        else:
            angle = 0

    return angle

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 5 - AR Markers")


def update():
    global angle
    global speed
    global curr_state
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    
    for marker in markers:
        
        if marker.get_id() == 0:
            curr_state = State.wall_follow_left
        elif marker.get_id() == 1:
            curr_state = State.wall_follow_right
        
        elif marker.get_id() == 199:
            print(marker.get_orientation())
            if marker.get_orientation().value == 1:
                curr_state = State.wall_follow_left
            else:
                curr_state = State.wall_follow_right
        
        elif marker.get_id() == 2:
            marker.detect_colors(color_image, colors)
            if marker.get_color() == "blue":
                curr_state = State.line_follow_blue
            elif marker.get_color() == "red":
                curr_state = State.line_follow_red
            else:
                curr_state = State.line_follow_green
    # TODO: Turn left if we see a marker with ID 0 and right for ID 1
    
    scan = rc.lidar.get_samples()
    if curr_state == State.wall_follow_left:
        angle = wall_follower(scan, False)
    elif curr_state == State.wall_follow_right:
        angle = wall_follower(scan, True)
    elif curr_state == State.line_follow_blue: 
        angle = line_follower("blue")
    elif curr_state == State.line_follow_red:
        angle = line_follower("red")
        
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


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()