"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Grand Prix 2021
"""

########################################################################################
# Imports
########################################################################################

#from labs.lab4.lab4b import DRIVE_SPEED, LEFT_WINDOW
#from labs.lab4.lab4b import FRONT_WINDOW
import sys
import cv2 as cv
import numpy as np
sys.path.insert(0, "../../library")

import racecar_core
import racecar_utils as rc_utils
from racecar_utils import ARMarker
from enum import IntEnum


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

### LINE FOLLOWING ###
BLUE = ((100,150,150), (120,255,255),"BLUE") 
RED = ((170, 150, 150), (10, 255, 255), "RED")
GREEN = ((40, 60, 60), (90, 255, 255), "GREEN") 
WHITE = ((90, 20, 200), (115, 60, 255), "WHITE")

CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

MIN_CONTOUR_AREA = 30

potential_colors = [BLUE, RED, GREEN]

speed = 0
angle = 0

time =0.0
# Camera values.
contour_center = None
contour_area = 0

cur_color = None
contour_distance = 0.0

cone_counter = 0
prev_color = None

MIN_CONTOUR_AREA = 650
########################################################################################
# Functions
########################################################################################

class State(IntEnum):
    greenLine = 0
    wallFollow = 1
    purpleLine = 2
    orangePillar = 3
    elevator = 4
    cone = 5
    train = 6
    orangePlate = 7
    jump = 8

curState = State.greenLine

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    ar_marker: ARMarker = None
    

    if len(markers) > 0:
        ar_marker = markers[0]

    cone()

    rc.drive.set_speed_angle(speed,angle)

def update_contour_cone():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global cur_color
    global contour_distance

    image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    depth_image_adjust = (depth_image - 0.01) % 9999
    depth_image_adjust_blur = cv.GaussianBlur(depth_image_adjust, (11,11), 0)

    contour = None

    if image is None:
        contour_center = None
        contour_area = 0
    else:

        red_contours = rc_utils.find_contours(image, RED[0], RED[1])
        blue_contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])

        largest_red_contour = rc_utils.get_largest_contour(red_contours, MIN_CONTOUR_AREA)
        largest_blue_contour = rc_utils.get_largest_contour(blue_contours, MIN_CONTOUR_AREA)

        if largest_red_contour is not None:
            largest_red_contour_area = rc_utils.get_contour_area(largest_red_contour)
            largest_red_contour_distance = rc_utils.get_pixel_average_distance(depth_image_adjust_blur, rc_utils.get_contour_center(largest_red_contour))
        else:
            largest_red_contour_area = 0
            largest_red_contour_distance = 0

        if largest_blue_contour is not None:
            largest_blue_contour_area = rc_utils.get_contour_area(largest_blue_contour)
            largest_blue_contour_distance = rc_utils.get_pixel_average_distance(depth_image_adjust_blur, rc_utils.get_contour_center(largest_blue_contour))
        else:
            largest_blue_contour_area = 0
            largest_blue_contour_distance = 0

        # Select the largest contour
        if largest_red_contour_area > largest_blue_contour_area and largest_red_contour_distance < 500:
            contour = largest_red_contour
            cur_color = RED[2]
        elif largest_blue_contour_area > largest_red_contour_area and largest_blue_contour_distance < 500:
            contour = largest_blue_contour
            cur_color = BLUE[2]
        else:
            contour = None

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            contour_center_x = rc_utils.clamp(contour_center[0], 0, 639)
            contour_center_y = rc_utils.clamp(contour_center[1], 0, 479)

            contour_distance = rc_utils.get_pixel_average_distance(depth_image_adjust_blur, contour_center)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0
            #contour_distance = 0
        # Display the image to the screen
        rc.display.show_color_image(image)
    
def cone():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    
    update_contour_cone()

    # color_img = rc.camera.get_color_image()
    # depth_img = rc.camera.get_depth_image()
    
    # Global variables.
    global speed
    global angle
    global cur_color
    global contour_distance
    global cone_counter
    global prev_color
    global time

    # Variables.
    speed = 0.0
    angle = 0.0

    color_img_x = rc.camera.get_width()

    if prev_color != cur_color:
        cone_counter += 1
        prev_color = cur_color


    if cur_color == 'BLUE':
        if contour_center is not None:
            point = rc_utils.remap_range(contour_distance, 10, 300, color_img_x, color_img_x * 3 // 4 , True)
            #speed = rc_utils.remap_range(contour_distance,30, 120,0.8,1,True,)
            speed = 1
            angle = rc_utils.remap_range(contour_center[1], point, color_img_x // 2 , 0 ,-0.62 ,True)
            if contour_distance > 175:
                angle = -0.16
        else:
            angle = 0.32
            speed = 1
        
    elif cur_color == 'RED':
        if contour_center is not None:
            point = rc_utils.remap_range(contour_distance, 50, 300, 0, color_img_x // 2, True)
            #speed = rc_utils.remap_range(contour_distance,30, 120,0.7,1,True,)
            speed = 1
            angle = rc_utils.remap_range(contour_center[1], point, color_img_x // 2 , 0 ,0.62 ,True)
            if contour_distance > 175:
                angle = 0.16
        else:
            angle = -0.32
            speed = 1
    
    #print(cone_counter)
    print(cone_counter)
    print(contour_distance)


    

    
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()