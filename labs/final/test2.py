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
from typing import final
import cv2 as cv
import numpy as np
#from simple_pid import PID
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
BLUE = ((88,245,199), (108,255,255), "BLUE")
RED = ((0, 50, 50), (20, 255, 255), "RED")
GREEN = ((40, 60, 60), (90, 255, 255), "GREEN") 
WHITE = ((90, 20, 200), (115, 60, 255), "WHITE")
WHITEMARKER = ((97, 43, 109), (117, 63, 189), "WHITE")
ORANGEMARKER = ((7, 172, 78), (27, 192, 158), "ORANGE")#
ORANGELINE = ((5, 245, 215), (25, 255, 255), "ORANGE")
PURPLEMARKER =  ((121, 192, 109), (141, 212, 189), "PURPLE")#
PURPLELINE = ((125, 245, 215), (145, 255, 255), "PURPLE")

CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

LEFT_WINDOW_LIDAR = (-135, -45)
RIGHT_WINDOW_LIDAR = (45, 135)
FRONT_WINDOW_LIDAR = (-10, 10)
BACK_WINDOW_LIDAR = (170, 190)

MIN_CONTOUR_AREA = 30

potential_colors_markers = [PURPLEMARKER, ORANGEMARKER] # WHITEMARKER]
potential_colors_lines = [PURPLELINE, ORANGELINE]

speed = 0
angle = 0

arColor = None
currentColor = None

rightLine = 0

counter = 0

finalJump = False

### WALL FOLLOWING ###
FRONT_WINDOW = (-10,10)
LEFT_WINDOW = (-50, -40) # center : -45
RIGHT_WINDOW = (40, 50) # center : 45

########################################################################################
# Functions
########################################################################################

class State(IntEnum):
    greenLine = 0       #done
    wallFollow = 1      #almost done
    canyon = 2          #done
    orangePillar = 3    #done
    elevator = 4        #done
    cone = 5            #almost done
    train = 6
    orangePlate = 7
    jump = 8            #done

currState = State.greenLine

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    #rc.drive.set_max_speed(2)
    rc.drive.stop()

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed, currState, arColorGlobal
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    ar_marker: ARMarker = None
    

    if len(markers) > 0:
        ar_marker = markers[0]
        #print(ar_marker.get_id())
        ar_marker.detect_colors(color_image, potential_colors_markers)
        arColor = ar_marker.get_color()
        arColorGlobal = arColor
        #print("ID IN UPDATE:", ar_marker.get_id())
        #print("COLOR IN UPDATE: ", arColorGlobal)

        if ar_marker.get_id() == 1:
            currState = State.canyon
        if ar_marker.get_id() == 8:
            currState = State.jump
        
    if currState == State.canyon:
        print("Passing in: ", arColorGlobal)
        canyonLineFollowing(arColorGlobal)
    
    if currState == State.jump:
        finalStageLineFollowing()

    rc.drive.set_speed_angle(speed, angle)

def update_contour(colorList, image):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global currentColor
    
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        #print("in update cont")
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        rc.display.show_color_image(image)
        for color in colorList:
            contours = rc_utils.find_contours(image, color[0], color[1])
            if len(contours) != 0:
                break

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        if contour is not None:
            print("found cont")
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            currentColor = color[2]
            print(f"Current color: {currentColor}")
            
        else:
            contour_center = None
            contour_area = 0
    
def canyonLineFollowing(arColor):
    global speed
    global angle
    global rightLine
    global image
    #print(order)

    image = rc.camera.get_color_image()
    print("RIGHT LINE: ", rightLine)
    if rightLine % 2 == 0:
        print("in correct image")
        image = rc_utils.crop(image, (0, 3 * rc.camera.get_width() // 4), (rc.camera.get_height(), rc.camera.get_width()))
    elif rightLine % 2 == 1:
        image = rc_utils.crop(image, (0, 0), (rc.camera.get_height(), rc.camera.get_width() // 4))
    
    # rc.display.show_color_image(image)


    update_contour([PURPLELINE, ORANGELINE], image)
    # if arColor == "PURPLE":
    #     update_contour([PURPLE, ORANGE])
    # elif arColor == "ORANGE":
    #     update_contour([ORANGE, PURPLE])

    imgWidth = rc.camera.get_width()
    halfImgWidth = imgWidth // 2

    quarterImgWidth = rc.camera.get_width() // 4

    if contour_center is not None:
        print("AR Color:" ,arColor)
        #print("Current Color: ", currentColor)

        if arColor == "PURPLE" or arColor == "ORANGE":
            if (currentColor == "PURPLE" and arColor == "PURPLE") or (currentColor == "ORANGE" and arColor == "ORANGE"):
                    print("Contour center: ",contour_center[1])
                    print("RIGHT LINE FOLLOWING")
                #center = rc_utils.clamp(contour_center[1], 3 * imgWidth // 4, imgWidth)
                #centerInv = rc_utils.clamp(contour_center[1], 3 * imgWidth // 4, imgWidth)
                    
            if (currentColor == "PURPLE" and arColor == "ORANGE") or (currentColor == "ORANGE" and arColor == "PURPLE"):
                rightLine += 1
                print("Contour center: ",contour_center[1])
                print("LEFT LINE FOLLOWING")
                
            if rightLine % 2 == 0:
                #RIGHT FOLLOWING
                #print("Changing angle")
                #print("Lower bound: ", int(0.75 * quarterImgWidth))
                #print("Upper bound: ", quarterImgWidth)

                centerInv = rc_utils.clamp(contour_center[1], int(0.75 * quarterImgWidth), quarterImgWidth)
                angle = rc_utils.remap_range(centerInv, int(0.75 * quarterImgWidth), quarterImgWidth, -1, 1)
                print("Angle: ", angle)
            if rightLine % 2 == 1:
                #LEFT FOLLOWING
                centerInv = rc_utils.clamp(contour_center[1], 0, int(0.25 * quarterImgWidth))
                angle = rc_utils.remap_range(centerInv, 0, int(0.25 * quarterImgWidth), -1, 1)

    speed = 0.6

def finalStageLineFollowing():
    global speed
    global angle
    global rightLine
    global image, finalJump, counter
    #print(order)

    image = rc.camera.get_color_image()
    print("RIGHT LINE: ", rightLine)
    image = rc_utils.crop(image, (0, 3 * rc.camera.get_width() // 4), (rc.camera.get_height(), rc.camera.get_width()))
    
    # rc.display.show_color_image(image)

    scan = rc.lidar.get_samples()
    _, left_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW_LIDAR)
    _, right_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW_LIDAR)
    _, front_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW_LIDAR)
    _, back_dist = rc_utils.get_lidar_closest_point(scan, BACK_WINDOW_LIDAR)

    update_contour([BLUE], image)

    imgWidth = rc.camera.get_width()
    halfImgWidth = imgWidth // 2

    quarterImgWidth = rc.camera.get_width() // 4
    print(f"Front: {front_dist}   Back: {back_dist}")

    if counter > 18 and ((front_dist < 170 and back_dist > 145) or finalJump == True):
        print("final ramp")
        speed = 3
        angle = 0
        finalJump = True
    if finalJump == False:
        counter += rc.get_delta_time()
        print("Counter: ", counter)
        if contour_center is not None:
            print("Current Color: ", currentColor)

            centerClamped = rc_utils.clamp(contour_center[1], int(0.75 * quarterImgWidth), quarterImgWidth)
            angle = rc_utils.remap_range(centerClamped, int(0.75 * quarterImgWidth), quarterImgWidth, -1, 1)
            print("Angle: ", angle)
            speed = 2

        if contour_center is None:
            angle = 0.3
            speed = 2.0



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()