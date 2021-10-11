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
BLUE = ((88,245,199), (108,255,255), "BLUE")
RED = ((0, 50, 50), (20, 255, 255), "RED")
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
        print(ar_marker.get_orientation())
    
    
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()