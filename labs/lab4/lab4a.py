"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Lab 4A - LIDAR Safety Stop
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

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
# >> Constants
# The (min, max) degrees to consider when measuring forward and rear distances
FRONT_WINDOW = (-10, 10)
REAR_WINDOW = (170, 190)
FRONT_STOP = False
BACK_STOP = False
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(
        ">> Lab 4A - LIDAR Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Right bumper = override forward safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left bumper = override rear safety stop\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print forward and back distances"
    )


def update():
    global FRONT_STOP
    global BACK_STOP
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    input_speed = rt - lt
    # Calculate the distance in front of and behind the car
    scan = rc.lidar.get_samples()
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    _, back_dist = rc_utils.get_lidar_closest_point(scan, REAR_WINDOW)

    # TODO (warmup): Prevent the car from hitting things in front or behind it.
    # Allow the user to override safety stop by holding the left or right bumper.
    speed = 0

    FRONT_STOP = True if forward_dist < 80 else False
    BACK_STOP = True if back_dist < 80 else False


    if rc.controller.is_down(rc.controller.Button.RB) or rc.controller.is_down(rc.controller.Button.LB): 
        FRONT_STOP = False
        BACK_STOP = False
        speed = input_speed
    else:
        if input_speed > 0 and not FRONT_STOP and forward_dist < 250:
            speed = rc_utils.remap_range(forward_dist, 350, 0, 1,0, True)
            print("Front:", speed)
        elif input_speed < 0 and not BACK_STOP and back_dist < 250:
            speed = rc_utils.remap_range(back_dist, 350, 0, -1,0, True)
            print("BACK:", speed)
        elif input_speed > 0:
            speed = input_speed if not FRONT_STOP else  0
        elif  input_speed < 0:
            speed = input_speed if not BACK_STOP else 0






    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the distance of the closest object in front of and behind the car
    if rc.controller.is_down(rc.controller.Button.B):
        print("Forward distance:", forward_dist, "Back distance:", back_dist)

    # Display the current LIDAR scan
    rc.display.show_lidar(scan)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()