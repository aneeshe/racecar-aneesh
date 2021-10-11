"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Final Challenge - Time Trials
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import numpy.polynomial.polynomial as npp
import scipy.cluster
import scipy.interpolate
import enum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from PID import PID
#from pidcontroller import pidcontroller

########################################################################################
# Global variables
########################################################################################


class Challenge(enum.IntEnum):
    Line = 1
    Lane = 2
    Cones = 3
    # Slalom = 3
    # Gate = 4
    Wall = 4
    ManualControl = 5


class Color(enum.IntEnum):
    Red = 1
    Blue = 2
    Green = 3
    Orange = 4
    Purple = 5
    White = 6


class WaypointType(enum.IntEnum):
    Red = 1
    Blue = 2
    Gate = 3
    Unknown = 4
    Self = 5


coneparams = cv.SimpleBlobDetector_Params()

coneparams.filterByColor = False
coneparams.filterByArea = True
coneparams.minArea = 10
# coneparams.maxArea = 500
coneparams.filterByCircularity = False
coneparams.filterByConvexity = False
coneparams.filterByInertia = True
coneparams.maxInertiaRatio = 2

conedetector = cv.SimpleBlobDetector_create(coneparams)

rc = racecar_core.create_racecar()

DATAVIS = True

VIS_RADIUS = 300
LINE_RADIUS = 100
LANE_HORIZ_RADIUS = 60
# VIS_SCALE = 0.5  # 0.5cm units
LIDAR_OFFSET = 1  # scale things closer

# perspective transformation matrix

width = 0
height = 0
currentChallenge = Challenge.ManualControl
oldState = Challenge.Line
last_waypoint_type = None

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((80, 120, 100), (125, 255, 255))
GREEN = ((45, 120, 100), (75, 255, 255))
RED1 = ((170, 84, 100), (180, 255, 255))
RED2 = ((0, 84, 100), (10, 255, 255))
PURPLE = ((125, 90, 100), (140, 255, 255))
ORANGE = ((10, 95, 100), (25, 255, 255))

Angle_PID = PID(4, 0.5, 0.5)  # d = 0.5

colorPriority = None
oldCones = None

########################################################################################
# Functions
########################################################################################


def p2c(r, t):
    """polar r (float), theta (float) to cartesian x (float), y (float)"""
    if np.isnan(r) or np.isnan(t):
        return 0, 0  # return 0, 0
    else:
        return r * np.sin(t), r * np.cos(t)  # x, y


def c2p(x, y):
    """cartesian x (float), y (float) to polar r (float), theta (float)"""
    if np.isnan(x) or np.isnan(y):
        return 0, 0  # return 0, 0
    else:
        r = np.sqrt(x ** 2 + y ** 2)
        t = np.arctan2(y, x)
        return r, t


Vp2c = np.vectorize(p2c, [np.float32, np.float32])
Vc2p = np.vectorize(c2p, [np.float32, np.float32])

#    Y+
# X- car X+
#    Y-


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    global width
    global height
    width = rc.camera.get_width()
    height = rc.camera.get_height()
    # rc.drive.set_max_speed(1)

    global currentChallenge
    global oldState
    currentChallenge = Challenge.ManualControl
    oldState = Challenge.Line

    global colorPriority
    colorPriority = None

    global oldCones
    oldCones = None

    global last_waypoint_type
    last_waypoint_type = None

    # Print start message
    print(">> Final Challenge - Time Trials")


def manualControl():
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    return speed, angle


def lidar2TopDown(scan, start=0, end=360):
    r = 360.0 / rc.lidar.get_num_samples()
    scan_r = np.take(scan, np.arange(start / r, end / r).astype(np.int), mode="wrap")
    # scan_r[abs(scan_r - np.mean(scan_r)) > 2 * np.std(scan_r)] = 0
    scan_t = np.radians(np.arange(start, end, step=r))
    scan_xy = np.array(Vp2c(scan_r, scan_t))
    valid_points = np.all(scan_xy != 0, axis=0)

    return np.transpose(scan_xy[:, valid_points])


def camera2Polar(points, depths):  # t, r
    """transform points in color image space to top down space"""
    # fov 45 degrees
    angles = np.radians(
        (points[:, 1] - width // 2) * 69.4 / width  # 74
    )  # convert from x to angle

    # angles = np.radians([10, 20, 30, 40, 50])
    # d = np.array([10, 10, 10, 10, 10])

    # print(np.array(Vp2c(d, angles)))
    a = np.argwhere(np.logical_and(depths > 0, depths <= VIS_RADIUS)).flatten()

    # print(depths, depths[a])

    return depths[a], angles[a]


def polar2TopDown(r, t):
    return np.transpose(np.array(Vp2c(r, t)))


def topDown2Vis(points):
    """transform points in top down space to visualization space"""
    if points is None or points.size == 0:
        return None

    i = np.copy(points)
    # print(i)
    i = i[(np.absolute(i) < VIS_RADIUS).all(axis=1)]
    i[:, 1] = np.negative(i[:, 1])  # invert y axis
    # print(i)
    i = i.astype(int) + VIS_RADIUS
    return (tuple(i[:, 1]), tuple(i[:, 0]))


def vis2TopDown(points):
    """transform points in visualization space to top down space"""
    if points is None or points.size == 0:
        return None

    i = np.array(points)
    i = i.astype(float) - VIS_RADIUS
    i[:, 0] = np.negative(i[:, 0])  # invert y axis
    return np.array([i[:, 1], i[:, 0]])


def visualizeColor(vis_image, hsv_image, depth_image, incolor, outcolor):
    mask = cv.inRange(hsv_image, incolor[0], incolor[1])
    # rc.display.show_color_image(mask)
    points = np.argwhere(mask != 0)  # ignores black
    depths = depth_image[points[:, 0], points[:, 1]]
    r, t = camera2Polar(points, depths)
    i = topDown2Vis(polar2TopDown(r, t))
    if i is not None:
        vis_image[i] = outcolor  # add pixels in color image


def fitCurveToPath(path, vis_image):
    # l = path.shape[0]

    np.append(path, np.zeros((20, 2)))

    try:
        # if l < 6000:
        #    polynomial = npp.polyfit(path[:, 1], path[:, 0], 1)
        # else:
        polynomial = npp.polyfit(path[:, 1], path[:, 0], 2)
    except:
        print("Polyfit error")
        return None

    i = topDown2Vis(
        np.transpose(
            [npp.polyval(np.arange(-100, 100), polynomial), np.arange(-100, 100)]
        )
    )
    if i is not None:
        vis_image[i] = [255, 255, 0]  # add pixels in color image
    return polynomial


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    scan = np.clip(
        rc.lidar.get_samples() * LIDAR_OFFSET, 0, None
    )  # smooth(rc.lidar.get_samples())

    scan_xy = None

    color_image = rc.camera.get_color_image()
    depth_image = cv.bilateralFilter(rc.camera.get_depth_image(), 9, 75, 75)
    vis_image = np.zeros((2 * VIS_RADIUS, 2 * VIS_RADIUS, 3), np.uint8, "C")
    hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

    # FSM

    speed = 0
    angle = 0
    global currentChallenge
    global oldState
    global colorPriority

    if currentChallenge == Challenge.ManualControl:
        speed, angle = manualControl()
        if rc.controller.was_pressed(rc.controller.Button.A):
            currentChallenge = oldState
    else:
        if rc.controller.was_pressed(rc.controller.Button.A):
            oldState = currentChallenge
            currentChallenge = Challenge.ManualControl

    curve = None
    path = None
    if currentChallenge == Challenge.Line:
        if colorPriority == None:
            # scan AR tags
            colorPriority = [
                Color.Red,
                Color.Green,
                Color.Blue,
            ]  # {Color.Red: 1, Color.Blue: 2, Color.Green: 3}

        hsv_image[0 : height // 2, :] = [0, 0, 0]  # crop out top half

        if colorPriority[2] == Color.Blue:
            blue_r = np.array([])
            blue_t = np.array([])
        else:
            mask = cv.inRange(hsv_image, BLUE[0], BLUE[1])
            # rc.display.show_color_image(mask)
            points = np.argwhere(mask != 0)
            depths = depth_image[points[:, 0], points[:, 1]]
            blue_r, blue_t = camera2Polar(points, depths)

        if colorPriority[2] == Color.Red:
            red_r = np.array([])
            red_t = np.array([])
        else:
            mask = cv.bitwise_or(
                cv.inRange(hsv_image, RED1[0], RED1[1]),
                cv.inRange(hsv_image, RED2[0], RED2[1]),
            )
            # rc.display.show_color_image(mask)
            points = np.argwhere(mask != 0)
            depths = depth_image[points[:, 0], points[:, 1]]
            red_r, red_t = camera2Polar(points, depths)

        if colorPriority[2] == Color.Green:
            green_r = np.array([])
            green_t = np.array([])
        else:
            mask = cv.inRange(hsv_image, GREEN[0], GREEN[1])
            # rc.display.show_color_image(mask)
            points = np.argwhere(mask != 0)
            depths = depth_image[points[:, 0], points[:, 1]]
            green_r, green_t = camera2Polar(points, depths)

            depths = np.concatenate([blue_r, red_r, green_r])
            sort = np.argsort(depths)

        points = np.array(
            [
                depths[sort],
                np.concatenate([blue_t, red_t, green_t])[sort],
                np.concatenate(
                    [
                        np.full_like(blue_r, Color.Blue),
                        np.full_like(red_r, Color.Red),
                        np.full_like(green_r, Color.Green),
                    ]
                )[sort],
            ]
        )

        path = None

        if len(depths) > 5:
            final_r = np.array([])
            final_t = np.array([])

            oldt = -1

            for i in range(LINE_RADIUS // 5):  # increments of 10 units
                p = points[
                    :,
                    np.logical_and(points[0, :] >= i * 5, points[0, :] < (i + 1) * 5),
                ]
                l = p.shape[1]
                if l > 0:
                    unique = np.unique(p[2])
                    # d = dict(zip(unique, counts))
                    for c in colorPriority:
                        if c in unique:
                            args = np.argwhere(p[2] == c)
                            tlist = p[1, args]
                            c_t = np.mean(tlist)
                            if oldt == -1 or abs(c_t - oldt) < 0.4:  # radians
                                final_r = np.append(final_r, p[0, args])
                                final_t = np.append(final_t, tlist)
                                oldt = c_t
                                break
                    # else:
                    #    final_r = np.append(final_r, p[0])
                    #    final_t = np.append(final_t, p[1])

            path = polar2TopDown(final_r, final_t)
            path[:, 1] -= 25
            curve = fitCurveToPath(path, vis_image)
    if currentChallenge == Challenge.Lane:
        mask = cv.bitwise_or(
            cv.inRange(hsv_image, PURPLE[0], PURPLE[1]),
            cv.inRange(hsv_image, ORANGE[0], ORANGE[1]),
        )
        # rc.display.show_color_image(mask)
        points = np.argwhere(mask != 0)
        depths = depth_image[points[:, 0], points[:, 1]]
        r, t = camera2Polar(points, depths)
        path = polar2TopDown(r, t)

        if len(path) > 0:
            final_x = np.array([])
            final_y = np.array([])

            # path = path[np.absolute(path[:, 1]) < LANE_HORIZ_RADIUS]

            for i in range(VIS_RADIUS // 5):
                p = path[np.logical_and(path[:, 1] >= i * 5, path[:, 1] < (i + 1) * 5)]
                if len(p) > 0:
                    y = p[:, 1]
                    p = p[:, 0]
                    m = np.mean(p)
                    left_mean = np.mean(p[p < m])
                    right_mean = np.mean(p[p > m])
                    if (
                        abs(left_mean - right_mean) < 5
                        or abs(left_mean - right_mean) > 100
                    ):
                        continue  # throw out this data, one side is not visible
                    m = (left_mean + right_mean) / 2
                    p[p > m] += m - right_mean
                    p[p < m] += m - left_mean
                    final_x = np.append(final_x, p)
                    final_y = np.append(final_y, y)

            path = np.transpose([final_x, final_y])
            curve = fitCurveToPath(path, vis_image)

    if curve is not None:  # line or lane gave valid result
        # speed = 0.5
        slope = npp.polyval(5, np.polyder(curve))
        error = slope / 50
        # angleError = TARGET_ANGLE - (rightWallAngle + leftWallAngle) / 2
        # distError = npp.polyval(0, curve)

        # if abs(angleError) < ANGLE_THRESHHOLD:
        #    error = distError
        # else:
        # error = distError / 10 + np.sin(np.radians(angleError)) * 2  # angleError / 30

        # angle = rc_utils.clamp(Angle_PID.update(error, rc.get_delta_time()), -1, 1)

    if True:  # currentChallenge == Challenge.Cones:
        blue_image = np.zeros((VIS_RADIUS * 2, VIS_RADIUS * 2), dtype=np.uint8)
        red_image = np.zeros((VIS_RADIUS * 2, VIS_RADIUS * 2), dtype=np.uint8)

        visualizeColor(blue_image, hsv_image, depth_image, BLUE, 255)
        visualizeColor(red_image, hsv_image, depth_image, RED1, 255)
        visualizeColor(red_image, hsv_image, depth_image, RED2, 255)

        cones = []

        keypoints = conedetector.detect(blue_image)
        # vis_image = cv.drawKeypoints(
        #    vis_image,
        #    keypoints,
        #    np.array([]),
        #    (0, 255, 255),
        #    cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        # )

        cones.append([[k.pt[0], k.pt[1], WaypointType.Blue] for k in keypoints])

        keypoints = conedetector.detect(red_image)
        # vis_image = cv.drawKeypoints(
        #    vis_image,
        #    keypoints,
        #    np.array([]),
        #    (0, 255, 255),
        #    cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        # )

        cones.append([[k.pt[0], k.pt[1], WaypointType.Red] for k in keypoints])

        scan_xy = lidar2TopDown(scan)  # , 30, 330)
        scan_xy[:, 1] -= 15  # lidar offset
        scan_xy = scan_xy[
            (np.absolute(scan_xy) < 60).all(axis=1)
        ]  # scipy.cluster.vq.whiten(scan_xy)

        centroids = None
        if scan_xy.size > 5:
            try:
                centroids, distortion = scipy.cluster.vq.kmeans(scan_xy, 3)
            except:
                print("K means error")

        # print(c)

        if centroids is not None:
            v = topDown2Vis(centroids)
            for i in range(len(v[0])):
                cones.append([v[1][i], v[0][i], WaypointType.Unknown])
                # x = v[1][i]
                # y = v[0][i]
                # if x < VIS_RADIUS:  # and y > VIS_RADIUS:
                #    red_cones.append([x, y])
                # else:
                #    blue_cones.append([x, y])
                # if c is not None:
                #    dist = min(np.minimum(c[:, 0] - x), np.minimum(c[:, 1] - y))
                #    if dist > 10:
                #        rc_utils.draw_circle(vis_image, (x, y), (0, 255, 255), 5)
                # else:
                # rc_utils.draw_circle(vis_image, (x, y), (0, 255, 255), 5)

        if len(cones) > 0:
            for i in range(len(waypoints)):

                x = int(rc_utils.clamp(waypoints[c][0], 0, VIS_RADIUS * 2))
                y = int(rc_utils.clamp(waypoints[c][1], 0, VIS_RADIUS * 2))
                for i in waypoints[c + 1 :]:
                    d = (i[0] - x) ** 2 + (i[1] - y) ** 2
                    if d < 100:
                        break

        waypoints = []
        gate_forming_cones = []

        for i in red_cones:
            for j in blue_cones:
                if abs(j[1] - i[1]) < 40 and 20 < abs(j[0] - i[0]) < 100:  # found gate
                    x = abs(j[0] - i[0]) / 2
                    # y = (j[1] + i[1]) / 2
                    # if y >= 20:
                    waypoints.append([j[0] - x, j[1], WaypointType.Gate])
                    waypoints.append([i[0] + x, i[1], WaypointType.Gate])
                    gate_forming_cones.append(i)
                    gate_forming_cones.append(j)
                    # break

        # print(gate_forming_cones)

        # print(waypoints)
        firstrun = True
        for a in red_cones:
            x = int(a[1])
            y = int(a[0])
            rc_utils.draw_circle(vis_image, (x, y), (0, 127, 255), 5)
            for b in blue_cones:
                xb = int(b[1])
                yb = int(b[0])
                if firstrun:
                    rc_utils.draw_circle(vis_image, (xb, yb), (255, 255, 0), 5)
                d = (xb - x) ** 2 + (yb - y) ** 2
                if d < 100:
                    gate_forming_cones.append(a)
            firstrun = False

        for i in red_cones:
            if i not in gate_forming_cones:
                waypoints.append([i[0] + 30, i[1], WaypointType.Red])
                waypoints.append([i[0] + 20, i[1] - 20, WaypointType.Red])
                waypoints.append([i[0] + 20, i[1] + 20, WaypointType.Red])

        for i in blue_cones:
            if i not in gate_forming_cones:
                waypoints.append([i[0] - 30, i[1], WaypointType.Blue])
                waypoints.append([i[0] - 20, i[1] - 20, WaypointType.Blue])
                waypoints.append([i[0] - 20, i[1] + 20, WaypointType.Blue])

        if len(waypoints) > 0:
            w = []

            for c in range(len(waypoints)):
                x = int(rc_utils.clamp(waypoints[c][0], 0, VIS_RADIUS * 2))
                y = int(rc_utils.clamp(waypoints[c][1], 0, VIS_RADIUS * 2))
                for i in waypoints[c + 1 :]:
                    d = (i[0] - x) ** 2 + (i[1] - y) ** 2
                    if d < 100:
                        break
                else:
                    w.append(waypoints[c])
                    rc_utils.draw_circle(vis_image, (y, x), (0, 255, 0), 5)

            # waypoints.append([VIS_RADIUS, VIS_RADIUS, WaypointType.Self])
            waypoints = np.array(w)

            waypoints[:, 0:2] -= VIS_RADIUS
            waypoints[:, 1] = np.negative(waypoints[:, 1])
            # print(waypoints)

            # fit curve to path
            try:
                curve = scipy.interpolate.interp1d(
                    waypoints[:, 1],
                    waypoints[:, 0],
                    # type="cubic",
                    fill_value="extrapolate",
                )
            except:
                print("Spline curve error")
                curve = None

    if curve is not None:
        i = topDown2Vis(
            np.transpose(
                [
                    curve(np.arange(-VIS_RADIUS, VIS_RADIUS)),
                    np.arange(-VIS_RADIUS, VIS_RADIUS),
                ]
            )
        )
        if i is not None:
            vis_image[i] = [255, 255, 0]  # add pixels in color image

        speed = 0.2
        slope = (curve(0.1) - curve(0)) / 0.1
        error = curve(0) / 10  # + slope * 5  # angleError / 30
        if np.isfinite(error):
            angle = rc_utils.clamp(Angle_PID.update(error, rc.get_delta_time()), -1, 1)
        global last_waypoint_type
        last_waypoint_type = waypoints[np.argmin(np.absolute(waypoints[:, 1])), 2]
    else:
        if last_waypoint_type == WaypointType.Blue:
            speed = 0.2
            angle = 1
        elif last_waypoint_type == WaypointType.Red:
            speed = 0.2
            angle = -1

    # print("centroids  : ", centroids)
    # print("distortion :", distortion)

    # pass
    # if currentChallenge == Challenge.Gate:
    #    pass
    if currentChallenge == Challenge.Wall:
        pass

    # green dot in middle for car
    rc_utils.draw_circle(vis_image, (VIS_RADIUS, VIS_RADIUS), (0, 255, 255), 2)

    if scan_xy is not None:
        i = topDown2Vis(scan_xy)
        if i is not None:
            vis_image[i] = [255, 255, 255]

    visualizeColor(vis_image, hsv_image, depth_image, BLUE, (255, 127, 0))
    visualizeColor(vis_image, hsv_image, depth_image, RED1, (0, 0, 255))
    visualizeColor(vis_image, hsv_image, depth_image, RED2, (0, 0, 255))
    # visualizeColor(vis_image, hsv_image, depth_image, GREEN, (0, 255, 0))

    if path is not None:
        i = topDown2Vis(path)
        if i is not None:
            vis_image[i] = [0, 255, 255]  # add pixels in color image

    # mask[points[:, 0], points[:, 1]] = depths
    # rc.display.show_depth_image(mask)

    # red = (255, 0, 0)
    # blue = (255, 127, 0)
    # green = (0, 255, 0)
    # orange = (0, 127, 255)
    # purple = (255, 0, 127)
    # visualizeColor(vis_image, hsv_image, depth_image, PURPLE, (255, 0, 127))
    # visualizeColor(vis_image, hsv_image, depth_image, ORANGE, (0, 127, 255))

    rc.display.show_color_image(vis_image)
    # rc.display.show_depth_image(depth_image)

    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()