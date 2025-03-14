print("STARTING...")

import argparse
import math
import time

import cv2
import numpy as np

import spidev

import YOLOP.tools.detect as yolop_detect

import util
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
SHOW_DEBUG_FRAMES = False
TARGET_FPS = 30


# Constants for "trackvideo.mp4"
HORIZON_Y_RATIO = 17/36
KART_Y_RATIO = 32/36
BOTTOM_Y_RATIO = 19/20   # Used in grass detection
ON_TRACK_Y_RATIO = 30/36 # Used in track detection
TARGET_Y_RATIO = 6/10

# Constants for "pov.mp4"
# HORIZON_Y_RATIO = 10/36
# KART_Y_RATIO = 20/36
# BOTTOM_Y_RATIO = 19/20   # Used in grass detection
# ON_TRACK_Y_RATIO = 18/36 # Used in track detection
# TARGET_Y_RATIO = 3.5/10

HISTORY_TIME = 1.0 # seconds

# Determine the "true" target point
GRASS_TARGET_WEIGHT = 2.0
TRACK_TARGET_WEIGHT = 1.0

# Convert the target_x to degrees
CAMERA_FOV = 60 # degrees
CAMERA_FOV_SCALE_FACTOR = math.tan(util.deg_to_rad(CAMERA_FOV / 2.0))

# SPI communication
# SPI format: [sign, steering, throttle] 
#   sign = 1 if steering negative, 0 otherwise
#   steering maps [0, CAMERA_FOV] angle to [0, 255] value
#   throttle is the throttle percentage
SPI_BUS = 1 # Jetson SPI1
SPI_DEVICE = 0 # CS0
SPI_SPEED = 500000 # 500kHz
SPI_MODE = 0b00 # SPI mode 0 (CPOL=0, CPHA=0)
SPI_MAX = 255 # 8 bit max value

# Steering control
DECAY_RATE = 0.3 # when the center is not found

# Throttle control
THROTTLE_LOW = 2 # percent
THROTTLE_HIGH = 5 # percent


# Track thresh constants 
TRACK_MAX_TOTAL_DIFF = 85 
TRACK_MAX_INDIVIDUAL_DIFF = 45
TRACK_MAX_INTENSITY = 200
TRACK_ERODE_KERNEL = (5, 5)
TRACK_ERODE_ITERATIONS = 5
TRACK_DILATE_KERNEL = (3, 3)
TRACK_DILATE_ITERATIONS = 7

# Grass thresh constants
GRASS_HSV_LOW = (25, 40, 40)
GRASS_HSV_HIGH = (80, 255, 255)
GRASS_ERODE_KERNEL = (7, 7)
GRASS_ERODE_ITERATIONS = 2
GRASS_DILATE_KERNEL = (10, 10)
GRASS_DILATE_ITERATIONS = 2
# GRASS_EDGE_RATIO = 1/3


# The new value chances by K_P times the change from the previous value
NEW_WEIGHT = 0.1

K_P = 0.6
K_D = 0.1
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class ThrottleState:
    REST = 0
    LOW = 1
    HIGH = 2

    def __init__(self):
        self.state = ThrottleState.REST

    def set_state(self, state):
        self.state = state

    def set_on(self, state):
        if self.state != ThrottleState.REST:
            self.state = state

    def to_spi(self):
        if self.state == ThrottleState.REST:
            return 0
        elif self.state == ThrottleState.LOW:
            return THROTTLE_LOW
        elif self.state == ThrottleState.HIGH:
            return THROTTLE_HIGH
        else:
            raise ValueError("Invalid throttle state")
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class GrassDetection:
    """Structure to hold detection information"""

    def __init__(self, cnt, approx, cx, cy, edge_slope, camera_slope, lower_pt, higher_pt):
        self.cnt = cnt # perfect/exact contour
        self.approx = approx # approximated polygon
        self.cx = cx # center x of contour
        self.cy = cy # center y of contour
        self.edge_slope = edge_slope # slope of the track side edge
        self.camera_slope = camera_slope # slope between the bottom point and the bottom center
        self.lower_pt = lower_pt # the lower point of the track side edge
        self.higher_pt = higher_pt # the higher point of the track side edge
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def find_track_contours(frame, grass_thresh):
    """
    Find the track by:
    - Removing portions of the frame that have too much overall color difference
    - Removing portions of the frame that have too much color difference between channels (blue, green, red)
    - Removing portions of the frame that are too bright
    - Removing portions of the frame that are in the horizon (sky)
    - Removing portions of the frame that are too close to the kart
    - Eroding (removing noise)
    - Dilating (filling in holes)
    - Removing the portion of the frame that is grass
    """

    # Split the frame into the three color channels
    b_frame, g_frame, r_frame = cv2.split(frame)
    
    # Calculate the difference between the channels
    bg_diff = cv2.absdiff(b_frame, g_frame)
    gr_diff = cv2.absdiff(g_frame, r_frame)
    rb_diff = cv2.absdiff(r_frame, b_frame)

    # Calculate the total difference
    total_diff = bg_diff + gr_diff + rb_diff

    if SHOW_DEBUG_FRAMES:
        cv2.imshow("total_diff", total_diff)

    # By default, assume everything is track
    track_thresh = np.ones_like(total_diff) * 255

    # Remove: the total difference is too high
    track_thresh[total_diff > TRACK_MAX_TOTAL_DIFF] = 0

    # Remove: the individual differences are too high
    track_thresh[bg_diff > TRACK_MAX_INDIVIDUAL_DIFF] = 0
    track_thresh[gr_diff > TRACK_MAX_INDIVIDUAL_DIFF] = 0
    track_thresh[rb_diff > TRACK_MAX_INDIVIDUAL_DIFF] = 0

    # Remove: the intensity is too high
    track_thresh[b_frame > TRACK_MAX_INTENSITY] = 0
    track_thresh[g_frame > TRACK_MAX_INTENSITY] = 0
    track_thresh[r_frame > TRACK_MAX_INTENSITY] = 0

    # Remove: the horizon (sky)
    horizon_y = int(frame.shape[0] * HORIZON_Y_RATIO)
    track_thresh[:horizon_y] = 0

    # Remove: the kart
    kart_y = int(frame.shape[0] * KART_Y_RATIO)
    track_thresh[kart_y:] = 0

    # Erode to remove noise
    erode_kernel = np.ones(TRACK_ERODE_KERNEL, np.uint8)
    eroded = cv2.erode(track_thresh, erode_kernel, iterations=TRACK_ERODE_ITERATIONS)

    # Dilate to fill in the holes
    dilate_kernel = np.ones(TRACK_DILATE_KERNEL, np.uint8)
    track_thresh = cv2.dilate(eroded, dilate_kernel, iterations=TRACK_DILATE_ITERATIONS)

    # subtract the grass from the track_thresh
    track_thresh = cv2.bitwise_and(track_thresh, cv2.bitwise_not(grass_thresh))

    # Find the contours (outlines) of the track
    contours, _ = cv2.findContours(track_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Return the contours and the track mask
    return contours, track_thresh
# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
def find_grass_contours(frame, old_dilated):
    """
    Find contours by:
    - Converting to hsv
    - Hue thresholding
    - Eroding (removing noise)
    - Dilating (filling in holes)
    - Finding contours
    """

    # hsv to use hue color detection
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # detect grass: yellow/green color
    grass_thresh = cv2.inRange(frame_hsv, GRASS_HSV_LOW, GRASS_HSV_HIGH)

    # Erode to remove noise
    erode_kernel = np.ones(GRASS_ERODE_KERNEL, np.uint8)
    eroded = cv2.erode(grass_thresh, erode_kernel, iterations=GRASS_ERODE_ITERATIONS)

    # Dilate to fill in the holes
    dilate_kernel = np.ones((10, 10), np.uint8)
    grass_thresh = cv2.dilate(eroded, dilate_kernel, iterations=GRASS_DILATE_ITERATIONS)

    # Try to fill in the holes/inconsistencies in the grass mask with the old dilated mask
    total_diated = cv2.bitwise_or(grass_thresh, old_dilated) if old_dilated is not None else grass_thresh

    if SHOW_DEBUG_FRAMES:
        cv2.imshow("dilated", total_diated)

    # Find the outlines from the final mask
    contours, _ = cv2.findContours(total_diated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Outlines, just the grass mask
    return contours, grass_thresh
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def handle_video(fname, vcz):
    """Open video, read frames, process each frame, and display"""

    cap = cv2.VideoCapture(fname)

    history = {
        # For the track centerline based on the grass edges
        "grass": {
            "thresh": None,
            "top": {
                "x": None,
                "y": None
            },
            "bot": {
                "x": None,
                "y": None
            },
            "time": time.time()
        },
        # For the track centerline based on the track itself
        "track": {
            # medians[y] = (x, time), last median x per y for the track detection
            "medians": {},
        },
        # For the track centerline based on YOLOP
        "yolop": {
            "drivable_area": None
        },
        # For the final output target
        "target": {
            "x": None,
            "error": None,
            "time": time.time()
        }
    }

    state = {
        "steering": 0.0,
        "throttle": ThrottleState(),
    }

    model, device, opt = yolop_detect.setup()

    if not vcz:
        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
    else:
        spi = None

    t0 = time.time()

    # i = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error reading frame...")
            break

        # i += 1
        # if i % 10 != 0:
        #     continue

        calc_time_start = time.time()
        marked_frame = image_read(model, device, opt, spi, frame, state, history)
        calc_time_end = time.time()
        calc_time = calc_time_end - calc_time_start

        # Calculate the FPS
        t1 = time.time()
        dt = t1 - t0
        fps = 1 / dt
        t0 = t1

        # Debug drawing: the FPS
        cv2.putText(marked_frame, f"FPS: {fps:.0f} ({calc_time * 1000:.0f} ms)", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Finally, show the frame
        cv2.imshow("Frame", marked_frame)

        # w to pause (and key to unpause), q to quit
        wait_time = max(1, int(1000 / TARGET_FPS) - int(calc_time * 1000))
        key = cv2.waitKey(wait_time) & 0xFF
        if key == ord('w'):
            cv2.waitKey(-1)
        elif key == ord('q'):
            break
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def image_read(model, device, opt, spi, frame, state, history):
    marked_frame = frame.copy()

    # ------------------------------------------------------------------------ #
    # Calculate the bottom center of the frame: used for drawing intersection lines with polygon edges to find the track edges
    bottom_center = (frame.shape[1] // 2, int(frame.shape[0] * BOTTOM_Y_RATIO))
    # Calculate the target y which is a constant distance from the kart
    target_y = int(frame.shape[0] * TARGET_Y_RATIO)

    # Find contours
    grass_contours, grass_thresh = find_grass_contours(frame, history["grass"]["thresh"])
    history["grass"]["thresh"] = grass_thresh

    # Debug drawining
    cv2.drawContours(marked_frame, grass_contours, -1, (0, 255, 0), 1) # draw perfect outlines of grass

    cv2.line(marked_frame, (frame.shape[1] // 2, frame.shape[0]), (frame.shape[1] // 2, 0), (255, 255, 0), 1) # vertical center line
    cv2.line(marked_frame, (0, target_y), (frame.shape[1], target_y), (255, 255, 0), 2) # horizontal target line


    all_detections = []

    for cnt in grass_contours:
        # Approximate the contour to a polygon (find straight edges)
        approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
        cv2.drawContours(marked_frame, [approx], 0, (0, 255, 255), 1) #  draw the approximated polygon

        # Calculate the center of the contour
        center = cv2.moments(cnt)
        if center["m00"] == 0:
            center["m00"] = 1
        cx = int(center["m10"] / center["m00"])
        cy = int(center["m01"] / center["m00"])

        # Throw out all contours that are are in the horizon (sky)
        horizon_y = int(frame.shape[0] * HORIZON_Y_RATIO)
        if cy < horizon_y:
            continue

        # Throw out all contours that are in the kart
        kart_y = int(frame.shape[0] * KART_Y_RATIO)
        if cy > kart_y:
            continue

        # Throw out all contours that are in the center
        # In the center means it is probably not a track edge but trees or other objects
        # left_edge = int(frame.shape[1] * GRASS_EDGE_RATIO)
        # right_edge = int(frame.shape[1] * (1 - GRASS_EDGE_RATIO))
        # if left_edge < cx < right_edge:
        #     continue

        # Debug drawing: center of contour
        cv2.circle(marked_frame, (cx, cy), 4, (0, 0, 255), -1)

        # Find the edge of the polygon that that is the track edge
        # Works by drawing a line from the bottom center to the center of the contour
        # And the closest intersection with the polygon is the edge
        intersection_idx = len(approx) - 1
        intersection_dist = float("inf")
        edge_slope = 0

        for j in range(len(approx)):
            pt1 = approx[j][0]
            i2 = (j + 1) % len(approx) # handle wrap around
            pt2 = approx[i2][0]

            intersection = util.line_intersection(pt1, pt2, bottom_center, (cx, cy))
            if intersection is not None:
                dist = (bottom_center[0] - intersection[0]) ** 2 + (bottom_center[1] - intersection[1]) ** 2
                if dist < intersection_dist:
                    intersection_dist = dist
                    intersection_idx = j

                    dx = pt2[0] - pt1[0]
                    dy = pt2[1] - pt1[1]
                    if dx == 0:
                        edge_slope = 999999
                    else:
                        edge_slope = dy / dx

        # Debug drawing: the track side edge
        i2 = (intersection_idx + 1) % len(approx)
        cv2.line(marked_frame, approx[intersection_idx][0], approx[i2][0], (0, 0, 255), 2)

        # Calculate the lower and higher point of the edge
        # Needed for later, but also using it to pick which contour to track if multiple contours are detected
        pt1 = approx[intersection_idx][0]
        pt2 = approx[i2][0]
        lower_pt = pt1 if pt1[1] > pt2[1] else pt2
        higher_pt = pt1 if pt1[1] < pt2[1] else pt2

        dx = cx - bottom_center[0]
        dy = cy - bottom_center[1]

        # dx = lower_pt[0] - bottom_center[0]
        # dy = lower_pt[1] - bottom_center[1]
        if dx == 0:
            camera_slope = 999999
        else:
            camera_slope = dy / dx

        # Trying to throw out contours that are probably noise
        if (camera_slope < 0 and edge_slope < 0) or (camera_slope > 0 and edge_slope > 0):
            continue
        # if (camera_slope > 0 and cx > frame.shape[1] // 2) or (camera_slope < 0 and cx < frame.shape[1] // 2):
        #     continue

        detection = GrassDetection(cnt, approx, cx, cy, edge_slope, camera_slope, lower_pt, higher_pt)
        all_detections.append(detection)

    # Sort by camera slope (slope between the bottom center and the bottom point of the track edge)
    # Lower number means lower on the screen
    track_detections = sorted(all_detections, key=lambda x: abs(x.camera_slope))

    grass_target_x = None

    # Right now only handling the cases were we know both edges
    if len(track_detections) >= 2:
        edges = [track_detections[0]]

        # Don't pick two edges that are on the same side of the screen
        if edges[0].cx >= frame.shape[1] // 2:
            track_detections = list(filter(lambda x: x.cx < frame.shape[1] // 2, track_detections[1:]))
        else:
            track_detections = list(filter(lambda x: x.cx > frame.shape[1] // 2, track_detections[1:]))
        
        if len(track_detections) > 0: # At least one other detection on the other side
            edges.append(track_detections[0]) # Used [1:] earlier so don't have to worry about repeating

            # Debug drawing: line between the bottom center and the bottom point of the track edge
            for detection in edges:
                # cv2.line(marked_frame, (detection.lower_pt[0], detection.lower_pt[1]), bottom_center, (255, 0, 0), 2)
                cv2.line(marked_frame, (detection.cx, detection.cy), bottom_center, (255, 0, 0), 1)

            # Find the lowest and highest points of the two edges
            lowest_point = edges[0].lower_pt if edges[0].lower_pt[1] > edges[1].lower_pt[1] else edges[1].lower_pt
            highest_point = edges[0].higher_pt if edges[0].higher_pt[1] < edges[1].higher_pt[1] else edges[1].higher_pt

            # Extend the edges so they start and end at the same heights
            edge_0_ext = util.extend_segment(edges[0].lower_pt, edges[0].higher_pt, lowest_point[1], highest_point[1])
            edge_1_ext = util.extend_segment(edges[1].lower_pt, edges[1].higher_pt, lowest_point[1], highest_point[1])

            # Debug drawing: extended edges
            cv2.line(marked_frame, (int(edge_0_ext[0][0]), int(edge_0_ext[0][1])), (int(edge_0_ext[1][0]), int(edge_0_ext[1][1])), (0, 128, 255), 2)
            cv2.line(marked_frame, (int(edge_1_ext[0][0]), int(edge_1_ext[0][1])), (int(edge_1_ext[1][0]), int(edge_1_ext[1][1])), (0, 128, 255), 2)

            # Find the midpoint of the two extended edges which should be the middle of the track
            lower_midpoint = ((int(edge_0_ext[0][0]) + int(edge_1_ext[0][0])) // 2, (int(edge_0_ext[0][1]) + int(edge_1_ext[0][1])) // 2)
            higher_midpoint = ((int(edge_0_ext[1][0]) + int(edge_1_ext[1][0])) // 2, (int(edge_0_ext[1][1]) + int(edge_1_ext[1][1])) // 2)
            cv2.line(marked_frame, lower_midpoint, higher_midpoint, (255, 0, 255), 2) # draw the middle of the track

            # if one of [top, bot][x, y] is None they all are
            if history["grass"]["top"]["x"] is None or time.time()- history["grass"]["time"] > HISTORY_TIME:
                x_top = higher_midpoint[0]
                y_top = higher_midpoint[1]
                x_bot = lower_midpoint[0]
                y_bot = lower_midpoint[1]
            else:
                # Calculate the change in x and y from the last frame
                dx_top = higher_midpoint[0] - history["grass"]["top"]["x"]
                dy_top = higher_midpoint[1] - history["grass"]["top"]["y"]
                dx_bottom = lower_midpoint[0] - history["grass"]["bot"]["x"]
                dy_bottom = lower_midpoint[1] - history["grass"]["bot"]["y"]

                x_top = history["grass"]["top"]["x"] + dx_top * NEW_WEIGHT
                y_top = history["grass"]["top"]["y"] + dy_top * NEW_WEIGHT
                x_bot = history["grass"]["bot"]["x"] + dx_bottom * NEW_WEIGHT
                y_bot = history["grass"]["bot"]["y"] + dy_bottom * NEW_WEIGHT

            history["grass"]["top"]["x"] = x_top
            history["grass"]["top"]["y"] = y_top
            history["grass"]["bot"]["x"] = x_bot
            history["grass"]["bot"]["y"] = y_bot
            history["grass"]["time"] = time.time()

            # Draw the middle of the track
            cv2.line(marked_frame, (int(x_bot), int(y_bot)), (int(x_top), int(y_top)), (0, 0, 0), 4)
            cv2.line(marked_frame, (int(x_bot), int(y_bot)), (int(x_top), int(y_top)), (255, 255, 255), 2)

            # Find the higher low and the lower high point of the two edges
            bot_point = edges[0].lower_pt if edges[0].lower_pt[1] < edges[1].lower_pt[1] else edges[1].lower_pt
            top_point = edges[0].higher_pt if edges[0].higher_pt[1] > edges[1].higher_pt[1] else edges[1].higher_pt
            
            # Shorten the edges so they start and end at the same heights
            edge_0_ext = util.extend_segment(edges[0].lower_pt, edges[0].higher_pt, bot_point[1], top_point[1])
            edge_1_ext = util.extend_segment(edges[1].lower_pt, edges[1].higher_pt, bot_point[1], top_point[1])

            # Debug drawing: shortened edges
            cv2.line(marked_frame, (int(edge_0_ext[0][0]), int(edge_0_ext[0][1])), (int(edge_0_ext[1][0]), int(edge_0_ext[1][1])), (128, 255, 128), 2)
            cv2.line(marked_frame, (int(edge_1_ext[0][0]), int(edge_1_ext[0][1])), (int(edge_1_ext[1][0]), int(edge_1_ext[1][1])), (128, 255, 128), 2)

            # Find the midpoint of the two shortened edges which should be the middle of the track
            # This is on the same line as the longer line, but it is a good debug reference
            lower_midpoint = ((int(edge_0_ext[0][0]) + int(edge_1_ext[0][0])) // 2, (int(edge_0_ext[0][1]) + int(edge_1_ext[0][1])) // 2)
            higher_midpoint = ((int(edge_0_ext[1][0]) + int(edge_1_ext[1][0])) // 2, (int(edge_0_ext[1][1]) + int(edge_1_ext[1][1])) // 2)
            cv2.line(marked_frame, lower_midpoint, higher_midpoint, (255, 0, 128), 2)

            # Calculate (in camera space) where we want to go
            # This is the intersection of the middle of the track and the target line (which is a constant y value or
            #   a constant distance from the kart)
            # fully_extended = util.extend_segment(lower_midpoint, higher_midpoint, 0, frame.shape[0])
            fully_extended = util.extend_segment((x_bot, y_bot), (x_top, y_top), 0, frame.shape[0])
            intersection = util.line_intersection(fully_extended[0], fully_extended[1], (0, target_y), (frame.shape[1], target_y))
            if intersection is not None:
                cv2.circle(marked_frame, (int(intersection[0]), int(intersection[1])), 10, (0, 0, 0), -1)
                cv2.circle(marked_frame, (int(intersection[0]), int(intersection[1])), 9, (255, 255, 255), -1)
                cv2.line(marked_frame, bottom_center, (int(intersection[0]), int(intersection[1])), (255, 255, 255), 2)
                grass_target_x = intersection[0]
    # ------------------------------------------------------------------------ #


    # ------------------------------------------------------------------------ #
    contours, track_thresh = find_track_contours(frame, grass_thresh)

    # Debug drawing: the track mask
    track_colored = np.zeros_like(frame, dtype=np.uint8)
    track_colored[:, :, 1] = track_thresh
    marked_frame = cv2.addWeighted(marked_frame, 1.0, track_colored, 0.5, 0)

    # Find the contours of the track mask
    on_track_pt = (frame.shape[1] // 2, int(frame.shape[0] * ON_TRACK_Y_RATIO))

    # Choose the contour that contains the bottom center (the kart)
    best_cnt = None
    for cnt in contours:
        result = cv2.pointPolygonTest(cnt, on_track_pt, False)
        if result == 1:
            best_cnt = cnt
            break
    # TODO: pick closest contour to the bottom center if no contour contains the point

    # Filter old medians
    now = time.time()
    history["track"]["medians"] = {y: (x, t) for y, (x, t) in history["track"]["medians"].items() if now - t < HISTORY_TIME}

    track_mask = np.zeros_like(track_thresh)

    track_target_x = None

    if best_cnt is not None:
        # Fill in the track mask with only the best contour
        cv2.drawContours(track_mask, [best_cnt], 0, 255, -1)

        # Debug drawing: the track contour
        cv2.drawContours(marked_frame, [best_cnt], 0, (255, 0, 0), 2)

        # For every y, find the median x of the track
        y_coords, x_coords = np.where(track_mask > 0)
        unique_y = np.unique(y_coords)
        medians = {y: np.median(x_coords[y_coords == y]) for y in unique_y}

        # The medians = supposed centerline of the track
        for y, x in medians.items():
            last_x = history["track"]["medians"].get(y, None)
            cv2.circle(marked_frame, (int(x), int(y)), 2, (255, 0, 255), -1)

            if last_x is None:
                history["track"]["medians"][y] = (x, time.time())
            else:
                dx = x - last_x[0]
                x = last_x[0] + dx * NEW_WEIGHT
                history["track"]["medians"][y] = (x, time.time())

                cv2.circle(marked_frame, (int(x), int(y)), 3, (0, 0, 255), -1)

        target_y = frame.shape[0] * TARGET_Y_RATIO
        best_ys = sorted(medians.keys(), key=lambda y: abs(y - target_y))
        target_x = None
        best_y = None
        for y in best_ys:
            target_x, _t = history["track"]["medians"].get(y, (None, None))
            if target_x is not None:
                best_y = y
                break
       
        if target_x is not None:
            cv2.circle(marked_frame, (int(target_x), int(target_y)), 10, (0, 0, 0), -1)
            cv2.circle(marked_frame, (int(target_x), int(best_y)), 9, (0, 255, 255), -1)
            track_target_x = target_x

        # Debug drawing: the point we are checking must be on the track
        cv2.circle(marked_frame, on_track_pt, 4, (0, 255, 255), -1)
    # ------------------------------------------------------------------------ #


    # ------------------------------------------------------------------------ #
    # Integrate the grass and track targets
    if grass_target_x is not None and track_target_x is not None:
        total_x = grass_target_x * GRASS_TARGET_WEIGHT + track_target_x * TRACK_TARGET_WEIGHT
        new_target_x = total_x / (GRASS_TARGET_WEIGHT + TRACK_TARGET_WEIGHT)
    elif grass_target_x is not None:
        new_target_x = grass_target_x
    elif track_target_x is not None:
        new_target_x = track_target_x
    else:
        new_target_x = None
        cv2.rectangle(marked_frame, (0, 0), (frame.shape[1], frame.shape[0]), (0, 0, 255), 10)
    
    target_x = None

    if new_target_x is not None:
        if history["target"]["x"] is None or time.time() - history["target"]["time"] > HISTORY_TIME:
            target_x = new_target_x
            pos_x = target_x
        else:
            dx = new_target_x - history["target"]["x"]

            target_x = history["target"]["x"] + dx * NEW_WEIGHT

        # Convert the target_x to degrees
        x_ratio = target_x / frame.shape[1]
        error_x = x_ratio - 0.5
        scaled_error_x = util.scale(error_x, -0.5, 0.5, -CAMERA_FOV_SCALE_FACTOR, CAMERA_FOV_SCALE_FACTOR)
        angle_dif_rad_x = math.atan(scaled_error_x)
        angle_dif_x = util.rad_to_deg(angle_dif_rad_x)

        if history["target"]["error"] is not None:
            d_error_x = (angle_dif_x - history["target"]["error"]) / (time.time() - history["target"]["time"])
        else:
            d_error_x = 0.0
        
        history["target"]["error"] = angle_dif_x

        total_error_x = K_P * angle_dif_x + K_D * d_error_x

        # Convert back to pixel space for debug drawing
        angle_dif_rad_x = util.deg_to_rad(total_error_x)
        pos_dif_scaled_x = math.tan(angle_dif_rad_x)
        pos_dif_x = util.scale(pos_dif_scaled_x, -CAMERA_FOV_SCALE_FACTOR, CAMERA_FOV_SCALE_FACTOR, -0.5, 0.5)
        pos_x = int(frame.shape[1] * (pos_dif_x + 0.5))
        
        history["target"]["x"] = target_x
        history["target"]["time"] = time.time()
    
    if target_x is not None:
        # Debug drawing: the target point
        cv2.circle(marked_frame, (int(new_target_x), int(target_y)), 13, (0, 0, 0), -1)
        cv2.circle(marked_frame, (int(target_x), int(target_y)), 12, (255, 0, 255), -1)
        cv2.circle(marked_frame, (int(pos_x), int(target_y)), 10, (255, 255, 0), -1)

        state["steering"] = total_error_x
        state["throttle"].set_on(ThrottleState.HIGH)
        
    else:
        state["steering"] = state["steering"] * DECAY_RATE * (time.time() - history["target"]["time"])
        state["throttle"].set_on(ThrottleState.LOW)
    
    # Sent steering and throttle over SPI to the Nucleo
    if state["throttle"].state == ThrottleState.REST and spi is not None:
        state["steering"] = 0.0
    
    spi_angle = util.scale(abs(state["steering"] ), 0, CAMERA_FOV, 0, SPI_MAX)
    spi_angle = int(spi_angle)
    spi_sign = 1 if state["steering"]  < 0 else 0
    spi_data = [spi_sign, spi_angle, state["throttle"].to_spi()]
    if spi is not None:
        spi.xfer2(spi_data)
    
    print(state["steering"], spi_data)
    # ------------------------------------------------------------------------ #


    # ------------------------------------------------------------------------ #
    # # Run YOLOP inference and convert the result to a compatible mask
    # drivable_area = yolop_detect.run_detection(model, device, opt, frame)
    # drivable_area = cv2.resize(drivable_area, (frame.shape[1], frame.shape[0]))
    # drivable_area = drivable_area.astype(np.uint8) * 255

    # if history["yolop"]["drivable_area"] is not None:
    #     total_drivable_area = cv2.bitwise_or(drivable_area, history["yolop"]["drivable_area"])
    # else:
    #     total_drivable_area = drivable_area
    # history["yolop"]["drivable_area"] = drivable_area

    # # Debug drawing: yolop detection
    # track_colored = np.zeros_like(frame, dtype=np.uint8)
    # track_colored[:, :, 2] = total_drivable_area
    # marked_frame = cv2.addWeighted(marked_frame, 1.0, track_colored, 1.0, 0)
    # ------------------------------------------------------------------------ #

    # Return the frame with all the debug drawings
    return marked_frame
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--file", required=True, help="open file")
    ap.add_argument("-z", "--video-capture-zero", required=False, help="testing version", action="store_true")
    args = vars(ap.parse_args())
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    # image_read(args["file"])
    handle_video(args["file"], args["video_capture_zero"])
    # ------------------------------------------------------------------------ #


    # ------------------------------------------------------------------------ #
    print("DONE...")
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
if __name__ == "__main__":
    main()