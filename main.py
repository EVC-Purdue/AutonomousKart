print("STARTING...")

import argparse

import cv2
import numpy as np
import math

import util
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
SHOW_DEBUG_FRAMES = False


BOTTOM_RATIO = 18/20
TARGET_Y_RATIO = 6/10

K_P = 0.1
# K_D = 0.1
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class Detection:
    """Structure to hold detection information"""

    def __init__(self, cnt, approx, cx, cy, edge_slope, camera_slope, lower_pt, higher_pt):
        self.cnt = cnt # perfect/exact contour
        self.approx = approx # approximated polygon
        self.cx = cx # center x of contour
        self.cy = cy # center y of contour
        # self.line_start_idx = line_start_idx # index in self.approx of the start of the track side edge
        # self.line_end_idx = (line_start_idx + 1) % len(approx) # index in self.approx of the end of the track side edge
        self.edge_slope = edge_slope # slope of the track side edge
        self.camera_slope = camera_slope # slope between the bottom point and the bottom center
        self.lower_pt = lower_pt # the lower point of the track side edge
        self.higher_pt = higher_pt # the higher point of the track side edge
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def find_track_thresh(frame):
    """
    Find the track by:
    - Removing portions of the frame that are too bright
    - Removing portions of the frame that have too much color difference
    - Removing portions of the frame that have too much color difference between channels
    - Eroding (removing noise)
    - Dilating (filling in holes)
    """

    MAX_TOTAL_DIFF = 85
    MAX_INDIVIDUAL_DIFF = 45
    MAX_INTENSITY = 200

    b_frame, g_frame, r_frame = cv2.split(frame)
    total_diff = cv2.absdiff(r_frame, g_frame) + cv2.absdiff(g_frame, b_frame) + cv2.absdiff(b_frame, r_frame)

    bg_diff = cv2.absdiff(b_frame, g_frame)
    gr_diff = cv2.absdiff(g_frame, r_frame)
    rb_diff = cv2.absdiff(r_frame, b_frame)
    total_diff = bg_diff + gr_diff + rb_diff

    if SHOW_DEBUG_FRAMES:
        cv2.imshow("total_diff", total_diff)

    track_thresh = np.ones_like(total_diff) * 255

    track_thresh[total_diff > MAX_TOTAL_DIFF] = 0

    track_thresh[bg_diff > MAX_INDIVIDUAL_DIFF] = 0
    track_thresh[gr_diff > MAX_INDIVIDUAL_DIFF] = 0
    track_thresh[rb_diff > MAX_INDIVIDUAL_DIFF] = 0

    track_thresh[b_frame > MAX_INTENSITY] = 0
    track_thresh[g_frame > MAX_INTENSITY] = 0
    track_thresh[r_frame > MAX_INTENSITY] = 0

    erode_kernel = np.ones((5, 5), np.uint8)
    eroded = cv2.erode(track_thresh, erode_kernel, iterations=5)

    dilate_kernel = np.ones((3, 3), np.uint8)
    track_thresh = cv2.dilate(eroded, dilate_kernel, iterations=7)

    return track_thresh


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
    grass_thresh = cv2.inRange(frame_hsv, (25, 40, 40), (80, 255,255))

    # Erode to remove noise
    erode_kernel = np.ones((7, 7), np.uint8)
    eroded = cv2.erode(grass_thresh, erode_kernel, iterations=2)

    # Dilate to fill in the holes
    dilate_kernel = np.ones((10, 10), np.uint8)
    grass_thresh = cv2.dilate(eroded, dilate_kernel, iterations=2)

    # Try to fill in the holes/inconsistencies in the grass mask with the old dilated mask
    total_diated = cv2.bitwise_or(grass_thresh, old_dilated) if old_dilated is not None else grass_thresh

    if SHOW_DEBUG_FRAMES:
        cv2.imshow("dilated", total_diated)

    # Find the outlines from the final mask
    contours, _ = cv2.findContours(total_diated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Outlines, just the grass mask, and the mask with the grass and the old dilated mask
    return contours, grass_thresh, total_diated


def handle_video(fname):
    """Open video, read frames, process each frame, and display"""

    cap = cv2.VideoCapture(fname)

    history = {
        # Will combine last dilated mask with the current to improve detection
        "last_dilated": None,
        
        # For the track centerline based on the grass edges
        "last_top_x": None,
        "last_top_y": None,
        "last_bottom_x": None,
        "last_bottom_y": None,

        # last_medians[y] = x, last median x per y for the track detection
        "last_medians": {}
    }

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error reading frame...")
            break

        # Do this before we mark up frame
        track_thresh = find_track_thresh(frame)

        image_read(frame, history)

        # subtract the grass from the track_thresh
        track_thresh = cv2.bitwise_and(track_thresh, cv2.bitwise_not(history["last_dilated"]))

        # Overlay: merge the track mask as green using an alpha value
        track_colored = np.zeros_like(frame, dtype=np.uint8)
        track_colored[:, :, 1] = track_thresh
        overlay = cv2.addWeighted(frame, 1.0, track_colored, 0.5, 0)

        # Find the contours of the track mask
        track_mask = np.zeros_like(frame[:, :, 0], dtype=np.uint8)
        contours, _ = cv2.findContours(track_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bottom_center = (frame.shape[1] // 2, int(frame.shape[0] * BOTTOM_RATIO))

        # Choose the contour that contains the bottom center (the kart)
        best_cnt = None
        for cnt in contours:
            result = cv2.pointPolygonTest(cnt, bottom_center, False)
            if result == 1:
                best_cnt = cnt
                break
        # TODO: pick closest contour to the bottom center


        if best_cnt is not None:
            # Debug drawing: the track contour
            cv2.drawContours(track_mask, [best_cnt], 0, 255, -1)
            cv2.drawContours(overlay, [best_cnt], 0, (255, 0, 0), 2)

            # For every y, find the median x of the track
            y_coords, x_coords = np.where(track_mask > 0)
            unique_y = np.unique(y_coords)
            medians = {y: np.median(x_coords[y_coords == y]) for y in unique_y}

            for y, x in medians.items():
                last_x = history["last_medians"].get(y, None)
                cv2.circle(overlay, (int(x), int(y)), 3, (255, 0, 255), -1)

                if last_x is None:
                    history["last_medians"][y] = x
                else:
                    dx = x - last_x
                    x = last_x + dx * K_P
                    history["last_medians"][y] = x

                    cv2.circle(overlay, (int(x), int(y)), 4, (0, 0, 255), -1)


        cv2.circle(overlay, bottom_center, 4, (0, 255, 255), -1)

        cv2.imshow("Frame", overlay)


        # w to pause (and key to unpause), q to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('w'):
            cv2.waitKey(-1)
        elif key == ord('q'):
            break


def image_read(frame, history):
    target_x = None

    # Calculate the bottom center of the frame: used for drawing intersection lines with polygon edges to find the track edges
    bottom_center = (frame.shape[1] // 2, int(frame.shape[0] * BOTTOM_RATIO))
    # Calculate the target y which is a constant distance from the kart
    target_y = int(frame.shape[0] * TARGET_Y_RATIO)

    # Find contours
    contours, dilated, _total_diated = find_grass_contours(frame, history["last_dilated"])
    history["last_dilated"] = dilated

    # Debug drawining
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 1) # draw perfect outlines of grass

    cv2.line(frame, (frame.shape[1] // 2, frame.shape[0]), (frame.shape[1] // 2, 0), (255, 255, 0), 1) # vertical center line
    cv2.line(frame, (0, target_y), (frame.shape[1], target_y), (255, 255, 0), 2) # horizontal target line


    all_detections = []

    for cnt in contours:
        # Approximate the contour to a polygon (find straight edges)
        approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
        cv2.drawContours(frame, [approx], 0, (0, 255, 255), 1) #  draw the approximated polygon

        # Calculate the center of the contour
        center = cv2.moments(cnt)
        if center["m00"] == 0:
            center["m00"] = 1
        cx = int(center["m10"] / center["m00"])
        cy = int(center["m01"] / center["m00"])

        # Throw out all contours that are in the center
        # In the center means it is probably not a track edge but trees or other objects
        left_33 = int(frame.shape[1] * (1/3))
        right_33 = int(frame.shape[1] * (2/3))
        if left_33 < cx < right_33:
            continue

        # Debug drawing: center of contour
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)


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
                dist = math.dist(bottom_center, intersection)
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
        cv2.line(frame, approx[intersection_idx][0], approx[i2][0], (0, 0, 255), 2)

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

        detection = Detection(cnt, approx, cx, cy, edge_slope, camera_slope, lower_pt, higher_pt)
        all_detections.append(detection)

    # Sort by camera slope (slope between the bottom center and the bottom point of the track edge)
    # Lower number means lower on the screen
    track_detections = sorted(all_detections, key=lambda x: abs(x.camera_slope))

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
                cv2.line(frame, (detection.lower_pt[0], detection.lower_pt[1]), bottom_center, (255, 0, 0), 2)
                # cv2.line(frame, (detection.cx, detection.cy), bottom_center, (255, 0, 0), 1)

            # Find the lowest and highest points of the two edges
            lowest_point = edges[0].lower_pt if edges[0].lower_pt[1] > edges[1].lower_pt[1] else edges[1].lower_pt
            highest_point = edges[0].higher_pt if edges[0].higher_pt[1] < edges[1].higher_pt[1] else edges[1].higher_pt

            # Extend the edges so they start and end at the same heights
            edge_0_ext = util.extend_segment(edges[0].lower_pt, edges[0].higher_pt, lowest_point[1], highest_point[1])
            edge_1_ext = util.extend_segment(edges[1].lower_pt, edges[1].higher_pt, lowest_point[1], highest_point[1])

            # Debug drawing: extended edges
            cv2.line(frame, (int(edge_0_ext[0][0]), int(edge_0_ext[0][1])), (int(edge_0_ext[1][0]), int(edge_0_ext[1][1])), (0, 128, 255), 2)
            cv2.line(frame, (int(edge_1_ext[0][0]), int(edge_1_ext[0][1])), (int(edge_1_ext[1][0]), int(edge_1_ext[1][1])), (0, 128, 255), 2)

            # Find the midpoint of the two extended edges which should be the middle of the track
            lower_midpoint = ((int(edge_0_ext[0][0]) + int(edge_1_ext[0][0])) // 2, (int(edge_0_ext[0][1]) + int(edge_1_ext[0][1])) // 2)
            higher_midpoint = ((int(edge_0_ext[1][0]) + int(edge_1_ext[1][0])) // 2, (int(edge_0_ext[1][1]) + int(edge_1_ext[1][1])) // 2)
            cv2.line(frame, lower_midpoint, higher_midpoint, (255, 0, 255), 2) # draw the middle of the track

            if history["last_top_x"] is None:
                history["last_top_x"] = higher_midpoint[0]
                history["last_top_y"] = higher_midpoint[1]
                history["last_bottom_x"] = lower_midpoint[0]
                history["last_bottom_y"] = lower_midpoint[1]

                x_top = history["last_top_x"]
                y_top = history["last_top_y"]
                x_bottom = history["last_bottom_x"]
                y_bottom = history["last_bottom_y"]
            else:
                # Calculate the change in x and y from the last frame
                dx_top = higher_midpoint[0] - history["last_top_x"]
                dy_top = higher_midpoint[1] - history["last_top_y"]
                dx_bottom = lower_midpoint[0] - history["last_bottom_x"]
                dy_bottom = lower_midpoint[1] - history["last_bottom_y"]

                x_top = history["last_top_x"] + dx_top * K_P
                y_top = history["last_top_y"] + dy_top * K_P
                x_bottom = history["last_bottom_x"] + dx_bottom * K_P
                y_bottom = history["last_bottom_y"] + dy_bottom * K_P

                history["last_top_x"] = x_top
                history["last_top_y"] = y_top
                history["last_bottom_x"] = x_bottom
                history["last_bottom_y"] = y_bottom

                # Draw the middle of the track
                cv2.line(frame, (int(x_bottom), int(y_bottom)), (int(x_top), int(y_top)), (0, 0, 0), 4)
                cv2.line(frame, (int(x_bottom), int(y_bottom)), (int(x_top), int(y_top)), (255, 255, 255), 2)

            # Find the higher low and the lower high point of the two edges
            bot_point = edges[0].lower_pt if edges[0].lower_pt[1] < edges[1].lower_pt[1] else edges[1].lower_pt
            top_point = edges[0].higher_pt if edges[0].higher_pt[1] > edges[1].higher_pt[1] else edges[1].higher_pt
            
            # Shorten the edges so they start and end at the same heights
            edge_0_ext = util.extend_segment(edges[0].lower_pt, edges[0].higher_pt, bot_point[1], top_point[1])
            edge_1_ext = util.extend_segment(edges[1].lower_pt, edges[1].higher_pt, bot_point[1], top_point[1])

            # Debug drawing: shortened edges
            cv2.line(frame, (int(edge_0_ext[0][0]), int(edge_0_ext[0][1])), (int(edge_0_ext[1][0]), int(edge_0_ext[1][1])), (128, 255, 128), 2)
            cv2.line(frame, (int(edge_1_ext[0][0]), int(edge_1_ext[0][1])), (int(edge_1_ext[1][0]), int(edge_1_ext[1][1])), (128, 255, 128), 2)

            # Find the midpoint of the two shortened edges which should be the middle of the track
            # This is on the same line as the longer line, but it is a good debug reference
            lower_midpoint = ((int(edge_0_ext[0][0]) + int(edge_1_ext[0][0])) // 2, (int(edge_0_ext[0][1]) + int(edge_1_ext[0][1])) // 2)
            higher_midpoint = ((int(edge_0_ext[1][0]) + int(edge_1_ext[1][0])) // 2, (int(edge_0_ext[1][1]) + int(edge_1_ext[1][1])) // 2)
            cv2.line(frame, lower_midpoint, higher_midpoint, (255, 0, 128), 2)

            # Calculate (in camera space) where we want to go
            # This is the intersection of the middle of the track and the target line (which is a constant y value or
            #   a constant distance from the kart)
            # fully_extended = util.extend_segment(lower_midpoint, higher_midpoint, 0, frame.shape[0])
            fully_extended = util.extend_segment((x_bottom, y_bottom), (x_top, y_top), 0, frame.shape[0])
            intersection = util.line_intersection(fully_extended[0], fully_extended[1], (0, target_y), (frame.shape[1], target_y))
            if intersection is not None:
                cv2.circle(frame, (int(intersection[0]), int(intersection[1])), 10, (0, 0, 0), -1)
                cv2.circle(frame, (int(intersection[0]), int(intersection[1])), 9, (255, 255, 255), -1)
                cv2.line(frame, bottom_center, (int(intersection[0]), int(intersection[1])), (255, 255, 255), 2)

        else:
            print("DETECTIONS ON SAME SIDE")
            cv2.rectangle(frame, (0, 0), (frame.shape[1], frame.shape[0]), (128, 0, 255), 4)
    else:
        print(f"DETECTION LOST: {len(track_detections)}")
        cv2.rectangle(frame, (0, 0), (frame.shape[1], frame.shape[0]), (0, 0, 255), 4)

    # Return the dilated mask for the next frame
    return dilated, target_x

    # if cv2.waitKey(0) & 0xFF == ord('q'):
    #     pass
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--file", required=True, help="open file")
    args = vars(ap.parse_args())
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    # image_read(args["file"])
    handle_video(args["file"])
    # ------------------------------------------------------------------------ #


    # ------------------------------------------------------------------------ #
    print("DONE...")
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
if __name__ == "__main__":
    main()