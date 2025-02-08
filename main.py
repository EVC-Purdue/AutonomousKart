print("STARTING...")

import argparse

import cv2
import numpy as np
import math

import util
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
BOTTOM_RATIO = 9/10
TARGET_Y_RATIO = 6/10
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class Detection:
    """Structure to hold detection information"""

    def __init__(self, id, cnt, approx, cx, cy, line_start_idx, edge_slope, camera_slope, lower_pt, higher_pt):
        self.id = id
        self.cnt = cnt
        self.approx = approx
        self.cx = cx
        self.cy = cy
        self.line_start_idx = line_start_idx
        self.line_end_idx = (line_start_idx + 1) % len(approx)
        self.edge_slope = edge_slope
        self.camera_slope = camera_slope
        self.lower_pt = lower_pt
        self.higher_pt = higher_pt
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def find_contours(frame, old_dilated):
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
    frame_threshold = cv2.inRange(frame_hsv, (25, 40, 40), (80, 255,255))

    # Erode to remove noise
    erode_kernel = np.ones((7, 7), np.uint8)
    eroded = cv2.erode(frame_threshold, erode_kernel, iterations=2)

    # Dilate to fill in the holes
    dilate_kernel = np.ones((10, 10), np.uint8)
    dilated = cv2.dilate(eroded, dilate_kernel, iterations=2)


    # edges = cv2.Canny(dilated, 50, 150)
    # cv2.imshow("dilated", dilated)

    total_diated = cv2.bitwise_or(dilated, old_dilated) if old_dilated is not None else dilated
    cv2.imshow("dilated", total_diated)

    contours, _ = cv2.findContours(total_diated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours, dilated, total_diated


def handle_video(fname):
    cap = cv2.VideoCapture(fname)

    last_dilated = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame...")
            break

        dilated = image_read(frame, last_dilated)
        last_dilated = dilated

        # if cv2.waitKey(33) & 0xFF == ord('q'):
        #     break
        # w to pause
        key = cv2.waitKey(33) & 0xFF
        if key == ord('w'):
            cv2.waitKey(-1)
        elif key == ord('q'):
            break


def image_read(frame, old_dilated):
# def image_read(fname):
    # frame = cv2.imread(fname)

    bottom_center = (frame.shape[1] // 2, int(frame.shape[0] * BOTTOM_RATIO))
    target_y = int(frame.shape[0] * TARGET_Y_RATIO)

    contours, dilated, total_diated = find_contours(frame, old_dilated)
    frame = cv2.bitwise_and(frame, frame, mask=total_diated)


    cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)

    cv2.line(frame, (frame.shape[1] // 2, frame.shape[0]), (frame.shape[1] // 2, 0), (255, 255, 0), 1)
    cv2.line(frame, (0, target_y), (frame.shape[1], target_y), (255, 255, 0), 2)


    all_detections = []

    # for cnt in contours:
    for i, cnt in enumerate(contours):
        # x, y, w, h = cv2.boundingRect(cnt)
        approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
        cv2.drawContours(frame, [approx], 0, (0, 255, 255), 2)

        center = cv2.moments(cnt)
        if center["m00"] == 0:
            center["m00"] = 1
        cx = int(center["m10"] / center["m00"])
        cy = int(center["m01"] / center["m00"])

        left_33 = int(frame.shape[1] * (1/3))
        right_33 = int(frame.shape[1] * (2/3))
        if left_33 < cx < right_33:
            continue


        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        intersection_idx = len(approx) - 1
        intersection_dist = float("inf")
        edge_slope = 0

        for j in range(len(approx)):
            pt1 = approx[j][0]
            i2 = (j + 1) % len(approx)
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

        
        i2 = (intersection_idx + 1) % len(approx)
        cv2.line(frame, approx[intersection_idx][0], approx[i2][0], (0, 0, 255), 3)

        pt1 = approx[intersection_idx][0]
        pt2 = approx[i2][0]
        lower_pt = pt1 if pt1[1] > pt2[1] else pt2
        higher_pt = pt1 if pt1[1] < pt2[1] else pt2

        dx = lower_pt[0] - bottom_center[0]
        dy = lower_pt[1] - bottom_center[1]
        if dx == 0:
            camera_slope = 999999
        else:
            camera_slope = dy / dx

        # if (camera_slope < 0 and edge_slope < 0) or (camera_slope > 0 and edge_slope > 0):
        #     continue
        # if (camera_slope > 0 and cx > frame.shape[1] // 2) or (camera_slope < 0 and cx < frame.shape[1] // 2):
        #     continue

        detection = Detection(i, cnt, approx, cx, cy, intersection_idx, edge_slope, camera_slope, lower_pt, higher_pt)
        all_detections.append(detection)


    track_detections = sorted(all_detections, key=lambda x: abs(x.camera_slope))

    if len(track_detections) >= 2:
        # track_detections = track_detections[:min(2, len(track_detections))]
        edges = [track_detections[0]]
        if edges[0].cx >= frame.shape[1] // 2:
            track_detections = list(filter(lambda x: x.cx < frame.shape[1] // 2, track_detections[1:]))
        else:
            track_detections = list(filter(lambda x: x.cx > frame.shape[1] // 2, track_detections[1:]))
        
        if len(track_detections) > 0:
            edges.append(track_detections[0])

            for detection in edges:
                # cv2.line(frame, (detection.lower_pt[0], detection.lower_pt[1]), bottom_center, (255, 0, 0), 2)
                cv2.line(frame, (detection.cx, detection.cy), bottom_center, (255, 0, 0), 1)

            # edges = sorted(edges, key=lambda x: abs(x.edge_slope), reverse=True)
            # closest_detection = edges[0]
            # cv2.line(frame, (closest_detection.lower_pt[0], closest_detection.lower_pt[1]), bottom_center, (255, 100, 0), 3)

            lowest_point = edges[0].lower_pt if edges[0].lower_pt[1] > edges[1].lower_pt[1] else edges[1].lower_pt
            highest_point = edges[0].higher_pt if edges[0].higher_pt[1] < edges[1].higher_pt[1] else edges[1].higher_pt

            edge_0_ext = util.extend_segment(edges[0].lower_pt, edges[0].higher_pt, lowest_point[1], highest_point[1])
            edge_1_ext = util.extend_segment(edges[1].lower_pt, edges[1].higher_pt, lowest_point[1], highest_point[1])

            cv2.line(frame, (int(edge_0_ext[0][0]), int(edge_0_ext[0][1])), (int(edge_0_ext[1][0]), int(edge_0_ext[1][1])), (0, 128, 255), 2)
            cv2.line(frame, (int(edge_1_ext[0][0]), int(edge_1_ext[0][1])), (int(edge_1_ext[1][0]), int(edge_1_ext[1][1])), (0, 128, 255), 2)


            lower_midpoint = ((int(edge_0_ext[0][0]) + int(edge_1_ext[0][0])) // 2, (int(edge_0_ext[0][1]) + int(edge_1_ext[0][1])) // 2)
            higher_midpoint = ((int(edge_0_ext[1][0]) + int(edge_1_ext[1][0])) // 2, (int(edge_0_ext[1][1]) + int(edge_1_ext[1][1])) // 2)
            cv2.line(frame, lower_midpoint, higher_midpoint, (255, 0, 255), 4)


            bot_point = edges[0].lower_pt if edges[0].lower_pt[1] < edges[1].lower_pt[1] else edges[1].lower_pt
            top_point = edges[0].higher_pt if edges[0].higher_pt[1] > edges[1].higher_pt[1] else edges[1].higher_pt

            edge_0_ext = util.extend_segment(edges[0].lower_pt, edges[0].higher_pt, bot_point[1], top_point[1])
            edge_1_ext = util.extend_segment(edges[1].lower_pt, edges[1].higher_pt, bot_point[1], top_point[1])

            lower_midpoint = ((int(edge_0_ext[0][0]) + int(edge_1_ext[0][0])) // 2, (int(edge_0_ext[0][1]) + int(edge_1_ext[0][1])) // 2)
            higher_midpoint = ((int(edge_0_ext[1][0]) + int(edge_1_ext[1][0])) // 2, (int(edge_0_ext[1][1]) + int(edge_1_ext[1][1])) // 2)
            cv2.line(frame, lower_midpoint, higher_midpoint, (255, 0, 128), 4)


            fully_extended = util.extend_segment(lower_midpoint, higher_midpoint, 0, frame.shape[0])
            intersection = util.line_intersection(fully_extended[0], fully_extended[1], (0, target_y), (frame.shape[1], target_y))
            if intersection is not None:
                cv2.circle(frame, (int(intersection[0]), int(intersection[1])), 9, (255, 255, 255), -1)
                cv2.line(frame, bottom_center, (int(intersection[0]), int(intersection[1])), (255, 255, 255), 2)
        else:
            print("DETECTIONS ON SAME SIDE")
    else:
        print(f"DETECTION LOST: {len(track_detections)}")


    cv2.imshow("frame", frame)
    return dilated

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