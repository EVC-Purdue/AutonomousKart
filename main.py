print("STARTING...")

import argparse

import time
import cv2
import numpy as np
import math

import util





CAMERA_WAIT_AFTER_ACTIVE = 1.0
CAP_ARGS = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink"




# def camera_loop(video_capture_zero):
#     print(f"Opening camera...")
#     time.sleep(CAMERA_WAIT_AFTER_ACTIVE)

#     if video_capture_zero:
#         cap = cv2.VideoCapture(0)
#     else:
#         cap = cv2.VideoCapture(CAP_ARGS, cv2.CAP_GSTREAMER)

#     print(f"Camera opened...")

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Error reading frame...")
#             break

#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
#         edges = cv2.Canny(gray, 175, 200) 

#         kernel = np.ones((5, 5), np.uint8)
        
        # edges = cv2.dilate(edges, kernel, iterations=2)

#         cv2.imshow("gray", gray)
#         cv2.imshow("edges", edges)
        
        
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break


class Detection:
    """Structure to hold detection information"""

    def __init__(self, id, cnt, approx, cx, cy, max_dist_idx, slope, max_slope, lower_pt):
        self.id = id
        self.cnt = cnt
        self.approx = approx
        self.cx = cx
        self.cy = cy
        self.max_dist_idx = max_dist_idx
        self.slope = slope
        self.max_slope = max_slope
        self.lower_pt = lower_pt

    


def find_contours(frame):
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
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours


def handle_video(fname):
    cap = cv2.VideoCapture(fname)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame...")
            break

        image_read(frame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break


def image_read(frame):
# def image_read(fname):
    # frame = cv2.imread(fname)

    bottom_center = (frame.shape[1] // 2, frame.shape[0])

    contours = find_contours(frame)

    # blank_frame = np.zeros(frame.shape, np.uint8)

    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
    # cv2.drawContours(blank_frame, contours, -1, (255, 255, 255), 2)



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
        max_slope = 0

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
                        slope = 999999
                    else:
                        slope = dy / dx
                    max_slope = slope

        
        i2 = (intersection_idx + 1) % len(approx)
        cv2.line(frame, approx[intersection_idx][0], approx[i2][0], (0, 0, 255), 5)

        pt1 = approx[intersection_idx][0]
        pt2 = approx[i2][0]
        lower_pt = pt1 if pt1[1] > pt2[1] else pt2

        dx = lower_pt[0] - bottom_center[0]
        dy = lower_pt[1] - bottom_center[1]
        if dx == 0:
            slope = 999999
        else:
            slope = dy / dx

        # cv2.line(frame, (lower_y_pt[0], lower_y_pt[1]), bottom_center, (255, 0, 0), 5)


        detection = Detection(i, cnt, approx, cx, cy, intersection_idx, slope, max_slope, lower_pt)
        all_detections.append(detection)


    track_detections = sorted(all_detections, key=lambda x: abs(x.slope))
    track_detections = track_detections[:min(2, len(track_detections))]

    for detection in track_detections:
        cv2.line(frame, (detection.lower_pt[0], detection.lower_pt[1]), bottom_center, (255, 0, 0), 5)

    track_detections = sorted(track_detections, key=lambda x: abs(x.max_slope))


    if len(track_detections) > 0:
        closest_detection = track_detections[0]

        cv2.line(frame, (closest_detection.lower_pt[0], closest_detection.lower_pt[1]), bottom_center, (255, 255, 0), 1)


    cv2.imshow("frame", frame)
    # # cv2.imshow("blank", blank_frame)

    # if cv2.waitKey(0) & 0xFF == ord('q'):
    #     pass


def main():
    ap = argparse.ArgumentParser()
    # ap.add_argument("-z", "--video-capture-zero", required=False, help="testing version", action="store_true")
    ap.add_argument("-f", "--file", required=True, help="open file")
    args = vars(ap.parse_args())
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    # camera_loop(args["video_capture_zero"])

    # image_read(args["file"])
    handle_video(args["file"])
    # ------------------------------------------------------------------------ #

    print("\n\nExiting...\n\n")

    # ------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------ #

    print("DONE...")
     


if __name__ == "__main__":
    main()