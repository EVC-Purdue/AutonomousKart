print("STARTING...")

import argparse

import time
import cv2
import numpy as np
import math





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

    # detect: track: grey and black
    # frame_threshold_1 = cv2.inRange(frame_hsv, (0, 10, 25), (25, 255, 150))
    # frame_threshold_2 = cv2.inRange(frame_hsv, (80, 0, 0), (180, 255, 150))
    # frame_threshold = cv2.bitwise_or(frame_threshold_1, frame_threshold_2)

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

        max_dist_idx = len(approx) - 1
        max_dist_sq = 0
        max_slope = 0

        for i in range(len(approx)):
            pt1 = approx[i]
            i2 = (i + 1) % len(approx)
            pt2 = approx[i2]

            x1, y1 = pt1[0]
            x2, y2 = pt2[0]

            dist_sq = (x2 - x1)**2 + (y2 - y1)**2
            if dist_sq >= max_dist_sq:
                max_dist_idx = i
                max_dist_sq = dist_sq
                dx = x2 - x1
                dy = y2 - y1
                if dx == 0:
                    slope = 999999
                else:
                    slope = dy / dx
                max_slope = slope

        
        i2 = (max_dist_idx + 1) % len(approx)
        cv2.line(frame, approx[max_dist_idx][0], approx[i2][0], (0, 0, 255), 5)

        pt1 = approx[max_dist_idx][0]
        pt2 = approx[i2][0]
        lower_pt = pt1 if pt1[1] > pt2[1] else pt2

        dx = lower_pt[0] - bottom_center[0]
        dy = lower_pt[1] - bottom_center[1]
        if dx == 0:
            slope = 999999
        else:
            slope = dy / dx

        # cv2.line(frame, (lower_y_pt[0], lower_y_pt[1]), bottom_center, (255, 0, 0), 5)


        detection = Detection(i, cnt, approx, cx, cy, max_dist_idx, slope, max_slope, lower_pt)
        all_detections.append(detection)


    # all_detections.sort(key=lambda x: abs(x.slope))
    # for detection in all_detections[:min(2, len(all_detections))]:
    #     # cv2.drawContours(frame, [detection.approx], 0, (255, 0, 0), 2)
    #     # cv2.circle(frame, (detection.cx, detection.cy), 5, (255, 0, 0), -1)
    #     # cv2.line(frame, (detection.cx, detection.cy), bottom_center, (255, 0, 0), 5)
    #     cv2.line(frame, (detection.lower_pt[0], detection.lower_pt[1]), bottom_center, (255, 0, 0), 5)

    #     dist_pt1 = detection.approx[detection.max_dist_idx][0]
    #     dist_pt2 = detection.approx[(detection.max_dist_idx + 1) % len(detection.approx)][0]

    track_detections = sorted(all_detections, key=lambda x: abs(x.slope))
    track_detections = track_detections[:min(2, len(track_detections))]

    for detection in track_detections:
        cv2.line(frame, (detection.lower_pt[0], detection.lower_pt[1]), bottom_center, (255, 0, 0), 5)

    track_detections = sorted(track_detections, key=lambda x: abs(x.max_slope))


    if len(track_detections) > 0:
        closest_detection = track_detections[0]

        cv2.line(frame, (closest_detection.lower_pt[0], closest_detection.lower_pt[1]), bottom_center, (255, 255, 0), 2)






    # for detection in all_detections:
    #     pt1 = detection.approx[detection.max_dist_idx]
    #     pt2 = detection.approx[(detection.max_dist_idx + 1) % len(detection.approx)]
    #     # x1, y1 = pt1[0]
    #     # x2, y2 = pt2[0]
    #     # mx, my = (x1 + x2) // 2, (y1 + y2) // 2
    #     # cv2.line(frame, (mx, my), BOTTOM_CENTER, (255, 0, 0), 5)
    #     lower_y_pt = pt1 if pt1[0][1] > pt2[0][1] else pt2
    #     # cv2.line(frame, (x1, lower_y), BOTTOM_CENTER, (255, 0, 0), 5)
    #     cv2.line(frame, (lower_y_pt[0][0], lower_y_pt[0][1]), bottom_center, (255, 0, 0), 5)



    # left_most = min(all_detections, key=lambda x: x.cx)
    # right_most = max(all_detections, key=lambda x: x.cx)

    # rightest_idx = 0
    # rightest_value = 0

    # for i in range(len(left_most.approx)):
    #     pt1 = left_most.approx[i]
    #     i2 = (i + 1) % len(left_most.approx)
    #     pt2 = left_most.approx[i2]

    #     x1, y1 = pt1[0]
    #     x2, y2 = pt2[0]

    #     cv2.circle(frame, (x1, y1), 5, (255, 255, 0), -1)

    #     d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    #     if d > rightest_value:
    #         rightest_value = d
    #         rightest_idx = i

    # rightest_idx_2 = (rightest_idx + 1) % len(left_most.approx)
    # cv2.line(frame, left_most.approx[rightest_idx][0], left_most.approx[rightest_idx_2][0], (255, 0, 255), 5)


    # cv2.drawContours(frame, [left_most.approx], 0, (255, 0, 0), 2)
    # cv2.drawContours(frame, [right_most.approx], 0, (255, 0, 0), 2)


    # # grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # # cv2.imshow("grey", grey)

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