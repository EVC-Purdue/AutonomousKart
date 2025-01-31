print("STARTING...")

import argparse

import time
import cv2
import numpy as np





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

def image_read(fname):
    frame = cv2.imread(fname)

    print(f"Camera opened...")

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # detect green color
    frame_threshold = cv2.inRange(frame_hsv, (25, 25, 25), (80, 255,255))

    erode_kernel = np.ones((5, 5), np.uint8)
    eroded = cv2.erode(frame_threshold, erode_kernel, iterations=2)

    kernel = np.ones((10, 10), np.uint8)
    dilated = cv2.dilate(eroded, kernel, iterations=2)

    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

    for cnt in contours:
        # x, y, w, h = cv2.boundingRect(cnt)
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        cv2.drawContours(frame, [approx], 0, (0, 255, 255), 2)
        # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)

    cv2.imshow("frame", frame)
    cv2.imshow("dilated", dilated)

    if cv2.waitKey(0) & 0xFF == ord('q'):
        pass


def main():
    ap = argparse.ArgumentParser()
    # ap.add_argument("-z", "--video-capture-zero", required=False, help="testing version", action="store_true")
    ap.add_argument("-f", "--file", required=False, help="open file")
    args = vars(ap.parse_args())
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    # camera_loop(args["video_capture_zero"])

    image_read(args["file"])
    # ------------------------------------------------------------------------ #

    print("\n\nExiting...\n\n")

    # ------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------ #

    print("DONE...")
     


if __name__ == "__main__":
    main()