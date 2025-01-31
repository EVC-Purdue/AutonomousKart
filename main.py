print("STARTING...")

import argparse

import time
import cv2





CAMERA_WAIT_AFTER_ACTIVE = 2.0
CAP_ARGS = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink"




def camera_loop(video_capture_zero):
    print(f"Opening camera...")
    time.sleep(CAMERA_WAIT_AFTER_ACTIVE)

    if video_capture_zero:
        cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(CAP_ARGS, cv2.CAP_GSTREAMER)

    print(f"Camera opened...")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame...")
            break

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break





def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-z", "--video-capture-zero", required=False, help="testing version", action="store_true")
    args = vars(ap.parse_args())
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    camera_loop(args["video_capture_zero"])
    # ------------------------------------------------------------------------ #

    print("\n\nExiting...\n\n")

    # ------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------ #

    print("DONE...")
     


if __name__ == "__main__":
    main()