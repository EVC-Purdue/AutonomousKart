"""
This file contains code that we might want to use in the future but it otherwise just cluttering the main file
"""


# ap.add_argument("-z", "--video-capture-zero", required=False, help="testing version", action="store_true")

# CAMERA_WAIT_AFTER_ACTIVE = 1.0
# CAP_ARGS = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink"

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