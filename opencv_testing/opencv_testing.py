import cv2

cap = cv2.VideoCapture("IMG_8824.mp4")  # or 0 for webcam

while True:
    ret, frame = cap.read()
    if not ret:
        break  # end of video

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    cv2.imshow("Video", gray_frame)  # show the video in a window

    # Press 'q' to exit early
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
