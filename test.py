import cv2
import numpy as np
import time


def isolate_green(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, np.array([20, 40, 40]), np.array([80, 255, 255]))
    hsv[:, :, 1] = np.clip(hsv[:, :, 1].astype(np.float32) * np.where(green_mask != 0, 1.5, 1.0), 0, 255).astype(
        np.uint8)
    boosted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask3 = cv2.merge([green_mask, green_mask, green_mask])
    result = np.where(mask3, boosted, cv2.merge([gray, gray, gray]))
    return result, green_mask


def calculate_angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p1[1] - p2[1]
    return np.degrees(np.arctan2(dy, dx))


cap = cv2.VideoCapture('Vid.mp4')
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Video: {width}x{height} @ {fps} fps")

fps_start = time.time()
fps_counter = 0
fps_display = 0

while cap.isOpened():
    frame_time_start = time.time()
    ret, frame = cap.read()
    if not ret:
        break
    frame = frame[2 * frame.shape[0] // 3:, :]
    display_h, display_w = frame.shape[:2]
    frame_small = cv2.resize(frame, (display_w // 2, display_h // 2), interpolation=cv2.INTER_AREA)
    result, green_mask = isolate_green(frame_small)
    h, w = frame_small.shape[:2]
    center_x = w // 2
    bottom_point = (center_x, h - 1)

    center_hit = None
    for y in range(h - 1, -1, -1):
        if green_mask[y, center_x] != 0:
            center_hit = (center_x, y)
            break

    left_hit = None
    for y in range(h - 1, -1, -1):
        if green_mask[y, 0] != 0:
            left_hit = (0, y)
            break

    right_hit = None
    for y in range(h - 1, -1, -1):
        if green_mask[y, w - 1] != 0:
            right_hit = (w - 1, y)
            break

    angle_left = None
    if left_hit:
        dx = left_hit[0] - bottom_point[0]
        dy = bottom_point[1] - left_hit[1]

        angle_left = 90 - np.degrees(np.arctan2(dy * 3, -dx))

    angle_right = None
    if right_hit:
        dx = right_hit[0] - bottom_point[0]
        dy = bottom_point[1] - right_hit[1]
        angle_right = np.degrees(np.arctan2(dy * 3, -dx)) - 90

    if center_hit:
        cv2.line(result, bottom_point, center_hit, (0, 0, 255), 2)
    if left_hit:
        cv2.line(result, bottom_point, left_hit, (255, 0, 0), 2)
    if right_hit:
        cv2.line(result, bottom_point, right_hit, (255, 0, 0), 2)

    result_large = cv2.resize(result, (display_w, display_h), interpolation=cv2.INTER_NEAREST)
    frame_time = (time.time() - frame_time_start) * 1000
    fps_counter += 1
    if (time.time() - fps_start) > 1:
        fps_display = fps_counter
        fps_counter = 0
        fps_start = time.time()

    y_offset = 30
    cv2.putText(result_large, f'Frame Time: {frame_time:.1f}ms', (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0), 2)
    y_offset += 30
    cv2.putText(result_large, f'Bottom: {bottom_point}', (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    y_offset += 30
    if left_hit:
        cv2.putText(result_large, f'Left Hit: {left_hit}', (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0),
                    2)
        y_offset += 30
    if angle_left:
        cv2.putText(result_large, f'Left Angle: {angle_left:.1f}deg', (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 0), 2)
    y_offset += 30
    if right_hit:
        cv2.putText(result_large, f'Right Hit: {right_hit}', (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0),
                    2)
        y_offset += 30
    if angle_right:
        cv2.putText(result_large, f'Right Angle: {angle_right:.1f}deg', (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 0), 2)

    cv2.namedWindow('Green Isolated', cv2.WINDOW_NORMAL)
    cv2.imshow('Green Isolated', result_large)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()



