import cv2 as cv
import numpy as np
import time
import math

# @param img: Image Matrix
# @param y_cutoff: Only consider y percent of image
# @ret: Mask of the road

def get_img_mask(img: np.ndarray, percent: float=0.65, r_coord=None, l_coord=None):
    if img is None:
        print ('Error opening image!')
        return None
    
    # Roi mask for a portion of image
    h, w = img.shape[:2]
    y_index = int(h * percent)
    roi_mask = np.zeros_like(img)
    roi_mask[y_index:, :] = 255
    roi_image = np.zeros_like(img)
    roi_image[roi_mask > 0] = img[roi_mask > 0]

    # Get hue mask
    image_hsv = get_hsv(roi_image)
    lower_red = np.array([0, 0, 70])
    higher_red = np.array([200, 50, 255])
    mask_red = cv.inRange(image_hsv, lower_red, higher_red) 

    kernel = np.ones((5, 5), np.uint8)
    result = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)

    if r_coord is None:
        r_coord = (w - 5, h - 5)
    
    if l_coord is None:
        l_coord = (5, h - 5)

    right = find_road_right(result, r_coord, fast_lookup=True)
    left = find_road_left(result, l_coord, fast_lookup=True)

    draw_lines(result, right, left)
    
    return (result, right, left)

# @param: Video
# @ret: Mask of the video's road
def get_video_mask(vid):
    if vid is None:
        print("Error opening video")
        return None
    
    r, f = vid.read()
    if not r:
        print("Can't get initial video frame")
        return None
    height, width = f.shape[:2]

    fps = vid.get(cv.CAP_PROP_FPS)
    if fps == 0:
        fps = 30

    right = (width - 5, height - 5)
    left = (5, height - 5)
    video = cv.VideoWriter("labeled_video.mp4", cv.VideoWriter_fourcc(*'mp4v'), fps, (width, height), isColor=False) 
    vid.set(cv.CAP_PROP_POS_FRAMES, 0)
    while (True):

        ret, frame = vid.read()

        if not ret:
            break
            
        result, right, left = get_img_mask(frame, r_coord=right, l_coord=left)

        left_deg = get_angle((0,0), (width // 2, 0), left)
        right_deg = get_angle((width - 1, 0), (width // 2, 0), right)

        cv.putText(result, f"{left_deg:.1f}", (100, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.putText(result, f"{right_deg:.1f}", (1300, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        video.write(result)
    
    vid.release()
    video.release()
    cv.destroyAllWindows()

# Start in bottom right of image
#
# @param prev: Takes previous coord found in format: (col, row) 
# @param fast_lookup: Optimize boolean
# @param threshold: Amt of neighbor pixels to check
#
# @return: Either edge of road or previous pixel
def find_road_right(img, prev, fast_lookup: bool=False, threshold=20):
    height, width = img.shape[:2]

    # Optimization code (check threshold amt of pixels)
    #
    # All width/height offset by 5 to ensure color detection
    if fast_lookup and prev != (width - 5, height - 5):
        c, r = prev     # Input takes (col, row)

        if (r == height - 5):
            w_start = min(width - 5, c + threshold)
            w_end = max(width // 2, c - threshold)
            h_start = height - 5
            h_end = 0
        else:
            h_start = min(height - 5, r + threshold)
            h_end = max(0, r - threshold)
            w_start = width - 5
            w_end = width // 2
    else:
        h_start = height - 5
        h_end = 0
        w_start = width - 5
        w_end = width // 2
    
    if img[h_start, w_start].any():
        for i in range(h_start, h_end, -1):
            if not img[i, w_start].any():
                return (w_start, i)
    
    if not img[h_start, w_start].any():
        for i in range(w_start, w_end, -1):
            if img[h_start, i].any():
                return (i, h_start)
    
    if fast_lookup and prev != (w_start, h_start):
        return find_road_right(img, prev, fast_lookup=False)
    
    return prev

# Start in bottom left of image
#
# @param prev: Takes previous coord found in format: (col, row) 
# @param fast_lookup: Optimize boolean
# @param threshold: Amt of neighbor pixels to check
#
# @return: Either edge of road or previous pixel
def find_road_left(img, prev, fast_lookup=False, threshold=20):
    height, width= img.shape[:2]

    # Optimization code (check threshold amt of pixels)
    #
    # All width/height offset by 5 to ensure color detection
    if fast_lookup and prev != (5, height - 5):
        c, r = prev

        if (r == height - 5):
            w_start = max(5, c - threshold)
            w_end = min(width // 2, c + threshold)
            h_start = height - 5
            h_end = height // 2
        else:
            h_start = min(height - 5, r + threshold)
            h_end = max(0, r - threshold)
            w_start = 5
            w_end = width // 2
    else:
        h_start = height - 5
        h_end = 0
        w_start = 5
        w_end = width // 2

    if img[h_start, w_start].any():
        for i in range(h_start, h_end, -1):
            if not img[i, w_start].any():
                return (w_start, i)
    
    if not img[h_start, w_start].any():
        for i in range(w_start, w_end):
            if img[h_start, i].any():
                return (i, h_start)
    
    # Didn't find it so do slow loop
    if fast_lookup and prev != (w_start, h_start):
        return find_road_left(img, prev, fast_lookup=False)
    
    return prev

def get_angle_vector(middle_coord, road_coord):
    dx = road_coord[0] - middle_coord[0] 
    dy = road_coord[1] - middle_coord[1]  
    angle_rad = math.atan2(dy, dx)
    return math.degrees(angle_rad)

def get_angle(zero_coord, middle_coord, road_coord):
    a = get_distance(road_coord, zero_coord)
    b = get_distance(zero_coord, middle_coord)
    c = get_distance(middle_coord, road_coord)

    numerator = a*a - c*c - b*b
    denominator = -2 * c * b

    rads = math.acos( numerator / denominator)
    return math.degrees(rads)


def get_distance(point1, point2):
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    return math.sqrt(dx*dx + dy *dy)

def within_difference(r_coord, l_coord, threshold):
    c1, r1 = r_coord
    c2, r2 = l_coord

    if (abs(r2 - r1) <= threshold and abs(c2 - c1) <= threshold):
        return True
    else:
        return False

# @param r_coord: Right road coord 
# @param l_coord: Left road coord
# @ret: Return img with lines
def draw_lines(img, r_coord, l_coord, middle=None):
    if not middle:
        middle = img.shape[1]
        middle: int = middle // 2
            

    cv.line(img, (middle, 0), r_coord, (200, 0, 200), 3)
    cv.line(img, (middle, 0), l_coord, (200, 0, 200), 3)
    return img

def get_threshold(img):
    return cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY,11,2)

def get_contours(thresh):
    contours, hierachy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    return contours

def get_gradient(img):
    h, w = img.shape[:2]
    gradient = np.linspace(0, 1, h).reshape(h, 1)
    gradient = np.repeat(gradient, w, axis=1)
    gradient = cv.merge([gradient]*3).astype(np.float32)

    img_float = img.astype(np.float32)
    img_grad = img_float * gradient
    img_grad = np.clip(img_grad, 0, 255).astype(np.uint8)
    return img_grad

# @param: Takes a str file path
# @ret: Return a video
def get_video(path: str):
    return cv.VideoCapture(path)

# @param: Takes a str file path
# @ret: Photo as a matrix
def get_image(path: str):
    return cv.imread(cv.samples.findFile(path), cv.IMREAD_COLOR)

# @param: Photo matrix
# @ret: HSV representation
def get_hsv(img: np.ndarray):
    return cv.cvtColor(img, cv.COLOR_BGR2HSV)

# @param: Photo matrix
# @ret: Grayscale representation
def get_greyscale(img: np.ndarray):
    return cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Logic for closing image window
def display_img(img):
    cv.imshow("Window", img)

    while True:
        if (cv.waitKey(1) == 13 or cv.getWindowProperty("Window", cv.WND_PROP_VISIBLE) < 1):
            cv.destroyAllWindows()
            return

# photo = get_image("/ws/data/internet_test_footage/driver-pov-img.png")

# display_img(photo)

# img, right, left = get_img_mask(photo, percent=0.0)

# width, height = photo.shape[:2]

# left_deg = get_angle((0,0), (width // 2, 0), (left[0], left[1]))
# right_deg = get_angle((width - 1, 0), (width // 2, 0), (right[0], right[1]))

# print(left_deg)
# print(right_deg)

# display_img(img)

original_video = get_video("/ws/data/EVC_test_footage/video.mp4")

get_video_mask(original_video)