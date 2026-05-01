import cv2 as cv
import numpy as np
import math

# Angle calculation
def get_angle(zero_coord, middle_coord, road_coord):
    a = get_distance(road_coord, zero_coord)
    b = get_distance(zero_coord, middle_coord)
    c = get_distance(middle_coord, road_coord)

    numerator = a*a - c*c - b*b
    denominator = -2 * c * b

    if math.isclose(denominator, 0):
        return None

    rads = math.acos( numerator / denominator)
    return math.degrees(rads)

# Distance calculation
def get_distance(point1, point2):
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    return math.sqrt(dx*dx + dy *dy)

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


# @param: Image
# @ret: Adaptive Gaussain threshold
def get_threshold(img):
    return cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY,11,2)


# @param: Threshold
# @ret: Traditional contours
def get_contours(thresh):
    contours, im = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    return contours

# @param: Photo matrix
# @ret: HSV representation
def convert_bgr_to_hsv(img: np.ndarray):
    return cv.cvtColor(img, cv.COLOR_BGR2HSV)

# @param: Photo matrix
# @ret: Grayscale representation
def convert_bgr_to_greyscale(img: np.ndarray):
    return cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# @param: Takes a str file path
# @ret: Return a video
def get_video(path: str):
    return cv.VideoCapture(path)

# @param: Takes a str file path
# @ret: Photo as a matrix
def get_image(path: str):
    return cv.imread(cv.samples.findFile(path), cv.IMREAD_COLOR)

# Logic for closing image window
def display_img(img):
    cv.imshow("Window", img)

    while True:
        if (cv.waitKey(1) == 13 or cv.getWindowProperty("Window", cv.WND_PROP_VISIBLE) < 1):
            cv.destroyAllWindows()
            return

# @param: Image
# @ret: Gaussian image
def get_gaussian_img(img):
    h, w = img.shape[:2]
    gradient = np.linspace(0, 1, h).reshape(h, 1)
    gradient = np.repeat(gradient, w, axis=1)
    gradient = cv.merge([gradient]*3).astype(np.float32)

    img_float = img.astype(np.float32)
    img_grad = img_float * gradient
    img_grad = np.clip(img_grad, 0, 255).astype(np.uint8)
    return img_grad