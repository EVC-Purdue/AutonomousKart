import cv2 as cv
import numpy as np
import time

# @param img: Image Matrix
# @param y_cutoff: Only consider y percent of image
# @param colored: Return a colored image
# @ret: Mask of the road
def get_img_mask(img: np.ndarray, percent: float=0.6, colored: bool=False):
    if img is None:
        print ('Error opening image!')
        return None
    
    # Roi mask for a portion of image
    h = img.shape[0]
    y_index = int(h * percent)
    roi_mask = np.zeros_like(img)
    roi_mask[y_index:, :] = 255
    roi_image = np.zeros_like(img)
    roi_image[roi_mask > 0] = img[roi_mask > 0]

    # Get hue mask
    image_hsv = get_hsv(roi_image)
    lower_red = np.array([0, 0, 100])
    higher_red = np.array([50, 50, 255])
    mask_red = cv.inRange(image_hsv, lower_red, higher_red)

    if not colored:
        result = np.zeros_like(roi_image)
        result[mask_red > 0] = roi_image[mask_red > 0]
    else:
        result = np.zeros_like(image_hsv)
        result[mask_red > 0] = image_hsv[mask_red > 0]

    kernel = np.ones((13, 13), np.uint8)
    result = cv.morphologyEx(result, cv.MORPH_OPEN, kernel)

    r = find_road_right(result)
    l = find_road_left(result)

    draw_lines(result, r, l)
    
    return result

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

    video = cv.VideoWriter("labeled_video.mp4", cv.VideoWriter_fourcc(*'mp4v'), fps, (width, height)) 
    vid.set(cv.CAP_PROP_POS_FRAMES, 0)
    while (True):
        ret, frame = vid.read()

        if not ret:
            break

        video.write(get_img_mask(frame))
    
    vid.release()
    video.release()
    cv.destroyAllWindows()

def find_road_right(img):
    h, w = img.shape[:2]
    for i in range(h):
        if img[i, w-3].any():
            return (w - 3, i)
    
    for i in range(w - 1, w // 2, -1):
        if img[h - 5, i].any():
            return (i, h - 5)
    
    return (0,0)

def find_road_left(img):
    h, w = img.shape[:2]
    for i in range(h):
        if img[i, 2].any():
            return (2, i)
    
    for i in range(w // 2):
        if img[h - 5, i].any():
            return (i, h - 5)
    
    return (0,0)

# @param r_coord: Right road coord 
# @param l_coord: Left road coord
# @ret: Return img with lines
def draw_lines(img, r_coord, l_coord):
    middle = img.shape[1]
    m: int = middle // 2
    cv.line(img, (m, 0), r_coord, (200, 0, 200), 3)
    cv.line(img, (m, 0), l_coord, (200, 0, 200), 3)
    return img

def get_threshold(img):
    return cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY,11,2)

def get_contours(thresh):
    contours, hierachy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    return contours

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



original_video = get_video("/ws/data/EVC_test_footage/video.mp4")

get_video_mask(original_video)