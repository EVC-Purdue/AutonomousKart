import cv2 as cv
import numpy as np
import time

# @param img: Image Matrix
# @param y_cutoff: Only consider y percent of image
# @ret: Mask of the road
def get_img_mask(img: np.ndarray, percent: float=0.0, r_coord=(0,0), l_coord=(0,0)):
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
    lower_red = np.array([0, 0, 100])
    higher_red = np.array([200, 50, 255])
    mask_red = cv.inRange(image_hsv, lower_red, higher_red) 

    kernel = np.ones((5, 5), np.uint8)
    result = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)

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

    right = (0,0)
    left = (0,0)
    video = cv.VideoWriter("labeled_video.mp4", cv.VideoWriter_fourcc(*'mp4v'), fps, (width, height), isColor=False) 
    vid.set(cv.CAP_PROP_POS_FRAMES, 0)
    while (True):
        ret, frame = vid.read()

        if not ret:
            break
            
        result, right, left = get_img_mask(frame, r_coord=right, l_coord=left)
        video.write(result)
    
    vid.release()
    video.release()
    cv.destroyAllWindows()

def find_road_right(img, prev, fast_lookup: bool=False, threshold=20):
    height, width = img.shape[:2]

    # # Starting & Ending pos based on bool
    # if fast_lookup and prev != (0,0):
    #     c, r = prev
    #     h_start = max(0, r - threshold)
    #     h_end = min(height, r + threshold)
    # else:
    #     h_start = height - 1
    #     h_end = 0
    
    if img[height - 5, width - 5].any():
        for i in range(height - 5, height // 2, -1):
            if not img[i, width - 5].any():     # Offset pixels bc. border pixels are black
                return (width - 5, i)
    else:
        for i in range(width - 5, width // 2, -1):
            if img[height - 5, i].any():
                return (i, height - 5)
    
    # # Didn't find it so do slow loop
    # if fast_lookup and prev != (0,0):
    #     return find_road_right(img, prev, fast_lookup=False)
    
    return prev

def find_road_left(img, prev, fast_lookup=False, threshold=20):
    height, width= img.shape[:2]

    if img[height - 5, 5].any():
        for i in range(height - 5, height // 2, -1):
            if not img[i, 5].any():     # Offset pixels bc. border pixels are black
                return (5, i)
    else:
        for i in range(width // 2):
            if img[height - 5, i].any():
                return (i, height - 5)

    # if fast_lookup and prev != (0,0):
    #     c, r = prev
    #     h_start = max(0, r - threshold)
    #     h_end = min(height - 1, r + threshold)
    # else:
    #     h_start = 0
    #     h_end = height

    # for i in range(h_start, h_end):
    #     if img[i, 5].any():             # Offset pixels bc. border pixels are black
    #         return (5, i)
    
    # if fast_lookup and prev != (0,0):
    #     c, r = prev
    #     w_start = max(0, c - threshold)
    #     w_end = min(width // 2, c + threshold)
    # else:
    #     w_start = 0
    #     w_end = width // 2
    
    # for i in range(w_start, w_end):
    #     if img[height - 5, i].any():    # Offset pixels bc. border pixels are black
    #         return (i, height - 5)
    
    # # Didn't find it so do slow loop
    # if fast_lookup and prev != (0,0):
    #     return find_road_left(img, prev, fast_lookup=False)
    
    return prev

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

# photo = get_image("./another_6.png")

# display_img(photo)

# img, r, l = get_img_mask(photo)

# display_img(img)



original_video = get_video("/ws/data/EVC_test_footage/video.mp4")

get_video_mask(original_video)