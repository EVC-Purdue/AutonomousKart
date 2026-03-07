import cv2 as cv
import numpy as np
import time
import math

# @param: Video
# @ret: Mask of the video's road
def get_video_mask(vid, debug=False, percent=0.65, optimize=True, pixel_range=6, pic_offset=5, steps=20):
    if vid is None:
        print("Error opening video")
        return None
    
    r, f = vid.read()
    if not r:
        print("Can't get initial video frame")
        return None
    h, w = f.shape[:2]
    height, width = h - 1 - pic_offset, w - 1 - pic_offset

    fps = vid.get(cv.CAP_PROP_FPS)
    if fps == 0:
        fps = 30

    prev_right = (width, height)
    prev_left = (pic_offset, height)
    video = cv.VideoWriter("labeled_video.mp4", cv.VideoWriter_fourcc(*'mp4v'), fps, (w, h), isColor=False) 
    vid.set(cv.CAP_PROP_POS_FRAMES, 0)
    while (True):

        ret, frame = vid.read()

        if not ret:
            break
    
        pl = prev_left
        pr = prev_right
            
        result, prev_right, prev_left = get_img_mask(frame, debug=debug, percent=percent, prev_right=prev_right, prev_left=prev_left, optimize=optimize, pic_offset=pic_offset, pixel_range=pixel_range, steps=steps)

        right_deg = get_angle((width, pic_offset), (width // 2, pic_offset), prev_right)
        left_deg = get_angle((pic_offset, pic_offset), (width // 2, pic_offset), prev_left)

        cv.putText(result, f"{pl}", (500, 200), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.putText(result, f"{pr}", (700, 200), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv.putText(result, f"{prev_left}", (500, 400), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.putText(result, f"{prev_right}", (700, 400), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        

        cv.putText(result, f"{right_deg:.1f}", (1300, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.putText(result, f"{left_deg:.1f}", (100, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        video.write(result)
    
    vid.release()
    video.release()
    cv.destroyAllWindows()


# @param img: Image Matrix
# @param y_cutoff: Only consider y percent of image
# @ret: Mask of the road
def get_img_mask(img: np.ndarray, debug=False, percent=0.65, prev_right=None, prev_left=None, optimize=True, pixel_range=6, pic_offset=5, steps=20):
    if img is None:
        print ('Error opening image!')
        return None
    
    # Roi mask for a portion of image
    h, w = img.shape[:2]
    height, width = h - 1 - pic_offset, w - 1 - pic_offset

    h, w = img.shape[:2]
    
    y_index = int(height * percent)

    roi_mask = np.zeros(img.shape[:2], dtype=np.uint8)
    roi_mask[y_index:, :] = 255

    if debug:
        roi_mask[y_index:, :] = 255
    else:
        roi_mask[y_index:height-10, pic_offset+10:width-10] = 255

    roi_image = cv.bitwise_and(img, img, mask=roi_mask)

    # Get hue mask
    image_hsv = get_hsv(roi_image)
    lower_red = np.array([0, 0, 70])
    higher_red = np.array([200, 50, 255])
    mask_red = cv.inRange(image_hsv, lower_red, higher_red) 

    kernel = np.ones((3, 3), np.uint8)
    result = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)

    if prev_right is None:
        prev_right = (width, height)
    
    if prev_left is None:
        prev_left = (pic_offset, height)

    right = find_road_coord(result, prev_right, True, optimize=optimize, pixel_range=pixel_range, pic_offset=pic_offset, steps=steps)
    left = find_road_coord(result, prev_left, False, optimize=optimize, pixel_range=pixel_range, pic_offset=pic_offset, steps=steps)

    if debug:
        draw_lines(result, right, left)
    
    return (result, right, left)

def find_road_coord(img, prev_pixel, right_side: bool, optimize=True, pixel_range=6, pic_offset=5, steps=20):
    h, w = img.shape[:2]
    height, width = h - 1 - pic_offset, w - 1 - pic_offset

    if (optimize):
        new_pixel = lookup_road_coord(img, prev_pixel, right_side, pixel_range=pixel_range, pic_offset=pic_offset)
        if new_pixel:
            return new_pixel
    
    steps_h = -1 * steps
    steps_w = -1 * steps if right_side else steps
    slow_step_h = 1
    slow_step_w = 1 if right_side else -1

    h_start = height
    h_end = pic_offset
    w_start = width if right_side else pic_offset
    w_end = width // 2
    
    if img[h_start, w_start].any():
        for i in range(h_start, h_end, steps_h):
            if not img[i, w_start].any():
                for j in range(i, h_start + 1, slow_step_h):
                    if img[j, w_start].any():
                        return (w_start, j)
    else:
        for i in range(w_start, w_end, steps_w):
            if img[h_start, i].any():
                for j in range(i, w_start + slow_step_w, slow_step_w):
                    if not img[h_start, j].any():
                        return (j, h_start)
    
    return prev_pixel
           

def lookup_road_coord(img, prev_pixel, right_side, pixel_range=6, pic_offset=5):
    h, w = img.shape[:2]
    height, width = h - 1 - pic_offset, w - 1 - pic_offset
    prev_width, prev_height, = prev_pixel[:2]

    if right_side:
        h_start = max(pic_offset, prev_height - pixel_range)
        h_end = min(height, prev_height + pixel_range)
        h_step = 1
        w_start = min(width, prev_width + pixel_range)
        w_end = max(width // 2, prev_width - pixel_range)
        w_step = -1
    else:
        h_start = max(pic_offset, prev_height - pixel_range)
        h_end = min(height, prev_height + pixel_range)
        h_step = 1
        w_start = max(pic_offset, prev_width - pixel_range)
        w_end = max(width // 2, prev_width + pixel_range)
        w_step = 1

    if (prev_width == width):
        if img[h_start, width].any():
            return None
        elif not img[h_end, width].any():
            return None
        else:
            for i in range(h_start, h_end + h_step, h_step):
                if img[i, width].any():
                    return (width, i)
    elif (prev_height == height):
        if img[height, w_start].any():
            return None
        elif not img[height, w_end].any():
            return None
        else:
            for i in range(w_start, w_end + w_step, w_step):
                if img[height, i].any():
                    return (i, height)
    else:
        return None
    
    return None

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
# track, r, l = get_img_mask(photo, percent=0.0)

# display_img(track)

# start = time.perf_counter()
# for i in range(0, 1000): 
#     img, right, left = get_img_mask(photo, percent=0.70, steps=40, pixel_range=6)
# end = time.perf_counter()

# t = (end - start) / 1000
# print(f"Time: {t}")

# original_video = get_video("/ws/data/EVC_test_footage/video.mp4")

# get_video_mask(original_video, debug=True)