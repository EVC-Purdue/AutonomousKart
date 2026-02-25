import cv2 as cv
import numpy as np
import time

# @param img: Image Matrix
# @param y_cutoff: Only consider y percent of image
# @param colored: Return a colored image
# @ret: Mask of the road
def get_img_mask(img: np.ndarray, percent: float=0.7, colored: bool=False):
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
    higher_red = np.array([60, 40, 255])
    mask_red = cv.inRange(image_hsv, lower_red, higher_red)

    if not colored:
        result = np.zeros_like(roi_image)
        result[mask_red > 0] = roi_image[mask_red > 0]
    else:
        result = np.zeros_like(image_hsv)
        result[mask_red > 0] = image_hsv[mask_red > 0]

    kernel = np.ones((9, 9), np.uint8)
    result = cv.morphologyEx(result, cv.MORPH_OPEN, kernel)
    
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

    video = cv.VideoWriter("labeled_video.mp4", cv.VideoWriter_fourcc(*'mp4v'), fps, (width, height)) 
    vid.set(cv.CAP_PROP_POS_FRAMES, 0)
    while (vid.isOpened()):
        ret, frame = vid.read()

        if not ret:
            break

        video.write(get_img_mask(frame))
    
    vid.release()
    video.release()
    cv.destroyAllWindows()
    
    return video

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


original_img = get_image("/ws/data/internet_test_footage/driver-pov-img.png")

display_img(original_img)

start = time.perf_counter()
road_image = get_img_mask(original_img, percent=0.0)
end = time.perf_counter()

time = start - end
print(f"Time: {time} secs", time)

display_img(road_image)


# original_video = get_video("/ws/data/EVC_test_footage/video.mp4")

# video = get_video_mask(original_video)