import label
import cv2 as cv
import time
import cProfile

def get_avg_time(vid, trips=1, optimize=True, percent=0.70, steps=40, pixel_range=6, pic_offset=5):
    frames = get_frames(vid)
    total_time = 0

    h, w= frames[0].shape[:2]
    width, height = w - 1 - pic_offset, h - 1 - pic_offset

    right = (width, height)
    left = (pic_offset, height)

    for i in range(min(trips, 2)):
        start = time.perf_counter()
        for j in frames:
            img, right, left = label.get_img_mask(j, debug=False, prev_right=right, prev_left=left, optimize=optimize, percent=percent, steps=steps, pixel_range=pixel_range, pic_offset=pic_offset)
        end = time.perf_counter()
        t = (end - start) / len(frames)
        total_time += t

    return total_time / trips

def get_frames(vid):
    fps = vid.get(cv.CAP_PROP_FPS)
    if fps == 0:
        fps = 30

    vid.set(cv.CAP_PROP_POS_FRAMES, 0)
    frames = []
    while (True):

        ret, frame = vid.read()

        if not ret:
            break

        frames.append(frame)
    
    return frames


# photo = label.get_image("/ws/data/internet_test_footage/driver-pov-img.png")

# cProfile.run('label.get_img_mask(photo)')
original_video = label.get_video("/ws/data/EVC_test_footage/video.mp4")

avg = get_avg_time(original_video, trips=1, optimize=True, percent=0.7, steps=40, pixel_range=4, pic_offset=5)

print(avg)