from autonomous_kart.nodes.opencv_pathfinder import label
import time
import cv2 as cv

original_img = label.get_image("/ws/data/internet_test_footage/driver-pov-img.png")

label.display_img(original_img)

start = time.perf_counter()
road_image = label.get_img_mask(original_img, percent=0.0)
end = time.perf_counter()

time = start - end
print(f"Time: {time} secs", time)

label.display_img(road_image)


original_video = label.get_video("/ws/data/EVC_test_footage/video.mp4")

video = label.get_video_mask(original_video)