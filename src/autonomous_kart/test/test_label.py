from autonomous_kart.autonomous_kart.nodes.opencv_pathfinder import angle
import time
import cv2 as cv

original_img = angle.get_image("/ws/data/internet_test_footage/driver-pov-img.png")

angle.display_img(original_img)

start = time.perf_counter()
road_image = angle.get_img_mask(original_img, percent=0.0)
end = time.perf_counter()

time = start - end
print(f"Time: {time} secs", time)

angle.display_img(road_image)


original_video = angle.get_video("/ws/data/EVC_test_footage/video.mp4")

video = angle.get_video_mask(original_video)