import cv2 as cv
import numpy as np
import os
import time

from autonomous_kart.nodes.opencv_pathfinder import utils

KERNEL = np.ones((3,3), np.uint8)
LOWER_RED = np.array([0,0,70])
UPPER_RED = np.array([200,50,255])

class AngleFinder:
    def __init__(self, logger):
        self.prev_right = None
        self.prev_left = None
        self.logger = logger
        self.start_time = time.strftime("%Y_%m_%d_%H_%M_%S")

    # @param img: Normal image
    # @param log_folder: Directory for write debug info
    # @param debug: Draws lines on image
    # @param bottom_percent: Percent of the road to cutoff from bottom
    # @param top_per: Percent of the road to cutoff from top
    # @param pixel_range: Range of pixels to check around previous pixel for O(1) lookup
    # @param pic_offset: Pixel offset from edge of image
    # @ret image angles or None when cannot find angles/coords
    def get_img_angles(self, img, frame_count=-1, log_folder="logs", debug=False, bottom_per=0.0, top_per=0.0, pixel_range=3, pic_offset=5, capture_frequency=100):
        image, right, left = self.get_img_mask(img, bottom_per=bottom_per, top_per=top_per, pixel_range=pixel_range, pic_offset=pic_offset)
        
        if image is None or right is None or left is None:
            return (None, None)

        h, w = img.shape[:2]
        height, width = h - 1 - pic_offset, w - 1 - pic_offset
        top_y_index = int(height * top_per)
        bottom_y_index = height - int(height * bottom_per)

        right_deg = utils.get_angle((width, pic_offset), (width // 2, pic_offset), right)
        left_deg = utils.get_angle((pic_offset, pic_offset), (width // 2, pic_offset), left)

        if right_deg is None or left_deg is None:
            return (None, None)
        
        if debug and frame_count % capture_frequency == 0:
            self.logger.info("Wrote Debug Frames To Disk")

            os.makedirs(log_folder, exist_ok=True)
            os.makedirs(f"{log_folder}/normal_frames", exist_ok=True)
            os.makedirs(f"{log_folder}/debug_frames", exist_ok=True)
            os.makedirs(f"{log_folder}/normal_frames/{self.start_time}", exist_ok=True)
            os.makedirs(f"{log_folder}/debug_frames/{self.start_time}", exist_ok=True)

            cv.imwrite(f"{log_folder}/normal_frames/{self.start_time}/frame_{frame_count}.jpg", img)
            utils.draw_lines(image, right, left)
            self.write_debug_info(image, (left, right), (left_deg, right_deg))
            cv.imwrite(f"{log_folder}/debug_frames/{self.start_time}/frame_{frame_count}.jpg", image)

            with open(f"{log_folder}/debug_frames/{self.start_time}/log", "a") as f:
                f.write(f"Frame: {frame_count}, ")
                f.write(f"LDeg: {left_deg:.1f}, ")
                f.write(f"LPrev: {self.prev_left}, ")
                f.write(f"LCur: {left}, ")

                f.write(f"RDeg: {right_deg:.1f}, ")
                f.write(f"RPrev: {self.prev_right}, ")
                f.write(f"RCur: {right}")

                f.write("\n")

        return (right_deg, left_deg)


    # @param img: Normal image
    # @param debug: Draws lines on image
    # @param top_per: Percent of the road to cutoff from top
    # @param pixel_range: Range of pixels to check around previous pixel for O(1) lookup
    # @param pic_offset: Pixel offset from edge of image
    # @ret Masked image & right/left divide between road & grass or None when image doesn't exist
    def get_img_mask(self, img: np.ndarray, bottom_per=0.0, top_per=0.0, pixel_range=3, pic_offset=5):
        if img is None:
            self.logger.error("No Image Provided as Input to get_img_mask")
            return (None, None, None)
        
        # Roi mask for a portion of image
        h, w = img.shape[:2]
        height, width = h - 1 - pic_offset, w - 1 - pic_offset
        top_y_index = int(height * top_per)
        bottom_y_index = height - int(height * bottom_per)

        if bottom_y_index <= top_y_index:
            self.logger.warning("ROI Image Bounds Collide")
            return (None, None, None)

        roi_image = img[top_y_index:bottom_y_index, pic_offset:width]

        # Get hue mask
        image_hsv = utils.convert_bgr_to_hsv(roi_image)
        mask_red = cv.inRange(image_hsv, LOWER_RED, UPPER_RED) 
        result = cv.morphologyEx(mask_red, cv.MORPH_OPEN, KERNEL)

        if self.prev_right is None:
            self.prev_right = (width, height)
        
        if self.prev_left is None:
            self.prev_left = (pic_offset, height)
        
        right = self.lookup_road_coord(result, self.prev_right, right_side=True, pic_offset=pic_offset, pixel_range=pixel_range)
        left = self.lookup_road_coord(result, self.prev_left, right_side=False, pic_offset=pic_offset, pixel_range=pixel_range)

        if not right:
            right = self.vectorized_road_coord(result, right_side=True)
        
        if not left:
            left = self.vectorized_road_coord(result, right_side=False)

        self.prev_right = right
        self.prev_left = left
        
        return (result, right, left)

    # Vectorized road lookup
    # @param right_side: Determine what side of image
    # @param pic_offset: Offset from edge of photo
    def vectorized_road_coord(self, img, right_side=True, pic_offset=5):
        h, w = img.shape[:2]
        height, width = h - 1 - pic_offset, w - 1 - pic_offset

        w_start = width if right_side else pic_offset
        w_end = width // 2
        h_start = height
        h_end = pic_offset

        if img[h_start, w_start] != 0:
            column = img[h_end:h_start + 1, w_start]
            inv = np.logical_not(column[::-1])
            idx = np.argmax(inv)
            return (w_start, h_start - idx)
        else:
            if right_side:
                row = img[h_start, w_end:w_start + 1][::-1]
                idx = np.argmax(row)
                return (w_start - idx, h_start)
            else:
                row = img[h_start, w_start:w_end + 1]
                idx = np.argmax(row)
                return (w_start + idx, h_start)

    # Give a previous pixel, look at at "pixel_range" number of 
    # pixel neighbors from previous pixel & determine if road exists
    # 
    # param right_side: Bool determine if on right or left side of image
    # param pixel_range: Number of neighbor pixels to check
    # param pic_offset: Offset from edge of image
    # ret: Pixel road coords otherwise None
    def lookup_road_coord(self, img, prev_pixel, right_side=True, pixel_range=6, pic_offset=5):
        h, w = img.shape[:2]
        height, width = h - 1 - pic_offset, w - 1 - pic_offset
        prev_width, prev_height, = prev_pixel[:2]

        # Determine starting & ending pixels
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
            w_end = min(width // 2, prev_width + pixel_range)
            w_step = 1

        if (prev_height == height and prev_width == width):
            if not img[height, w_end].any() and not img[h_start, width].any():
                return None
                
        
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

        return None
    
    # @param vid: Video
    # @param debug: Draws lines on image
    # @param percent: Percent of the road (top down)
    # @param pixel_range: Range of pixels to check around previous pixel for O(1) lookup
    # @param pic_offset: Pixel offset from edge of image
    # @ret video or None
    # def get_video_mask(self, vid, debug=False, top_per=0.0, bottom_per=0.0, pixel_range=3, pic_offset=5):
    #     if vid is None:
    #         self.logger.error("Error opening video")
    #         return None
        
    #     r, f = vid.read()
    #     if not r:
    #         self.logger.error("Can't get initial video frame")
    #         return None
    #     h, w = f.shape[:2]
    #     width = w - 1 - pic_offset

    #     fps = vid.get(cv.CAP_PROP_FPS)
    #     if fps == 0:
    #         fps = 30

    #     video = cv.VideoWriter("labeled_video.mp4", cv.VideoWriter_fourcc(*'mp4v'), fps, (w, h), isColor=False) 
    #     vid.set(cv.CAP_PROP_POS_FRAMES, 0)

    #     while (True):
    #         ret, frame = vid.read()

    #         if not ret:
    #             break
                
    #         result, cur_right, cur_left = self.get_img_mask(frame, debug=debug, top_per=top_per, bottom_per=bottom_per, pic_offset=pic_offset, pixel_range=pixel_range)
    #         result = cv.cvtColor(result, cv.COLOR_GRAY2BGR)

    #         width = w - 1 - pic_offset

    #         # Write debug info on video
    #         if debug:
    #             right_deg = utils.get_angle((width, pic_offset), (width // 2, pic_offset), cur_right)
    #             left_deg = utils.get_angle((pic_offset, pic_offset), (width // 2, pic_offset), cur_left)

    #             if right_deg is None:
    #                 right_deg = -1
                
    #             if left_deg is None:
    #                 left_deg = -1
            
    #             pos = (cur_left, cur_right)
    #             degrees = (left_deg, right_deg)
    #             self.write_debug_info(result, pos, degrees)

    #         video.write(result)
        
    #     vid.release()
    #     video.release()
    #     cv.destroyAllWindows()

    def write_debug_info(self, image, pos, degrees):
        cur_left, cur_right = pos
        left_deg, right_deg = degrees

        height, width = image.shape[:2]
        
        margin_x = int(width * 0.05)
        margin_y = int(height * 0.05)
        font_scale = min(width, height) / 300.0
        thickness = max(1, int(font_scale * 6))
        
        line_spacing = int(font_scale * 40)
        
        y_pos = height - margin_y
        cv.putText(image, f"LDeg: {left_deg:.1f}", (margin_x, y_pos), cv.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness)
        cv.putText(image, f"LPrev: {self.prev_left}", (margin_x, y_pos - line_spacing), cv.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness)
        cv.putText(image, f"LCur: {cur_left}", (margin_x, y_pos - line_spacing * 2), cv.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness)

        right_x = int(width * 0.5)
        cv.putText(image, f"RDeg: {right_deg:.1f}", (right_x, y_pos), cv.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness)
        cv.putText(image, f"RPrev: {self.prev_right}", (right_x, y_pos - line_spacing), cv.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness)
        cv.putText(image, f"RCur: {cur_right}", (right_x, y_pos - line_spacing * 2), cv.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness)