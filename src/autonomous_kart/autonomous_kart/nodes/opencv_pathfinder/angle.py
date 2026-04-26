import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import cv2 as cv
import numpy as np
import utils

KERNEL = np.ones((3,3), np.uint8)
LOWER_RED = np.array([0,0,70])
UPPER_RED = np.array([200,50,255])

class AngleFinder:
    def __init__(self, node):
        self.prev_right = None
        self.prev_left = None
        self.logger = node

    # @param vid: Video
    # @param debug: Draws lines on image
    # @param percent: Percent of the road (top down)
    # @param pixel_range: Range of pixels to check around previous pixel for O(1) lookup
    # @param pic_offset: Pixel offset from edge of image
    # @ret video or None
    def get_video_mask(self, vid, debug=False, percent=0.0, pixel_range=3, pic_offset=5):
        if vid is None:
            self.logger.error("Error opening video")
            return None
        
        r, f = vid.read()
        if not r:
            self.logger.error("Can't get initial video frame")
            return None
        h, w = f.shape[:2]
        height, width = h - 1 - pic_offset, w - 1 - pic_offset

        fps = vid.get(cv.CAP_PROP_FPS)
        if fps == 0:
            fps = 30

        video = cv.VideoWriter("labeled_video.mp4", cv.VideoWriter_fourcc(*'mp4v'), fps, (w, h), isColor=False) 
        vid.set(cv.CAP_PROP_POS_FRAMES, 0)

        while (True):
            ret, frame = vid.read()

            if not ret:
                break
                
            result, cur_right, cur_left = self.get_img_mask(frame, debug=debug, percent=percent, pic_offset=pic_offset, pixel_range=pixel_range)
            
            presult = np.zeros((h, w), dtype=result.dtype)

            height = h - 1 - pic_offset
            width = w - 1 - pic_offset
            y_index = int(height * percent)

            presult[y_index:height, pic_offset:width] = result

            # Write debug info on video
            if debug:
                right_deg = utils.get_angle((width, pic_offset), (width // 2, pic_offset), cur_right)
                left_deg = utils.get_angle((pic_offset, pic_offset), (width // 2, pic_offset), cur_left)

                if right_deg is None:
                    right_deg = -1
                
                if left_deg is None:
                    left_deg = -1
            
                cv.putText(presult, f"Previous left: {self.prev_left}", (300, 200), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv.putText(presult, f"Previous right: {self.prev_right}", (900, 200), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                cv.putText(presult, f"Current left: {cur_left}", (300, 400), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv.putText(presult, f"Current right: {cur_right}", (900, 400), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                

                cv.putText(presult, f"{right_deg:.1f}", (1300, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv.putText(presult, f"{left_deg:.1f}", (100, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            video.write(presult)
        
        vid.release()
        video.release()
        cv.destroyAllWindows()

    # @param img: Normal image
    # @param debug: Draws lines on image
    # @param percent: Percent of the road to cutoff (top down)
    # @param pixel_range: Range of pixels to check around previous pixel for O(1) lookup
    # @param pic_offset: Pixel offset from edge of image
    # @ret image angles or None
    def get_img_angles(self, img, debug=False, percent=0.0, pixel_range=3, pic_offset=5):
        image, right, left = self.get_img_mask(img, debug=debug, percent=percent, pixel_range=pixel_range, pic_offset=pic_offset)

        if image is None or right is None or left is None:
            return (None, None)

        w = img.shape[1]
        width = w - 1 - pic_offset

        right_deg = utils.get_angle((width, pic_offset), (width // 2, pic_offset), right)
        left_deg = utils.get_angle((pic_offset, pic_offset), (width // 2, pic_offset), left)

        if right_deg is None or left_deg is None:
            return (None, None)

        return (right_deg, left_deg)


    # @param img: Normal image
    # @param debug: Draws lines on image
    # @param percent: Percent of the road to cutoff (top down)
    # @param pixel_range: Range of pixels to check around previous pixel for O(1) lookup
    # @param pic_offset: Pixel offset from edge of image
    # @ret Masked image mage or None
    def get_img_mask(self, img: np.ndarray, debug=False, percent=0.0, pixel_range=3, pic_offset=5):
        if img is None:
            self.error.logger('Error opening image!')
            return None, None, None
        
        # Roi mask for a portion of image
        h, w = img.shape[:2]
        height, width = h - 1 - pic_offset, w - 1 - pic_offset
        y_index = int(height * percent)

        roi_image = img[y_index:height, pic_offset:width]

        # Get hue mask
        image_hsv = utils.convert_bgr_to_hsv(roi_image)
        mask_red = cv.inRange(image_hsv, LOWER_RED, UPPER_RED) 
        result = cv.morphologyEx(mask_red, cv.MORPH_OPEN, KERNEL)

        if self.prev_right is None:
            self.prev_right = (width, height)
        
        if self.prev_left is None:
            self.prev_left = (pic_offset, height)
        
        right = self.lookup_road_coord(result, self.prev_right, True, pic_offset=pic_offset, pixel_range=pixel_range)
        left = self.lookup_road_coord(result, self.prev_left, False, pic_offset=pic_offset, pixel_range=pixel_range)

        self.prev_right = right
        self.prev_left = left

        if not right or not left:
            right = self.vectorized_road_coord(result, True)
            left = self. vectorized_road_coord(result, False)
        
        if debug:
            utils.draw_lines(result, right, left)
        
        return (result, right, left)

    # Vectorized road lookup
    # @param right_side: Determine what side of image
    # @param pic_offset: Offset from edge of photo
    def vectorized_road_coord(self, img, right_side, pic_offset=5):
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
    def lookup_road_coord(self, img, prev_pixel, right_side, pixel_range=6, pic_offset=5):
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