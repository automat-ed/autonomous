#!/usr/bin/env python

import numpy as np
import cv2

def detect_path(image):
    pre_processed_image = pre_process(image)
    canny_edge_image = canny_edge_detect(pre_processed_image)
    ROI_edges = find_ROI(canny_edge_image)
    houghed = hough_lines(ROI_edges)

    cv2.imshow('window', houghed)
    cv2.waitKey(3)



# Remove obstacles, Convert to grey scale and Apply Gaussian blur
def pre_process(image):
    greyscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    kernel_size = 5
    blurred_image = cv2.GaussianBlur(greyscale_image, (kernel_size, kernel_size), 0)
    return blurred_image


# Perform Canny edge detection on pre-processed image
def canny_edge_detect(image):
    min_threshold = 100
    max_threshold = 200
    return cv2.Canny(image, min_threshold, max_threshold)

# Extract robot's ROI - keep only region delimited by vertices
def find_ROI(image):
    lower_left_corner1 = [0, 640]
    upper_left_corner1 = [200, 350]
    upper_right_corner1 = [500, 350]
    lower_right_corner1 = [820, 320]
    
    vertices = np.array([[lower_left_corner1, upper_left_corner1, upper_right_corner1, lower_right_corner1]], dtype=np.int32)

    # Blank mask
    mask = np.zeros_like(image)

    # print(image.shape)
    # channel_count  = image.shape[2]
    # mask_color = (255, ) * channel_count

    # Fill ROI with white pixels
    cv2.fillPoly(mask, vertices, 255)

    # Return image pixels which correspond to non-zero pixels in the mask
    return cv2.bitwise_and(image, mask)


# Apply Hough Transform
def hough_lines(image):
    rho = 2
    theta = np.pi/180
    threshold = 30
    min_line_len = 40 
    max_line_gap = 5

    lines = cv2.HoughLinesP(image, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((*image.shape, 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img


# Extrapolates lines from houghed-transformed image
def draw_lines(image, lines, color=(0, 0, 255), thickness=2):
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2, = line.reshape(4)
            cv2.line(image, (x1, y1), (x2, y2), color, thickness)