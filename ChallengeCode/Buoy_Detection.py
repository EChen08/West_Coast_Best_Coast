import sys
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
import cv2

green_pixel_means = []
green_sens_means = []
green_sens_angles = []

red_pixel_means = []
red_sens_means = []
red_sens_angles = []

pixel_means = []
sens_means = []
sens_angles = []

next_frames = []
thresh = 100

# returns the x and y pixel position on the sensor
def sensor_position(pix_x, pix_y, res_x, res_y):
    origin = (res_x/2, res_y/2)
    pix_point = (pix_x - origin[0], pix_y - origin[1])
    sensor_pos_x, sensor_pos_y = (pix_point[0] * 0.00368/res_x, -pix_point[1] * 0.00276/res_y)
    return sensor_pos_x, sensor_pos_y

# returns horizontal angles to a point on a camera frame
def angles(x, y):
    horizontal_angle = np.degrees(np.arctan(x/0.00304))
    return horizontal_angle

# appends contour means, pixel position/angles if above threshold, returns number of buoys for each color
def contour_func(contours, img, color:str): 
    ctr_areas = [0]
    ctr_sens_means = [[0,0]]
    ctr_sens_angles = [0]
    ctr_pixel_means = [[0,0]]

    red_buoys = 0
    green_buoys = 0
    for contour in contours:
            ctr_mean = np.mean(contour, axis=0)
            ctr_max = np.max(contour, axis=0)
            ctr_min = np.min(contour, axis=0)
            ctr_diff = ctr_max - ctr_min
            xdiff = ctr_diff[:,0]
            ydiff = ctr_diff[:,1]
            ctr_area = xdiff * ydiff
            if ctr_area >= thresh:
                if color == 'red':
                    print("red detected!")
                    red_buoys += 1
                elif color == 'green':
                    print("green detected!")
                    green_buoys += 1
                else:
                    continue
                if ctr_area >= ctr_areas[0]:
                    ctr_areas.pop(0)
                    ctr_areas.append(ctr_area)                
                    ctr_pixel_means.pop(0)
                    ctr_pixel_means.append(ctr_mean)
                    cntr = (int(ctr_mean[0,0]), int(ctr_mean[0,1]))
                    sens_ctr = (sensor_position(cntr[0], cntr[1], img.shape[1], img.shape[0]))
                    ctr_sens_means.pop(0)
                    ctr_sens_means.append(sens_ctr)
                    ctr_sens_angles.pop(0)
                    ctr_sens_angles.append(angles(sens_ctr[0], sens_ctr[1]))

    next_frames.append(img)
    if ctr_areas != [0] and color == 'red':
        red_sens_means.append(ctr_sens_means[0])
        red_sens_angles.append(ctr_sens_angles[0])
        red_pixel_means.append(ctr_pixel_means[0])
        return red_buoys
    elif ctr_areas != [0] and color == 'green':
        green_sens_means.append(ctr_sens_means[0])
        green_sens_angles.append(ctr_sens_angles[0])
        green_pixel_means.append(ctr_pixel_means[0])
        return green_buoys

    return

def green_update(im): #updates green buoy data
    green_buoys = 0
    
    img = cv2.resize(im, (640, 480)) # resize each image

    imhsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert to HSV
    median_blur = cv2.medianBlur(imhsv, 9) # apply median blur filter to smoothen the HSV image, denoise
    hue = median_blur[:,:,0]
    sat = median_blur[:,:,1]
    val = median_blur[:,:,2]
    
    # thresholding the HSV values to isolate only green color
    img_thresh_hue = np.logical_and(hue>36, hue<86)
    img_thresh_sat = np.logical_and(sat>100, sat<255)
    img_thresh_val = np.logical_and(val>60, val<255)
    img_thresh_HSV = np.logical_and(img_thresh_hue, img_thresh_sat, img_thresh_val)
    box_filt_HSV2 = cv2.boxFilter(img_thresh_HSV.astype(int), -1, (40,40), normalize=False)

    if not img_thresh_HSV.any(): # saturation threshold too high, so reset with accurate saturation threshold to isolate green buoy from image
        box_filt_HSV1 = cv2.boxFilter(imhsv, -1, (9, 9)) # apply median blur filter to smoothen the HSV image, denoise
        hue = box_filt_HSV1[:,:,0]
        sat = box_filt_HSV1[:,:,1]
        val = box_filt_HSV1[:,:,2]

        img_thresh_hue = np.logical_and(hue>36, hue<86)
        img_thresh_sat = np.logical_and(sat>67, sat<255)
        img_thresh_val = np.logical_and(val>60, val<255)
        img_thresh_HSV = np.logical_and(img_thresh_hue, img_thresh_sat, img_thresh_val)

        box_filt_HSV2 = cv2.boxFilter(img_thresh_HSV.astype(int), -1, (40, 40), normalize=False)

    img_threshold = np.argwhere(box_filt_HSV2 > thresh)
    
    if img_threshold.all():
        img8 = box_filt_HSV2 * 255/np.max(box_filt_HSV2)
        img8 = img8.astype(np.uint8)
        thresh8 = thresh * 255/np.max(box_filt_HSV2)
        thresh8 = thresh8.astype(np.uint8)

        retval, img_out = cv2.threshold(img8, thresh8, 255, cv2.THRESH_BINARY)
        if cv2.__version__ == '3.2.0':
            _, contours, hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        else:
            contours, hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # isolate only significant green blobs by creating a minimum area bound that determines whether blob is significant
        green_buoys = contour_func(contours, img, color='green')
    return green_buoys

def red_update(im): # updates red buoy data
    red_buoys = 0
    
    img = cv2.resize(im, (640, 480)) # resize each image

    imhsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert to HSV
    median_blur = cv2.medianBlur(imhsv, 5) # apply median to smoothen the HSV image, denoise
    hue = median_blur[:,:,0]
    sat = median_blur[:,:,1]
    val = median_blur[:,:,2]
    
    # thresholding the HSV values to isolate only red color
    img_thresh_hue = np.logical_or(np.logical_and(hue>=160, hue<=180), np.logical_and(hue>=0, hue<=10))
    img_thresh_sat = np.logical_and(sat>=0, sat<=255)
    img_thresh_val = np.logical_and(val>0, val<255)
    img_thresh_HSV = np.logical_and(img_thresh_hue, img_thresh_sat, img_thresh_val)

    # threshold yields too small area, second chance for the red area to show up
    if np.sum(np.count_nonzero(img_thresh_HSV)) < 40:
        img_thresh_hue = np.logical_or(np.logical_or(np.logical_and(hue>=160, hue<=180), np.logical_and(hue>=0, hue<=10)), np.logical_and(hue>80, hue<160))
        img_thresh_sat = np.logical_and(sat>=0, sat<=38)
        img_thresh_val = np.logical_and(val>=0, val<=255)
        img_thresh_HSV = np.logical_and(img_thresh_hue, img_thresh_sat, img_thresh_val)
    
    box_filt_HSV2 = cv2.boxFilter(img_thresh_HSV.astype(int), -1, (7,7), normalize=False)
    img_threshold = np.argwhere(box_filt_HSV2 >= thresh)

    if img_threshold.all():
        img8 = box_filt_HSV2 * 255/np.max(box_filt_HSV2)
        img8 = img8.astype(np.uint8)
        thresh8 = thresh * 255/np.max(box_filt_HSV2)
        thresh8 = thresh8.astype(np.uint8)

        retval, img_out = cv2.threshold(img8, thresh8, 255, cv2.THRESH_BINARY)
        if cv2.__version__ == '3.2.0':
            _, contours, hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        else:
            contours, hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # isolate only significant red blobs by creating a minimum area bound that determines whether blob is significant
        red_buoys = contour_func(contours, img, color='red')
    return red_buoys

def run(image):
    im = cv2.imread(image)
    red_buoys = red_update(im)
    green_buoys = green_update(im)
    print(red_pixel_means, green_pixel_means)

    pixel_means = [red_pixel_means, green_pixel_means]
    sens_means = [red_sens_means, green_sens_means]
    sens_angles = [red_sens_angles, green_sens_angles]
    if sens_angles[0] == []:
        sens_angles[0] = None
    if sens_angles[1] == []:
        sens_angles[1] = None
    num_buoys = [red_buoys, green_buoys]
    return sens_angles[0], sens_angles[1]