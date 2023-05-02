#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




cap = cv2.VideoCapture(0)
print(cap.isOpened())
      
if  not cap.isOpened():
    print("no webcam")


def show():
    
    rospy.init_node('test', anonymous=False)
    mask_img = rospy.Publisher("/mask_img", Image, queue_size=1)
    resized_img = rospy.Publisher("/resized_img", Image, queue_size=1)
    rate = rospy.Rate(10)
    
    while True:
        ret, frame = cap.read()

        # blur the image
        blur = cv2.GaussianBlur(frame, (5, 5), 1)

        # morphology
        kernel = np.ones((3,3), np.uint8)   
        morph = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel)

        # color detection       
        lower_red = np.array([0, 0, 120])
        upper_red = np.array([130, 90, 255])
        lower_yellow = np.array([0, 137, 180])
        upper_yellow = np.array([137, 255, 255])
        mask_yellow = cv2.inRange(morph, lower_yellow, upper_yellow)
        mask_yellow = cv2.bitwise_and(morph, morph, mask=mask_yellow)

        mask_red = cv2.inRange(morph, lower_red, upper_red)
        mask_red = cv2.bitwise_and(morph, morph, mask=mask_red)

        mask_combined = cv2.bitwise_or(mask_red, mask_yellow)
        
        # get the size of the image
        height = frame.shape[0]
        
        # crop the image
        row_num = 2
        col_num = 8
        low_x = 150
        aspect_ratio = row_num/col_num
        high_x = low_x + (aspect_ratio * height)
        crop = mask_combined[int(low_x):int(high_x), :]


        # split the image in to x rows and y columns
        columns = np.vsplit(crop, row_num)
        cells = []
        for column in columns:
            cells.extend(np.hsplit(column, col_num))
        
        # get the average rgb color of each cell
        cell_colors = []
        for cell in cells:
            avg_color_per_row = np.average(cell, axis=0)
            avg_color = np.average(avg_color_per_row, axis=0)
            avg_color = np.uint8(avg_color)
            cell_colors.append(avg_color)
        
        # assemble the cells into a 2d array
        cell_colors = np.array(cell_colors)
        cell_colors = cell_colors.reshape((row_num, col_num, 3))

        # cv2.imshow("1",cell_colors)
        # cv2.imshow("2",crop)

        cv2.waitKey(1)

        try:
            mask_img.publish(CvBridge().cv2_to_imgmsg(crop, "bgr8"))
            resized_img.publish(CvBridge().cv2_to_imgmsg(cell_colors, "bgr8"))
            rate.sleep()
        except CvBridgeError as e:
            rospy.logerr(e)

        
# cap.release()
if __name__ == '__main__':
    show()
    cap.release()
    cv2.destroyAllWindows