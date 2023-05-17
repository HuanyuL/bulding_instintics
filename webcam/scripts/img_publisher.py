#!/usr/bin/env python2

from __future__ import division
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class Capture:

    def __init__(self):

        # self.vec_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
        self.crop_img = rospy.Publisher("/crop_img", Image, queue_size=1)
        self.resize_img = rospy.Publisher("/resize_img", Image, queue_size=1)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.rate = rospy.Rate(10)

    def run(self):
        if not self.cap.isOpened():
            rospy.loginfo("no camera detected")
        else:
            rospy.loginfo("Publishing images")
            while not rospy.is_shutdown():
                ret, frame = self.cap.read()
                if not ret:
                    rospy.logerr("Failed to capture frame")
                    continue
                try:
                    crop, resize = self.analyse_frame(frame)
                    crop_msg = self.bridge.cv2_to_imgmsg(crop, "bgr8")
                    resize_msg = self.bridge.cv2_to_imgmsg(resize, "bgr8")
                    # twist = Twist()
                    # twist.linear.x = 0.1
                    # twist.angular.z = msg
                    # self.vec_pub.publish(twist)
                    self.crop_img.publish(crop_msg)
                    self.resize_img.publish(resize_msg)
                    self.rate.sleep()
                except CvBridgeError as e:
                    self.on_shutdown()
                    rospy.logerr(e)
                    break
            self.on_shutdown()

    def on_shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()

    @staticmethod
    def analyse_frame(frame):

        # blur the image
        blur = cv2.GaussianBlur(frame, (5, 5), 1)

        # morphology
        kernel = np.ones((3, 3), np.uint8)
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

        return crop, cell_colors


def main():
    rospy.init_node('test', anonymous=False)
    cap = Capture()
    try:
        cap.run()
    except KeyboardInterrupt:
        print("shutting down")


if __name__ == '__main__':
    main()
