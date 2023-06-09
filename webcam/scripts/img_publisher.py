#!/usr/bin/env python2

from __future__ import division
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray


class Capture:

    def __init__(self):

        self.crop_img = rospy.Publisher("/crop_rgb", Image, queue_size=1)
        self.img_rgb = rospy.Publisher("img_rgb", Image, queue_size=1)
        self.resize_img = rospy.Publisher("/turtle_view", Image, queue_size=1)
        self.rgb_msg = rospy.Publisher(
            "/view", Float64MultiArray, queue_size=1)
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.rate = rospy.Rate(10)

    def run(self):
        if not self.cap.isOpened():
            rospy.logerr("no camera detected")
            self.on_shutdown()
        else:
            rospy.loginfo("Publishing images")
            while not rospy.is_shutdown():
                ret, frame = self.cap.read()
                if not ret:
                    rospy.logerr("Failed to capture frame")
                    break
                try:
                    img_rgb = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    crop, resize, flattened = self.analyse_frame(frame)
                    crop_msg = self.bridge.cv2_to_imgmsg(crop, "bgr8")
                    resize_msg = self.bridge.cv2_to_imgmsg(resize, "bgr8")
                    self.crop_img.publish(crop_msg)
                    self.resize_img.publish(resize_msg)
                    self.rgb_msg.publish(flattened)
                    self.img_rgb.publish(img_rgb)
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
        lower_red = np.array([0, 0, 79])
        upper_red = np.array([65, 51, 255])
        
        lower_green = np.array([0, 72, 0])
        upper_green = np.array([240, 255, 39])

        lower_blue = np.array([199, 0, 0])
        upper_blue = np.array([255, 203, 197])

        mask_green = cv2.inRange(morph, lower_green, upper_green)

        mask_red = cv2.inRange(morph, lower_red, upper_red)

        mask_blue = cv2.inRange(morph, lower_blue, upper_blue)

        mask_combined = cv2.bitwise_or(mask_red, mask_blue)
        mask_combined = cv2.bitwise_or(mask_combined, mask_green)
        
        filtered_img = cv2.bitwise_and(morph, morph, mask=mask_blue)
        
        filtered_img[np.where((filtered_img != [0, 0, 0]).all(axis=2))] = [255, 0, 0]

        # get the size of the image
        height, width = frame.shape[0], frame.shape[1]

        # crop the image and keep the maximum width
        row_num = 2
        col_num = 4
        low_y = 0
        aspect_ratio = row_num/col_num
        high_y = low_y + (aspect_ratio * width)
        crop = filtered_img[int(low_y):int(high_y), :]

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
        red, green, blue = cell_colors[:, :, 2], cell_colors[:, :, 1], cell_colors[:, :, 0]

        # reverse the order of the rows
        turtle_view = blue[1, :].reshape(1, 4)
        flattened = turtle_view.flatten()
        # use list conprehension to divide by 255
        view_msg = Float64MultiArray()

        if cell_colors.ndim > 1:
            flattened = turtle_view.flatten()

        # Convert the ndarray elements to float and assign to the message data
            flattened = [x/255 for x in flattened]
            flattened.append(1)
            view_msg.data = [float(value) for value in flattened]
            print(view_msg)

        return crop, cell_colors, view_msg


def main():
    rospy.init_node('test', anonymous=False)
    cap = Capture()
    try:
        cap.run()
    except KeyboardInterrupt:
        print("shutting down")


if __name__ == '__main__':
    main()
