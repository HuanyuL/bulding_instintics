import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ColorFilter:
    def __init__(self):
        self.cv_img = rospy.Subscriber("img_rgb", Image, self.callback)
        self.bridge = CvBridge()
        self.frame = None

    def callback(self, img):
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
            blur = cv2.GaussianBlur(rgb_img, (5, 5), 0)
            # morphology
            kernel = np.ones((3, 3), np.uint8)
            self.frame = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel)
        except CvBridgeError as e:
            print(e)

    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo('Shutting down')

    def run(self):
        cv2.namedWindow("Tracking")
        cv2.createTrackbar("LR", "Tracking", 0, 255, self.nothing)
        cv2.createTrackbar("LG", "Tracking", 0, 255, self.nothing)
        cv2.createTrackbar("LB", "Tracking", 0, 255, self.nothing)
        cv2.createTrackbar("UR", "Tracking", 255, 255, self.nothing)
        cv2.createTrackbar("UG", "Tracking", 255, 255, self.nothing)
        cv2.createTrackbar("UB", "Tracking", 255, 255, self.nothing)

        while not rospy.is_shutdown():
            if self.frame is not None:

                lR = cv2.getTrackbarPos("LR", "Tracking")
                lG = cv2.getTrackbarPos("LG", "Tracking")
                lB = cv2.getTrackbarPos("LB", "Tracking")
                uR = cv2.getTrackbarPos("UR", "Tracking")
                uG = cv2.getTrackbarPos("UG", "Tracking")
                uB = cv2.getTrackbarPos("UB", "Tracking")

                RGBMin = np.array([lR, lG, lB])
                RGBMax = np.array([uR, uG, uB])

                mask = cv2.inRange(self.frame, RGBMin, RGBMax)
                res = cv2.bitwise_and(self.frame, self.frame, mask=mask)

                cv2.imshow("Result view", res)
                cv2.waitKey(1)
            else:
                rospy.loginfo('No frame')
                rospy.sleep(1)

    def nothing(self, x):
        pass


def main():
    rospy.init_node('img_filter')
    color_filter = ColorFilter()
    color_filter.run()
    rospy.spin()


if __name__ == '__main__':
    main()
