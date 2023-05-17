import cv2
import numpy as np

# Just dummy function for callbacks from trackbar


def nothing(x):
    pass


# Create a trackbar window to adjust the HSV values
# They are preconfigured for a yellow object
cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LS", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 0, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)

# video capture
cap = cv2.VideoCapture(0)

# Read test image
while True:
    _, frame = cap.read()

    # blur the image
    blur = cv2.GaussianBlur(frame, (5, 5), 0)

    # morphology
    kernel = np.ones((3, 3), np.uint8)
    morph = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel)

    # Read the trackbar values
    lh = cv2.getTrackbarPos("LH", "Tracking")
    ls = cv2.getTrackbarPos("LS", "Tracking")
    lv = cv2.getTrackbarPos("LV", "Tracking")
    uh = cv2.getTrackbarPos("UH", "Tracking")
    us = cv2.getTrackbarPos("US", "Tracking")
    uv = cv2.getTrackbarPos("UV", "Tracking")

    # Create arrays to hold the minimum and maximum HSV values
    hsvMin = np.array([lh, ls, lv])
    hsvMax = np.array([uh, us, uv])

    # Apply HSV thresholds
    mask = cv2.inRange(morph, hsvMin, hsvMax)

    # Uncomment the lines below to see the effect of erode and dilate
    # mask = cv2.erode(mask, None, iterations=3)
    # mask = cv2.dilate(mask, None, iterations=3)

    # The output of the inRange() function is black and white
    # so we use it as a mask which we AND with the orignal image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Show the result
    cv2.imshow("Result view", res)

    # Wait for the escape key to be pressed
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
