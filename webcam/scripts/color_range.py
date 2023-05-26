import cv2
import numpy as np

# Just dummy function for callbacks from trackbar


def nothing(x):
    pass


# Create a trackbar window to adjust the HSV values
# They are preconfigured for a yellow object
cv2.namedWindow("Tracking")
cv2.createTrackbar("LR", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LG", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LB", "Tracking", 0, 255, nothing)
cv2.createTrackbar("UR", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UG", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UB", "Tracking", 255, 255, nothing)

# video capture
cap = cv2.VideoCapture(2)

# Read test image
while True:
    _, frame = cap.read()

    # blur the image
    blur = cv2.GaussianBlur(frame, (5, 5), 0)

    # morphology
    kernel = np.ones((3, 3), np.uint8)
    morph = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel)

    # Read the trackbar values
    lR = cv2.getTrackbarPos("LR", "Tracking")
    lG = cv2.getTrackbarPos("LG", "Tracking")
    lB = cv2.getTrackbarPos("LB", "Tracking")
    uR = cv2.getTrackbarPos("UR", "Tracking")
    uG = cv2.getTrackbarPos("UG", "Tracking")
    uB = cv2.getTrackbarPos("UB", "Tracking")

    # Create arrays to hold the minimum and maximum HSV values
    hsvMin = np.array([lR, lG, lB])
    hsvMax = np.array([uR, uG, uB])

    # Apply HSV thresholds
    mask = cv2.inRange(morph, hsvMin, hsvMax)

    # Uncomment the lines below to see the effect of erode and dilate
    # mask = cv2.erode(mask, None, iterations=3)
    # mask = cv2.dilate(mask, None, iterations=3)

    # The output of the inRange() function is black and white
    # so we use it as a mask which we AqND with the orignal image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Show the result
    cv2.imshow("Result view", res)

    # Wait for the escape key to be pressed
    key = cv2.waitKey(1)
    if key == 27 or key == ord("q"):
        break

cv2.destroyAllWindows()
