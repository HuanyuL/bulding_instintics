import cv2
import numpy as np

cap = cv2.VideoCapture(0)
while True:

    ret, frame = cap.read()
    # print(frame)
    
    if cv2.waitKey(3) & 0xFF == ord('q'):
        break

    cap.release()
    cv2.imshow('frame',frame)


cv2.destroyAllWindows()