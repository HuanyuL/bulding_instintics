import cv2

cap = cv2.VideoCapture(0)
while True:

    ret, frame = cap.read()
    # print(frame)

    if cv2.waitKey(3) & 0xFF == ord('q'):
        break

    cv2.imshow('frame', frame)

cap.release()
cv2.destroyAllWindows()
