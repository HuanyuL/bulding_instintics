import cv2
import numpy as np
np.set_printoptions(threshold=np.inf)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

while True:
    ret, frame = cap.read()
    if not ret:
        break
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

    lower_blue = np.array([225, 141, 0])
    upper_blue = np.array([255, 201, 255])

    mask_green = cv2.inRange(morph, lower_green, upper_green)

    mask_red = cv2.inRange(morph, lower_red, upper_red)

    mask_blue = cv2.inRange(morph, lower_blue, upper_blue)

    mask_combined = cv2.bitwise_or(mask_red, mask_blue)
    mask_combined = cv2.bitwise_or(mask_combined, mask_green)
    
    filtered_img = cv2.bitwise_and(morph, morph, mask=mask_combined)
    # get the size of the image
    height, width = frame.shape[0], frame.shape[1]

    # crop the image
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
    red, green, blue = cell_colors[:, :, 0], cell_colors[:, :, 1], cell_colors[:, :, 2]
    # reverse the order of the rows
    turtle_view = blue[1, :].reshape(1, 4)
    flattened = turtle_view.flatten()
    # use list conprehension to divide by 255
    flattened = [x/255 for x in flattened]
    print(flattened)
    cv2.imshow('view', turtle_view)
    # cv2.imshow('view2', crop)
    if cv2.waitKey(3) & 0xFF == ord('q'):
        break

cap.release()
