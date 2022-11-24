import cv2
import numpy as np

class VisionSliderHelp:
    def __init__(self, filename):
        img_load = cv2.imread(filename)
        # Create a black image, a window
        img = np.zeros((300, 512, 3), np.uint8)
        cv2.namedWindow('image')
        #HSV_img = cv2.cvtColor(img_load, cv2.COLOR_BGR2HSV)
        #filename = "calibration_grid2.jpg"
        #img_load = cv2.imread(filename)

        # create trackbars for color change
        cv2.createTrackbar('UR', 'image', 0, 255, self.nothing)
        cv2.createTrackbar('LR', 'image', 0, 255, self.nothing)
        cv2.createTrackbar('UG', 'image', 0, 255, self.nothing)
        cv2.createTrackbar('LG', 'image', 0, 255, self.nothing)
        cv2.createTrackbar('UB', 'image', 0, 255, self.nothing)
        cv2.createTrackbar('LB', 'image', 0, 255, self.nothing)

        # create switch for ON/OFF functionality
        switch = '0 : OFF \n1 : ON'
        cv2.createTrackbar(switch, 'image', 0, 1, self.nothing)

        while (1):
            cv2.imshow('image', img)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            # get current positions of four trackbar
            ur = cv2.getTrackbarPos('UR', 'image')
            ug = cv2.getTrackbarPos('UG', 'image')
            ub = cv2.getTrackbarPos('UB', 'image')
            lr = cv2.getTrackbarPos('LR', 'image')
            lg = cv2.getTrackbarPos('LG', 'image')
            lb = cv2.getTrackbarPos('LB', 'image')
            s = cv2.getTrackbarPos(switch, 'image')
            lower = (lr, lg, lb)
            upper = (ur, ug, ub)
            black_white_cal_img = cv2.inRange(img_load, lower, upper)
            cv2.imshow("output", black_white_cal_img)
            if s == 0:
                img[:] = 0
            else:
                img[:] = [ub, ug, ur]

    def nothing(self, x):
        pass