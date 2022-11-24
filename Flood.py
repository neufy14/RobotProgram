import numpy as np
import cv2
import time

class Fill:
    def __init__(self, img, seed):
        visitedX = []
        visitedY = []
        fillList = []
        # xCheck = [0, 0, 1, 1, 1, 0, -1, -1, -1]
        # yCheck = [0, 1, 1, 0, -1, -1, -1, 0, 1]
        xCheck = [0, 0, 1, 0, -1]
        yCheck = [0, 1, 0, -1, 0]
        xPos = seed[0]
        yPos = seed[1]
        fillList.append([xPos, yPos])
        # self.testSeed(img, seed)
        black = 127
        white = 255
        count = 0
        # while len(fillList) > 0:
        while count < 5000:
            # print(fillList)
            currentPos = fillList.pop(0)
            xPos = currentPos[0]
            yPos = currentPos[1]
            # print("currentPos = ", currentPos)
            # print(len(img))
            # print(len(img[0]))
            # print(len(fillList))
            if img[currentPos[0]][currentPos[1]] == white:
                img[currentPos[0]][currentPos[1]] = black
                for i in range(1, len(xCheck)):
                    xPos = xPos + xCheck[i]
                    yPos = yPos + yCheck[i]
                    if (xPos >= 0) and (yPos >= 0) and (xPos < len(img)) and (yPos < len(img[0])):
                        # print("positions being added = [", xPos, ", ", yPos, "]")
                        fillList.append([xPos, yPos])
                    xPos = xPos - xCheck[i]
                    yPos = yPos - yCheck[i]
            count += 1
        cv2.circle(img, (seed[1], seed[0]), 2, (0, 0, 0), 1)
        cv2.imshow("image", img)

    def testSeed(self, frame, orgin):
        cv2.circle(frame, (orgin[0], orgin[1]), 2, (0, 0, 0), 1)

filename = "floodFillTest/empty3.jpg"
# filename = "findImagesFolder/opencv_frame_0.png"
seedLoc = [400, 325]
sourceImage = cv2.imread(filename)
sourceImageGray = cv2.cvtColor(sourceImage, cv2.COLOR_RGB2GRAY)
_, sourceImageBW = cv2.threshold(sourceImageGray, 127, 255, cv2.THRESH_BINARY)

Fill(sourceImageBW, seedLoc)
cv2.waitKey(0)
cv2.destroyAllWindows()


