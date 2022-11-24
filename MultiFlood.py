import numpy as np
import cv2
import threading
import time

class Fill:
    def __init__(self, image, seedLocs):
        # print(seedLocs[1])
        # self.testSeed(image, seedLocs[1])
        self.sourceImg = image
        numSeedLocs = len(seedLocs)
        fillThreads = []*numSeedLocs
        color = np.zeros(numSeedLocs)
        darkColor = 50
        lightColor = 200
        if numSeedLocs > 1:
            step = (lightColor - darkColor) / (numSeedLocs - 1)
        else:
            step = (lightColor - darkColor)
        for i in range(numSeedLocs):
            color = darkColor + (i * step)
            tempThread = threading.Thread(target=self.queueForStorage, args=(image, seedLocs[i], i, color))
            fillThreads.append(tempThread)
        for i in range(numSeedLocs):
            fillThreads[i].start()
        for i in range(numSeedLocs):
            fillThreads[i].join()
        print("done all!")
        cv2.imshow("final image", self.sourceImg)

    def queueForStorage(self, img, seed, whichThread, currentColor):
        print("seed = ", seed)
        fillList = []
        step = 1
        # xCheck = [0, 0, 1, 1, 1, 0, -1, -1, -1]
        # yCheck = [0, 1, 1, 0, -1, -1, -1, 0, 1]
        xCheck = [0, 0, step, step, step, 0, -step, -step, -step]
        yCheck = [0, step, step, 0, -step, -step, -step, 0, step]
        # xCheck = [0, 0, 1, 0, -1]
        # yCheck = [0, 1, 0, -1, 0]
        xPos = seed[0]
        yPos = seed[1]
        fillList.append([xPos, yPos])
        # self.testSeed(img, seed)
        # color = [100, 200]

        # black = colors[whichThread]
        white = 255
        count = 0
        while len(fillList) > 0:
        # while count < 5000:
        #     print("thread number = ", whichThread)
            currentPos = fillList.pop(0)
            xPos = currentPos[0]
            yPos = currentPos[1]
            # print("currentPos = ", currentPos)
            # print(len(fillList))
            if self.sourceImg[int(currentPos[0])][int(currentPos[1])] == white:
                self.sourceImg[int(currentPos[0])][int(currentPos[1])] = currentColor
                for i in range(0, len(xCheck)):
                    xPos = xPos + xCheck[i]
                    yPos = yPos + yCheck[i]
                    # print("positions being added = [", xPos, ", ", yPos, "]")
                    if xPos >= 0 and yPos >= 0 and xPos < len(img) and yPos < len(img[0]):
                        fillList.append([xPos, yPos])
                    xPos = xPos - xCheck[i]
                    yPos = yPos - yCheck[i]
            count += 1
        cv2.circle(img, (int(seed[1]), int(seed[0])), 2, (0, 0, 0), 1)
        print("current color = ", currentColor)
        print("done ", whichThread)

    def testSeed(self, frame, orgin):
        cv2.circle(frame, (orgin[1], orgin[0]), 2, (0, 0, 0), 1)
        cv2.imshow("test", frame)

# filename = "floodFillTest/empty3.jpg"
# # filename = "findImagesFolder/opencv_frame_0.png"
# seedLoc = [[325, 295], [325, 450]]
# sourceImage = cv2.imread(filename)
# sourceImageGray = cv2.cvtColor(sourceImage, cv2.COLOR_RGB2GRAY)
# _, sourceImageBW = cv2.threshold(sourceImageGray, 127, 255, cv2.THRESH_BINARY)
# Fill(sourceImageBW, seedLoc)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


