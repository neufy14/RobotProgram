import cv2
import numpy as np
import TeachObject
import math
import time
from matplotlib import pyplot as plt

class compareImages:
    def __init__(self, sourceImage, compareImage, type, format):
        self.sourceHeight = len(sourceImage[0])
        self.sourceWidth = len(sourceImage)
        self.compareHeight = len(compareImage[0])
        self.compareWidth = len(compareImage)

        # print("sourceHeight = ", self.sourceHeight)
        # print("sourceWidth = ", self.sourceWidth)
        # print("compareHeight = ", self.compareHeight)
        # print("compareWidth = ", self.compareWidth)

        imgDif = np.zeros((self.sourceHeight, self.sourceWidth, 3))
        batch = np.zeros((self.sourceHeight, self.sourceWidth, 3))

        self.heightStride = int(self.sourceHeight / 4)
        self.widthStride = int(self.sourceWidth / 4)
        # self.heightStride = 10
        # self.widthStride = 10
        if format == 'rgb':
            x, y = self.histoMatch(sourceImage, compareImage, type)
        elif format == 'hsv':
            sourceImageHsv = cv2.cvtColor(sourceImage, cv2.COLOR_BGR2HSV)
            compareImageHsv = cv2.cvtColor(compareImage, cv2.COLOR_BGR2HSV)
            x, y = self.histoMatch(sourceImageHsv, compareImageHsv, type)
        cv2.rectangle(compareImage, (x, y),
                      ((x + self.sourceHeight), (y + self.sourceWidth)),
                      (0, 0, 255), 2)
        cv2.imshow("hist compare", compareImage)
        cv2.waitKey(type)

    def histoMatch(self, source, compare, cnt):
        # self.applyFilter(compare)
        # self.applyFilter3D(compare)
        colors = ('r', 'g', 'b')
        start = [0, 257, 514]
        end = [256, 513, 770]
        # start = [0, 0, 0]
        # end = [256, 256, 256]
        totalSourceHist = np.zeros(770)
        allSourceHist = np.zeros([3, 256])
        for i, color in enumerate(colors):
            sourceHist = cv2.calcHist([source], [i], None, [256], [0, 256])
            allSourceHist[i] = sourceHist[:, 0]
            # print("sourceHist")
            # print(sourceHist)
            # totalSourceHist[0:256] = sourceHist[:, 0]
            # print("start = ", start[i])
            # print("end = ", end[i])
            # print("size = ", end[i] - start[i])
            totalSourceHist[start[i]:end[i]] = sourceHist[:, 0]
        # print("total source hist")
        # print(totalSourceHist)
        # print("sourceHist lenght = ", len(sourceHist))
        # print("sourceHIst = ", sourceHist)
        # print("hist sum = ", sum(sourceHist[0]))
        i = 0
        j = 0
        minHistDif = float('inf')
        minHistDifLoc = (0, 0)
        histDifCount = 0
        while (i + self.sourceWidth) < self.compareWidth:
            while (j + self.sourceHeight) < self.compareHeight:
                batch = compare[i:(i+self.sourceWidth), j:(j+self.sourceHeight), :]
                histDif = np.zeros([3,256])
                totalBatch = np.zeros(770)
                histDifSum = [0,0,0]
                # sumBatchHist = 0
                # sumSourceHist = 0
                for k, color in enumerate(colors):
                    batchHist = cv2.calcHist([batch], [k], None, [256], [0, 256])
                    # print("batch sum = ", sum(batchHist[:, 0]))
                    # print("source sum = ", sum(allSourceHist[k]))
                    histDif[k] = abs(batchHist[:, 0] - allSourceHist[k])
                    # print("len(histDif) = ", len(histDif))
                    # print("len(histDif[0]) = ", len(histDif[0]))
                    histDifSum[k] = sum(histDif[k, :])
                    totalBatch[start[k]:end[k]] = batchHist[0, :]

                # print("sum total hist dif = ", sum(abs(totalHistDif)))
                if sum(histDifSum) != 0:
                    histDifCount += 1
                if sum(histDifSum) < minHistDif:
                    minHistDif = sum(histDifSum)
                    minHistDifLoc = (i, j)

                j += self.widthStride
            j = 0
            i += self.heightStride

        x = minHistDifLoc[1]
        y = minHistDifLoc[0]

        # cv2.rectangle(compare, (x, y),
        #               ((x + self.sourceHeight), (y + self.sourceWidth)),
        #               (0, 0, 255), 2)
        # cv2.imshow("hist compare", compare)
        # cv2.waitKey(cnt)
        return (x, y)

    def exactMatch(self):
        i = 0
        j = 0
        minImgDif = float('inf')
        minImageDifLoc = (0, 0)
        # cv2.imshow("closest match", sourceImage)
        while (i + self.sourceWidth) < self.compareWidth:
            while (j + self.sourceHeight) < self.compareHeight:
                batch = self.compareImage[i:(i+self.sourceWidth), j:(j+self.sourceHeight), :]
                # print("i = ", i)
                # print("sourceWidth = ", sourceWidth)
                # print("batch size = ", len(batch), " x ", len(batch[0]))
                # print("sourceImage size = ", len(sourceImage), " x ", len(sourceImage[0]))
                imgDif = self.sourceImage - batch
                # imgDif = sourceImage[:][:][:] - compareImage[i:(i + sourceHeight)][j:(j + sourceWidth)][:]
                if sum(sum(sum(imgDif))) < minImgDif:
                    minImgDif = sum(sum(sum(imgDif)))
                    print("minImgDif = ", minImgDif)
                    minImageDifLoc = (i, j)
                j += self.widthStride
                # print("i = ", i)
                # print("j = ", j)
                # cv2.rectangle(compareImage, (minImageDifLoc[1], minImageDifLoc[0]),
                #               ((minImageDifLoc[1] + sourceHeight), (minImageDifLoc[0] + sourceWidth)),
                #               (0, 0, 255), 2)
                cache = self.compareImage.copy()
                cv2.rectangle(cache, (j, i),
                              ((j + self.sourceHeight), (i + self.sourceWidth)),
                              (0, 0, 255), 2)
                cv2.imshow("source image", self.sourceImage)
                cv2.imshow("compare image", self.compareImage)
                cv2.imshow("intermediate image", cache)
                cv2.imshow("image difference", imgDif)
                # time.sleep(1)
                cv2.waitKey(1)
                # cv2.destroyWindow("intermediate image")
            i += self.heightStride
            j = 0
        print("i = ", i)
        print("j = ", j)
        print("minImageDefLoc = ", minImageDifLoc)
        # closestMatch = compareImage[minImageDifLoc[0]:(minImageDifLoc[0] + sourceHeight),
        #                minImageDifLoc[1]:(minImageDifLoc[1] + sourceWidth)]

        closestMatch = self.compareImage[minImageDifLoc[0]:(minImageDifLoc[0] + self.sourceWidth),
                       minImageDifLoc[1]:(minImageDifLoc[1] + self.sourceHeight)]

        cvClosestMatch = np.array(closestMatch).astype(np.uint8)
        cvClosestMatch = cvClosestMatch[:, :, ::-1].copy().astype(np.uint8)
        # cv2.imshow("closest match", closestMatch)
        cv2.rectangle(self.compareImage, (minImageDifLoc[1], minImageDifLoc[0]), ((minImageDifLoc[1]+self.sourceHeight), (minImageDifLoc[0]+self.sourceWidth)),
                      (0, 0, 255), 2)
        cv2.imshow("full image", self.compareImage)
        print("finished")
        cv2.waitKey(1)

    def applyFilter3D(self, img):
        # filter = [[[0, 1, 0],
        #            [0, 1, 0],
        #            [0, 1, 0]],
        #           [[0, 1, 0],
        #            [0, 1, 0],
        #            [0, 1, 0]],
        #           [[0, 1, 0],
        #            [0, 1, 0],
        #            [0, 1, 0]]]

        # filterLayer = [[-1, 0, 1],
        #               [-2, 0, 2],
        #               [-1, 0, 1]]

        filterLayer = [[-0, 1, 2],
                       [-1, 0, 1],
                       [-2, -1, 0]]

        filter = [filterLayer, filterLayer, filterLayer]

        batch = [[0, 0, 0],
                 [0, 0, 0],
                 [0, 0, 0]]
        # filteredImg = np.zeros([len(img), len(img[0]), len(img[0][0])])
        filteredImg = np.zeros([len(img), len(img[0])], dtype=np.uint8)
        i = 1
        j = 1
        step = 3
        # for i in range(1, (len(img)-1)):
        #     for j in range(1, (len(img[0])-1)):
        while i < (len(img) - 1):
            while j < (len(img[0]) - 1):
                # print("len(img[0][0] = ", len(img[0][0]))
                # print("-------------------------------------------------")
                convult = 0
                for k in range(0, len(img[0][0])):
                    batch = img[(i - 1):(i + 2), (j - 1):(j + 2), k]
                    product = batch * filterLayer
                    convult = sum(sum(product)) + convult

                # batch = img[(i - 1):(i + 2), (j - 1):(j + 2), :]
                # product = batch * filterLayer
                # convult = sum(sum(sum(product))) / ((len(filter)+len(filter[0]) * 3) * (len(filter)+len(filter[0]) * 3))

                value = 0.4 * j
                print("filtered image value = ", convult / (len(filter)+len(filter[0]) * 3))
                # print("value = ", value)
                # filteredImg[i][j] = convult / (len(filter)+len(filter[0]) * 3)
                # filteredImg[i-1:i+1, j-1:j+1] = convult / (len(filter) + len(filter[0]) * 3)
                # filteredImg[i][j] = value
                if convult < 0:
                    convult = 0
                if convult > 255:
                    convult = 255
                filteredImg[i - 1:i + 1, j - 1:j + 1] = convult


                cache = img.copy()
                cv2.rectangle(cache, ((j - 1), (i - 1)),
                              ((j + 2), (i + 2)),
                              (0, 0, 255), 2)
                # cv2.rectangle(filteredImg, (((i-1), (j-1)), ((i+2), (j+2))), (0,0,255), 2)
                cv2.imshow("vertical filter", filteredImg)
                cv2.imshow("cache", cache)
                cv2.waitKey(1)
                j += step
            j = 1
            i += step
        cv2.waitKey(0)

    def applyFilter(self, img):
        filter = [[0, 1, 0],
                  [0, 1, 0],
                  [0, 1, 0]]

        batch = [[0, 0, 0],
                 [0, 0, 0],
                 [0, 0, 0]]
        filteredImg = np.zeros([len(img), len(img[0]), len(img[0][0])])
        i = 1
        j = 1
        step = 3
        # for i in range(1, (len(img)-1)):
        #     for j in range(1, (len(img[0])-1)):
        while i < (len(img)-1):
            while j < len(img[0])-1:
                # print("len(img[0][0] = ", len(img[0][0]))
                print("-------------------------------------------------")
                for k in range(0, len(img[0][0])):
                    batch = img[(i - 1):(i + 2), (j - 1):(j + 2), k]
                    filteredImg[(i - 1):(i + 2), (j - 1):(j + 2), k] = (batch * filter)
                    # print("k = ", k)
                    # print("i = ", i)
                    # print("j = ", j)
                    # print("img")
                    # print(img[3:6, 0:3, k])
                    # print(img[(i-1):(i+2), (j-1):(j+2), k])
                    print("filter")
                    print(filter)
                    print("batch")
                    print(batch)
                    print("output")
                    print(filteredImg[(i - 1):(i + 2), (j - 1):(j + 2), k])
                cache = img.copy()
                cv2.rectangle(cache, ((j-1), (i-1)),
                              ((j+2), (i+2)),
                              (0, 0, 255), 2)
                # cv2.rectangle(filteredImg, (((i-1), (j-1)), ((i+2), (j+2))), (0,0,255), 2)
                cv2.imshow("vertical filter", filteredImg)
                cv2.imshow("cache", cache)
                cv2.waitKey(1)
                j += step
            j = 1
            i += step
        cv2.waitKey(0)

# filename = "findImagesFolder/opencv_frame_0.png"
# print("start")
# TeachObject.ObjectAssist(filename)
# print("past teach object")
# fullImage = cv2.imread(filename)

saveFilename = "findImagesFolder/sourceImage1.png"
vid = cv2.VideoCapture(0)
while True:
    _, img = vid.read()
    cv2.imshow("live image", img)
    if cv2.waitKey(1) & 0xFF == ord('c'):
        cv2.destroyWindow("live image")
        break
cv2.imwrite(saveFilename, img)
TeachObject.ObjectAssist(saveFilename)
print("past teach object")
fullImage = cv2.imread(saveFilename)

while True:
    print("new frame")
    _, frame = vid.read()
    cv2.imshow("raw frame", frame)
    cv2.waitKey(1)
    compareImages(TeachObject.ObjectAssist.croppedRawImg, frame, 1, 'rgb')
    print("past compare image")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("hello?")
        break


#test case against source image
compareImages(TeachObject.ObjectAssist.croppedRawImg, fullImage, 0)
