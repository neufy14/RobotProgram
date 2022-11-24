import cv2
import numpy as np
import threading
from tkinter import *
import tkinter.ttk as ttk
import TeachObject
import time
import MultiFlood

class CreateSegmentations:
    def __init__(self):
        # guiThread = threading.Thread(target=self.guiMethod)
        # guiThread.start()
        self.maxContours = 0
        self.camera = cv2.VideoCapture(0)
        self.root = Tk()
        self.root.title("Camera Controller")
        self.root.geometry("500x400")
        self.teachObjectButton = Button(self.root, text="Teach", command=lambda: self.selectImage())
        # self.teachObjectButton.grid(column=5, row=1)
        # self.teachObjectButton = Button(self.root, text="Teach", command=lambda: self.selectImageCheat())
        self.teachObjectButton.grid(column=5, row=1)
        self.chooseImage = ttk.Combobox(self.root)
        self.chooseImage.grid(column=1, row=1)
        self.applyMaskButton = Button(self.root, text="Apply Mask", command=lambda: self.applyMask(1))
        self.applyMaskButton.grid(column=10, row=1)
        self.marginSlider = Scale(self.root, from_=5, to=50, orient=HORIZONTAL, length=300)
        self.marginSlider.set(10)
        self.marginSlider.grid(column=1, row=4)
        self.marginLabel = Label(self.root, text="Margin")
        self.marginLabel.grid(column=10, row=4)
        self.upperCanny = Scale(self.root, from_=0, to=255, orient=HORIZONTAL, length=300)
        self.upperCanny.grid(column=1, row=5)
        self.upperCanny.set(200)
        self.lowerCanny = Scale(self.root, from_=0, to=255, orient=HORIZONTAL, length=300)
        self.lowerCanny.grid(column=1, row=6)
        self.lowerCanny.set(100)
        self.emptyButton = Button(self.root, text="Empty Mask", command=lambda: self.createEmptyMask())
        self.emptyButton.grid(column=15, row=1)
        cameraThread = threading.Thread(target=self.cameraCapture)
        cameraThread.start()
        self.taughtImg = False
        self.emptyMaskExist = False
        self.root.mainloop()

    def cameraCapture(self):
        print("camera on")
        self.img_counter = 0
        self.emptyCount = 0
        self.imagesAvailable = np.array(0, dtype=object)
        while True:
            _, frame = self.camera.read()
            cv2.imshow("frame", frame)
            k = cv2.waitKey(1)
            if k % 256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                break
            elif k % 256 == 32:
                # SPACE pressed
                img_name = "findImagesFolder/opencv_frame_{}.png".format(self.img_counter)
                cv2.imwrite(img_name, frame)
                print("{} written!".format(img_name))
                self.img_counter += 1
                self.imagesAvailable = [0]*self.img_counter
                if self.img_counter > 1:
                    for i in range(0, self.img_counter):
                        self.imagesAvailable[i] = "Image " + str(i+1)
                else:
                    self.imagesAvailable = "Image 1"
                self.chooseImage['values'] = self.imagesAvailable
            elif k % 256 == 101:
                # e pressed - empty mask image
                self.emptyButton.configure(bg="green")
                self.emptyMaskExist = True
                self.empty_img_name = "findImagesFolder/empty_mask{}.png".format(self.emptyCount)
                cv2.imwrite(self.empty_img_name, frame)
                print("{} written!".format(self.empty_img_name))
                self.emptyCount += 1
            if self.img_counter == 0:
                self.chooseImage.configure(state=DISABLED)
            elif self.img_counter > 0:
                self.chooseImage.configure(state=NORMAL) #, values=str(self.imagesAvailable)
        self.camera.release()
        cv2.destroyAllWindows()

    def selectImage(self):
        self.taughtImg = True
        imgNum = self.chooseImage.current()
        filename = "findImagesFolder/opencv_frame_" + str(imgNum) + ".png"
        print("filename = ", filename)
        TeachObject.ObjectAssist(filename)

    def applyMask(self, whatDo):
        print("apply the mask from the teach function")
        if self.taughtImg:
            colors = TeachObject.ObjectAssist.center
            self.black_white = TeachObject.ObjectAssist.black_on_white
            color_loc = TeachObject.ObjectAssist.center_loc
            if whatDo == 1:
                showMaskVideoThread = threading.Thread(target=self.showMaskVideo, args=(colors, color_loc))
                showMaskVideoThread.start()
            elif whatDo == 2:
                showMaskVideoThread = threading.Thread(target=self.showMaskImg, args=(colors, color_loc))
                showMaskVideoThread.start()
        else:
            errorMessage = Toplevel()
            errorMessage.geometry("200x100")
            errorMessage.title("Error")
            errorMessageLabel = Label(errorMessage, text="Nothing Taught Yet")
            errorMessageLabel.pack()

    def showMaskVideo(self, allColor, colorInterest):
        print("allColor[colorInterest] = ", allColor[colorInterest])
        numItms = 10
        random_colors = np.empty([numItms, 3])
        for i in range(numItms):
            random_colors[i] = list(np.random.choice(range(255), size=3))
        while True:
            convertHSV = True
            upperMax = 255
            lowerMin = 0
            upper = np.array([0,0,0])
            lower = np.array([0,0,0])
            upperMargin = np.array([0,0,0])
            lowerMargin = np.array([0,0,0])
            for i in range(3):
                upperMargin[i] = (upperMax - allColor[colorInterest][i]) * (self.marginSlider.get() / 100)
                lowerMargin[i] = (allColor[colorInterest][i] - lowerMin) * (self.marginSlider.get() / 100)
                if i < 4:
                    upper[i] = allColor[colorInterest][i] + upperMargin[i]
                    lower[i] = allColor[colorInterest][i] - lowerMargin[i]
                else:
                    upper[i] = allColor[colorInterest][i] + 10
                    lower[i] = allColor[colorInterest][i] - 10

            _, frame = self.camera.read()
            upperEdge = self.upperCanny.get()
            lowerEdge = self.lowerCanny.get()
            edgeImg = cv2.Canny(frame, lowerEdge, upperEdge)
            if convertHSV:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            intermediate = cv2.inRange(frame, lower, upper)
            cv2.imshow("mask frame", intermediate)
            contours, hierarchy = cv2.findContours(intermediate,
                                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if convertHSV:
                frame = cv2.cvtColor(frame, cv2.COLOR_HSV2RGB)
            self.determineContour(contours, frame, random_colors)
            # cv2.imshow("edge image", edgeImg)
            k = cv2.waitKey(1)
            if k % 256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                break
            elif k == ord('g'):
                # g pressed
                img_name = "maskImagesFolder/opencv_frame_{}.png".format(self.img_counter)
                cv2.imwrite(img_name, frame)
        self.camera.release()
        cv2.destroyAllWindows()

    def determineContour(self, contours, frame, random_colors):
        colorCount = 0
        nonObjectCount = 0
        print("num contours = ", len(contours))
        for c in contours:
            self.getCircularity(c)
            # check if contour touches edge
            rectBoxStartX, rectBoxStartY, rectBoxW, rectBoxH = cv2.boundingRect(c)

            # if rectBoxStartX > 0 and rectBoxStartY > 0 and (rectBoxStartX+rectBoxW) <= len(frame[0]) and \
            #         (rectBoxStartY+rectBoxH) <= len(frame):
            # print("max area = ", TeachObject.ObjectAssist.maxArea)
            cv2.drawContours(frame, c, -1, (0, 255, 0), 3)
            if (cv2.contourArea(c) > TeachObject.ObjectAssist.maxArea * 0.6) and \
                    (cv2.contourArea(c) < TeachObject.ObjectAssist.maxArea * 1.4) and \
                    (self.getCircularity(c) > TeachObject.ObjectAssist.Circularity * 0.8):
                cv2.drawContours(frame, c, -1, (int(random_colors[colorCount][0]), int(random_colors[colorCount][1]),
                                                int(random_colors[colorCount][2])), 3)
                colorCount += 1
            elif cv2.contourArea(c) > TeachObject.ObjectAssist.maxArea * 1.4:
                print("area = ", cv2.contourArea(c))
                cv2.drawContours(frame, c, -1,
                                 (int(random_colors[colorCount][0]), int(random_colors[colorCount][1]),
                                  int(random_colors[colorCount][2])), 3)
                xStart, yStart, xEnd, yEnd = self.splitContour(c, nonObjectCount, frame)
                cv2.rectangle(frame, (xStart, yStart), (xEnd, yEnd), (0, 255, 0), 2)
                nonObjectCount += 1
        if nonObjectCount > self.maxContours:
            self.maxContours = nonObjectCount
        imgViewNum = nonObjectCount
        for i in range(nonObjectCount, self.maxContours):
            imgTitle = "distance to edge color " + str(i)
            imgTitle2 = "black white threshold " + str(i)
            # imgTitle3 = "watershed marker " + str(i)
            cv2.destroyWindow(imgTitle)
            cv2.destroyWindow(imgTitle2)
            # cv2.destroyWindow(imgTitle3)
        cv2.imshow("Contour View", frame)

    def edgeDetector(self):
        while True:
            _, frame = self.camera.read()
            upperEdge = self.upperCanny.get()
            lowerEdge = self.lowerCanny.get()
            edgeImg = cv2.Canny(frame, lowerEdge, upperEdge)
            cv2.imshow("edge image", edgeImg)
            k = cv2.waitKey(1)
            if k % 256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                break
        self.camera.release()
        cv2.destroyAllWindows()

    def splitContour(self, contour, contNum, originalImg):
        x, y, w, h = cv2.boundingRect(contour)
        extraMargin = 20
        if (x-extraMargin) < 0 or (y-extraMargin) < 0 or (x + w + (2*extraMargin)) >= len(originalImg[0]) or (y + h + extraMargin) >= len(originalImg):
            print("touching edge")
        else:
            # emptyBoundRect = np.empty([(x+w), (y+h)])
            emptyBoundRect = np.zeros([h+(2*extraMargin), w+(2*extraMargin)])
            maxEdgeDist = 0
            # print("x = ", x)
            # print("y = ", y)
            # print("w = ", w)
            # print("h = ", h)
            maxEdgeDist = 0
            for i in range((x-extraMargin), (w+x+extraMargin)):
                for j in range((y-extraMargin), (h+y+extraMargin)):
                    distToEdge = cv2.pointPolygonTest(contour, (i, j), True)
                    if distToEdge >= 0:
                        emptyBoundRect[j-y][i-x] = distToEdge
            # maxEdgeDist = emptyBoundRect.max()
            # if len(emptyBoundRect) == (h + (2 * extraMargin)) and len(emptyBoundRect[0]) == (w + (2 * extraMargin)):
            # print("(h + (2 * extraMargin)) = ", (h + (2 * extraMargin)))
            # print("len(emptyBoundRect) = ", len(emptyBoundRect))
            # print("(w + (2 * extraMargin)) = ", (w + (2 * extraMargin)))
            # print("len(emptyBoundRect[0]) = ", len(emptyBoundRect[0]))

            maxEdgeDist = np.amax(emptyBoundRect)
            print("maxEdgeDist = ", maxEdgeDist)
            minEdgeDist = 0
            #mapEdgeDist = np.empty([3, len(emptyBoundRect), len(emptyBoundRect[0])])

            mapEdgeDistEmpty = np.full((len(emptyBoundRect), len(emptyBoundRect[0])), 1, dtype=np.uint8)
            # print("emptyBoundRect = ", len(emptyBoundRect))
            # print("emptyBoundRect[0] = ", len(emptyBoundRect[0]))
            # print("mapEdgeDistEmpty = ", len(mapEdgeDistEmpty))
            # print("mapEdgeDistEmpty[0] = ", len(mapEdgeDistEmpty[0]))
            mapEdgeDistFull = mapEdgeDistEmpty * (int(255 / maxEdgeDist) * emptyBoundRect)

            emptyR = np.full((len(emptyBoundRect), len(emptyBoundRect[0])), 0, dtype=np.uint8)
            emptyG = np.full((len(emptyBoundRect), len(emptyBoundRect[0])), 0, dtype=np.uint8)
            fullImg = np.full((len(emptyBoundRect), len(emptyBoundRect[0]), 3), 0, dtype=np.uint8)
            contourOnlyImg = np.full((len(emptyBoundRect), len(emptyBoundRect[0]), 3), 255, dtype=np.uint8)
            realImg = np.full((len(emptyBoundRect), len(emptyBoundRect[0]), 3), 0, dtype=np.uint8)
            fullImg[:, :, 0] = emptyG
            fullImg[:, :, 1] = emptyR
            fullImg[:, :, 2] = mapEdgeDistFull

            cv2.drawContours(contourOnlyImg, contour, -1, (200, 0, 0), 3, offset=(-1*(x-extraMargin), -1*(y-extraMargin)))

            # print("(y-extraMargin) = ", (y-extraMargin))
            # print("(y+h+extraMargin) = ", (y+h+extraMargin))
            # print("(x-extraMargin) = ", (x-extraMargin))
            # print("(x+w+extraMargin) = ", (x+w+extraMargin))

            realImg[:, :, :] = originalImg[(y-extraMargin):(y+h+extraMargin), (x-extraMargin):(x+w+extraMargin), :]

            # mapEdgeDistGray = (mapEdgeDistFull - minEdgeDist) * (65535.0 / (maxEdgeDist - minEdgeDist))
            # mapEdgeDistFixtype = cv2.convertScaleAbs(mapEdgeDistGray, alpha=(255.0 / 65535.0))
            # mapEdgeDistColor = cv2.applyColorMap(mapEdgeDistFixtype, cv2.COLORMAP_JET)

            # localMaxThresh = cv2.cvtColor(fullImg, cv2.COLOR_RGB2GRAY)
            # _,blackWhiteThresh = cv2.threshold(localMaxThresh, 100, 255, cv2.THRESH_BINARY)
            lower = (0, 0, 150)
            upper = (0, 0, 255)
            blackWhiteThresh = cv2.inRange(fullImg, lower, upper)
            # blackWhiteThresh32 = np.float32(blackWhiteThresh)
            # realImg8 = np.uint8(realImg)
            # ret, test = cv2.connectedComponents(blackWhiteThresh)
            # markers = cv2.watershed(realImg, test)
            # realImg[markers == -1] = [255, 0, 0]

            # sourceImageGray = cv2.cvtColor(fullImg, cv2.COLOR_RGB2GRAY)
            sourceImageGray = cv2.cvtColor(contourOnlyImg, cv2.COLOR_RGB2GRAY)
            _, sourceImageBW = cv2.threshold(sourceImageGray, 127, 255, cv2.THRESH_BINARY)
            hotSpotContour, hotSpotHeirarchy = cv2.findContours(sourceImageBW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            hotSpotContour, hotSpotHeirarchy = cv2.findContours(blackWhiteThresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            seedPos = np.zeros((len(hotSpotContour), 2))
            print("num hot spots = ", len(hotSpotContour))
            cntNum = 0
            for cnt in hotSpotContour:
                moment = cv2.moments(cnt)
                seedPos[cntNum][1] = int(moment["m10"] / moment["m00"])
                seedPos[cntNum][0] = int(moment["m01"] / moment["m00"])
                cntNum += 1
            print("seed positinos = ", seedPos)

            startTime = time.process_time()
            MultiFlood.Fill(sourceImageBW, seedPos)
            endTime = time.process_time()

            # cv2.watershed(realImg, marker)
            title = "distance to edge color " + str(contNum)
            title2 = "black white threshold " + str(contNum)
            # title3 = "watershed marker " + str(contNum)
            title4 = "feed to floodfill"

            cv2.imshow(title, fullImg)
            cv2.imshow(title2, blackWhiteThresh)
            cv2.imshow(title4, sourceImageGray)
            # cv2.imshow(title3, realImg)
            # cv2.imshow(title2, mapEdgeDistColor)

        return (x-extraMargin), (y-extraMargin), (x+w+extraMargin), (y+h+extraMargin)

    def selectImageCheat(self):
        self.taughtImg = True
        imgNum = 0
        filename = "findImagesFolder/opencv_frame_" + str(imgNum) + ".png"
        print("filename = ", filename)
        TeachObject.ObjectAssist(filename)

    def showMaskImg(self, allColor, colorInterest):
        print("allColor[colorInterest] = ", allColor[colorInterest])
        numItms = 10
        random_colors = np.empty([numItms, 3])
        for i in range(numItms):
            random_colors[i] = list(np.random.choice(range(255), size=3))
        while True:
            convertHSV = True
            upperMax = 255
            lowerMin = 0
            upper = np.array([0,0,0])
            lower = np.array([0,0,0])
            upperMargin = np.array([0,0,0])
            lowerMargin = np.array([0,0,0])
            for i in range(3):
                upperMargin[i] = (upperMax - allColor[colorInterest][i]) * (self.marginSlider.get() / 100)
                lowerMargin[i] = (allColor[colorInterest][i] - lowerMin) * (self.marginSlider.get() / 100)
                if i < 4:
                    upper[i] = allColor[colorInterest][i] + upperMargin[i]
                    lower[i] = allColor[colorInterest][i] - lowerMargin[i]
                else:
                    upper[i] = allColor[colorInterest][i] + 10
                    lower[i] = allColor[colorInterest][i] - 10
            filename = "findImagesFolder/opencv_frame_" + str(0) + ".png"
            frame = cv2.imread(filename)
            upperEdge = self.upperCanny.get()
            lowerEdge = self.lowerCanny.get()
            edgeImg = cv2.Canny(frame, lowerEdge, upperEdge)
            if convertHSV:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            intermediate = cv2.inRange(frame, lower, upper)
            cv2.imshow("mask frame", intermediate)
            contours, hierarchy = cv2.findContours(intermediate,
                                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if convertHSV:
                frame = cv2.cvtColor(frame, cv2.COLOR_HSV2RGB)

            colorCount = 0
            for c in contours:
                self.getCircularity(c)
                if (cv2.contourArea(c) > TeachObject.ObjectAssist.maxArea * 0.6) and \
                        (self.getCircularity(c) > TeachObject.ObjectAssist.Circularity * 0.8):
                    cv2.drawContours(frame, c, -1, (int(random_colors[colorCount][0]), int(random_colors[colorCount][1]),
                                                    int(random_colors[colorCount][2])), 3)
                    xStart, yStart, xEnd, yEnd = self.splitContour(c, colorCount)
                    cv2.rectangle(frame, (xStart, yStart), (xEnd, yEnd), (0, 255, 0), 2)
                    colorCount += 1
                    # cv2.drawContours(frame, c, -1, (0, 255, 0), 3)
                # cv2.drawContours(frame, c, -1, (0, 255, 0), 3)
            cv2.imshow("Contour View", frame)
            cv2.imshow("edge image", edgeImg)
            k = cv2.waitKey(1)
            if k % 256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                break
        cv2.destroyAllWindows()

    def getCircularity(self, givenContour):
        minCircle = cv2.minEnclosingCircle(givenContour)
        center = [0, 0]
        center[0] = int(minCircle[0][0])
        center[1] = int(minCircle[0][1])
        radius = int(minCircle[1])
        minCircleArea = 3.14 * (radius * radius)
        if minCircleArea > 0:
            circularity = cv2.contourArea(givenContour) / minCircleArea
        else:
            circularity = 0
        return circularity

    def createEmptyMask(self):
        if self.emptyMaskExist:
            self.empty_img_name = "findImagesFolder/empty_mask" + str(0) + ".png"
            self.emptyImage = cv2.imread(self.empty_img_name)

            allHue = np.full((len(self.emptyImage), len(self.emptyImage[0]), self.emptyCount), 0)
            allSaturation = np.full((len(self.emptyImage), len(self.emptyImage[0]), self.emptyCount), 0)
            allValue = np.full((len(self.emptyImage), len(self.emptyImage[0]), self.emptyCount), 0)
            averageHue = np.full((len(self.emptyImage), len(self.emptyImage[0])), 0)
            averageSaturation = np.full((len(self.emptyImage), len(self.emptyImage[0])), 0)
            averageValue = np.full((len(self.emptyImage), len(self.emptyImage[0])), 0)
            # print("emptyCount = ", self.emptyCount)
            for i in range(0, self.emptyCount):
                # print('i = ', i)
                self.empty_img_name = "findImagesFolder/empty_mask" + str(i) + ".png"
                self.emptyImage = cv2.imread(self.empty_img_name)
                self.emptyImageHSV = cv2.cvtColor(self.emptyImage, cv2.COLOR_RGB2HSV)
                allHue[:, :, i] = self.emptyImageHSV[:, :, 0]
                allSaturation[:, :, i] = self.emptyImageHSV[:, :, 1]
                allValue[:, :, i] = self.emptyImageHSV[:, :, 2]
            #     print("self.emptyImageHSV[100][100][0] = ", self.emptyImageHSV[100][100][0])
            # print("allHue[100][100][0] = ", allHue[100][100][0])
            # print("allHue[100][100][1] = ", allHue[100][100][1])
            # print("allHue[100][100][2] = ", allHue[100][100][2])
            averageHue = np.mean(np.array([allHue]), axis=(0, 3)).astype(np.uint8)
            # print("averageHue[100][100] = ", averageHue[100][100])
            averageSaturation = np.mean(np.array([allSaturation]), axis=(0, 3)).astype(np.uint8)
            averageValue = np.mean(np.array([allValue]), axis=(0, 3)).astype(np.uint8)

            # print("averageHue.shape = ", averageHue.shape)
            # print("len(averageHue) = ", len(averageHue))
            # print("len(averageHue[0]) = ", len(averageHue[0]))
            # print("len(averageSaturation) = ", len(averageSaturation))
            # print("len(averageSaturation[0]) = ", len(averageSaturation[0]))
            # print("len(averageValue) = ", len(averageValue))
            # print("len(averageValue[0]) = ", len(averageValue[0]))

            averageHSV = np.stack((averageHue, averageSaturation, averageValue), axis=2)
            cv2.imshow("Average Empty", averageValue)

            # # averageHSV = [[averageHue], [averageSaturation], [averageValue]]
            # print("averageHSV.shape = ", averageHSV.shape)
            # # self.emptyImageHSV = cv2.cvtColor(self.emptyImage, cv2.COLOR_RGB2HSV)
            # imgNum = self.chooseImage.current()
            # filename = "findImagesFolder/opencv_frame_" + str(imgNum) + ".png"
            # compareImg = cv2.imread(filename)
            # compareImgHSV = cv2.cvtColor(compareImg, cv2.COLOR_RGB2HSV)
            # compareImgValue = compareImgHSV[:, :, 2]
            # # imgDifference = abs(compareImgHSV - self.emptyImageHSV)
            # # imgDifference = abs(compareImg - self.emptyImage)
            # imgDifference = abs(compareImgHSV - averageHSV)
            # imgValueDifference = abs(compareImgValue - averageValue)
            # grayImage = np.full((len(self.emptyImage), len(self.emptyImage[0])), 0)
            # grayImage = imgDifference[:, :, 2]
            # grayImage = cv2.cvtColor(imgDifference, cv2.COLOR_RGB2GRAY)
            # cv2.imshow("Empty Comparison Value", imgValueDifference)
            # cv2.imshow("Comparison Value", compareImgValue)
            # upper = self.upperCanny.get()
            # lower = self.lowerCanny.get()
            # _, blackWhiteCompareImg = cv2.threshold(grayImage, lower, upper, cv2.THRESH_BINARY_INV)
            # blackWhiteCompareImg = np.uint8(blackWhiteCompareImg)
            # cv2.imshow("Empty Comparison Black White", blackWhiteCompareImg)
            emptyMaskThread = threading.Thread(target=self.applyEmptyMask, args=(averageHSV, i))
            emptyMaskThread.start()
            # self.applyEmptyMask(averageHSV)
        else:
            print("Missing required images")

    def applyEmptyMask(self, emptyHSV, blank):
        numItms = 10
        random_colors = np.empty([numItms, 3])
        for i in range(numItms):
            random_colors[i] = list(np.random.choice(range(255), size=3))
        while True:
            _, frame = self.camera.read()
            upperEdge = self.upperCanny.get()
            lowerEdge = self.lowerCanny.get()
            edgeImg = cv2.Canny(frame, lowerEdge, upperEdge)
            frameHSV = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            differenceValue = abs(frame[:, :, 2] - emptyHSV[:, :, 2])

            upper = self.upperCanny.get()
            lower = self.lowerCanny.get()
            _, blackWhiteCompareImg = cv2.threshold(differenceValue, lower, upper, cv2.THRESH_BINARY)
            blackWhiteCompareImg = np.uint8(blackWhiteCompareImg)
            cv2.imshow("Empty Comparison Black White", blackWhiteCompareImg)

            cv2.imshow("differenceValue", differenceValue)
            contours, hierarchy = cv2.findContours(blackWhiteCompareImg,
                                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            self.determineContour(contours, frame, random_colors)
            k = cv2.waitKey(1)
            if k % 256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                break
            elif k == ord('g'):
                # g pressed
                img_name = "maskImagesFolder/opencv_frame_{}.png".format(self.img_counter)
                cv2.imwrite(img_name, frame)

        self.camera.release()
        cv2.destroyAllWindows()



CreateSegmentations()