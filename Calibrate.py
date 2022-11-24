import cv2
import numpy as np
import time
import math

class Calibration:
    def __init__(self, raw_calibration_img, getCal):
        # startTime = time.process_time()
        # print("start time = ", startTime)
        print("start")
        grid_size = 10
        self.gridLength = 9
        self.gridWidth = 11
        Calibration.pixel2Mm = 0
        #width = 400
        #height = 288
        #Calibration.raw_calibration_img_resize = raw_calibration_img.resize((width, height))
        scale_percent = 100  # percent of original size
        self.width = int(raw_calibration_img.shape[1] * scale_percent / 100)
        self.length = int(raw_calibration_img.shape[0] * scale_percent / 100)
        dim = (self.width, self.length)
        Calibration.raw_calibration_img_resize = cv2.resize(raw_calibration_img, dim)
        Calibration.raw_calibration_img_resize_hsv = cv2.cvtColor(Calibration.raw_calibration_img_resize, cv2.COLOR_BGR2HSV)
        #hsv_img = cv2.cvtColor(Calibration.raw_calibration_img_resize, cv2.COLOR_BGR2HSV)
        upperVal = 60
        lower = (0, 0, 0)
        upper = (upperVal, upperVal, upperVal)
        lower_hsv = (0, 0, 0)
        upper_hsv = (25, 25, 100)
        #black_white_cal_img = cv2.inRange(hsv_img, lower, upper)
        black_white_cal_img = cv2.inRange(Calibration.raw_calibration_img_resize, lower, upper)
        black_white_cal_img_hsv = cv2.inRange(Calibration.raw_calibration_img_resize_hsv, lower_hsv, upper_hsv)
        length = len(black_white_cal_img)
        width = len(black_white_cal_img[0])
        # print('length = ', self.length)
        # print('width = ', self.width)
        # Calibration.calibration_success = False
        # cv2.imshow("black and white", black_white_cal_img)
        # cv2.imshow("black and white hsv", black_white_cal_img_hsv)
        edge_calibration_img = cv2.Canny(black_white_cal_img, 100, 200)
        # cv2.imshow("edge image", edge_calibration_img)

        # self.findLines(Calibration.raw_calibration_img_resize)
        # self.findLines(edge_calibration_img)

        # findCornerStart = time.clock()
        # corner_loc1, slope1, intersect1 = self.find_corner(0, 0, edge_calibration_img, 1)
        # findCornerEnd = time.clock()
        # time2findCorner = findCornerEnd - findCornerStart
        # print("time to find corners = ", time2findCorner)
        #
        # #corner_loc1_test, slope1_test, intersect1_test = self.corner_test(0, 0, edge_calibration_img, 1)
        # print("corner = ", corner_loc1)
        # end1 = [0,0]
        # end1[0] = corner_loc1[0] + 550
        # end1[1] = int((slope1 * end1[0]) + intersect1)
        # corner_loc2, slope2, intersect2 = self.find_corner(length, width, edge_calibration_img, 1)
        # corner_loc3, slope3, intersect3 = self.find_corner(length, 0, edge_calibration_img, 2)
        # corner_loc4, slope4, intersect4 = self.find_corner(0, width, edge_calibration_img, 2)

        findCornerStart = time.process_time()
        # cornerCheck, _, _ = self.find_all_corners(black_white_cal_img)
        cornerContours, numGridPoints, cornerCheck = self.find_all_corners_contours(black_white_cal_img, Calibration.raw_calibration_img_resize)
        [topLeftContour, bottomRightContour, topRightContour, bottomLeftContour] = cornerContours
        # [topLeft, bottomRight, topRight, bottomLeft] = contourCornerCentroids
        [topLeft, bottomRight, topRight, bottomLeft] = cornerCheck
        # print("topLeft = ", topLeft)

        # print(topLeftContour)


        # cv2.imshow("contour image", Calibration.raw_calibration_img_resize)

        # print("corners")
        # print(cornerCheck)
        # findCornerEnd = time.process_time()
        # time2findCorner = findCornerEnd - findCornerStart
        # print("time to find corners = ", time2findCorner)
        #
        # end2 = [0, 0]
        # end2[0] = corner_loc2[0] + 98
        # end2[1] = int((slope2 * end2[0]) + intersect2)
        # data1 = (corner_loc1, slope1, intersect1)
        # data2 = (corner_loc2, slope2, intersect2)
        #corner_loc3a = self.find_intersect_corners(data1, data2)
        #cv2.imshow("Canny", edge_calibration_img)


        #elf.funky_stuff(black_white_cal_img, Calibration.raw_calibration_img_resize)

        all_corners = [[0,0,0,0],[0,0,0,0]]
        # all_corners[0][0] = corner_loc1[0]
        # all_corners[1][0] = corner_loc1[1]
        # all_corners[0][1] = corner_loc2[0]
        # all_corners[1][1] = corner_loc2[1]
        # all_corners[0][2] = corner_loc3[0]
        # all_corners[1][2] = corner_loc3[1]
        # all_corners[0][3] = corner_loc4[0]
        # all_corners[1][3] = corner_loc4[1]

        all_corners[0][0] = cornerCheck[0][0]
        all_corners[1][0] = cornerCheck[0][1]
        all_corners[0][1] = cornerCheck[1][0]
        all_corners[1][1] = cornerCheck[1][1]
        all_corners[0][2] = cornerCheck[2][0]
        all_corners[1][2] = cornerCheck[2][1]
        all_corners[0][3] = cornerCheck[3][0]
        all_corners[1][3] = cornerCheck[3][1]

        # if sum(map(sum, all_corners)) != 0:
        if numGridPoints >= 4:
            # print("hello?")
            # TL, TR, BL, BR = self.define_corners(all_corners)
            # #rect = (TL, TR, BL, BR)
            #
            # print("top right and bottom right")
            # TR_Confirmed, BR_Confirmed = self.confirm_corner(TR, BR, black_white_cal_img)
            # print("top left and bottom left")
            # TL_Confirmed, BL_Confirmed = self.confirm_corner(TL, BL, black_white_cal_img)
            # print("top left and top right")
            # TL_Confirmed, TR_Confirmed = self.confirm_corner(TL_Confirmed, TR_Confirmed, black_white_cal_img)
            # print("bottom left and bottom right")
            # BL_Confirmed, BR_Confirmed = self.confirm_corner(BL_Confirmed, BR_Confirmed, black_white_cal_img)
            #
            # rect = (TL_Confirmed, TR_Confirmed, BL_Confirmed, BR_Confirmed)
            #
            # stretched_img = self.four_point_transformation(rect, Calibration.raw_calibration_img_resize)
            # top_crossings = self.count_crossings(TL_Confirmed, TR_Confirmed, [-10, 5], black_white_cal_img, Calibration.raw_calibration_img_resize)
            # bottom_crossings = self.count_crossings(BL_Confirmed, BR_Confirmed, [-10, -5], black_white_cal_img, Calibration.raw_calibration_img_resize)
            # left_crossings = self.count_crossings(TL_Confirmed, BL_Confirmed, [5, -10], black_white_cal_img, Calibration.raw_calibration_img_resize)
            # right_crossings = self.count_crossings(TR_Confirmed, BR_Confirmed, [-5, -10], black_white_cal_img, Calibration.raw_calibration_img_resize)
            # cv2.imshow("stretched image", stretched_img)

            # print("height = ", len(stretched_img))
            # print("length = ", len(stretched_img[0]))
            #ratio (pixel/mm)
            cv2.drawContours(Calibration.raw_calibration_img_resize, topLeftContour, -1, (255, 255, 0), 3)
            cv2.drawContours(Calibration.raw_calibration_img_resize, bottomRightContour, -1, (0, 255, 255), 3)
            cv2.drawContours(Calibration.raw_calibration_img_resize, topRightContour, -1, (255, 0, 0), 3)
            cv2.drawContours(Calibration.raw_calibration_img_resize, bottomLeftContour, -1, (0, 0, 255), 3)

            rect = (topLeft, topRight, bottomLeft, bottomRight)
            stretched_img = self.four_point_transformation(rect, Calibration.raw_calibration_img_resize)
            # cv2.imshow("stretched image", stretched_img)

            top_crossings = self.count_crossings(topLeft, topRight, [-10, 5], black_white_cal_img,
                                                 Calibration.raw_calibration_img_resize)
            bottom_crossings = self.count_crossings(bottomLeft, bottomRight, [-10, -5], black_white_cal_img,
                                                    Calibration.raw_calibration_img_resize)
            left_crossings = self.count_crossings(topLeft, bottomLeft, [5, -10], black_white_cal_img,
                                                  Calibration.raw_calibration_img_resize)
            right_crossings = self.count_crossings(topRight, bottomRight, [-5, -10], black_white_cal_img,
                                                   Calibration.raw_calibration_img_resize)



            # cv2.circle(Calibration.raw_calibration_img_resize, (corner_loc1[0], corner_loc1[1]), 3, (37, 167, 150), -1)
            # # cv2.circle(Calibration.raw_calibration_img_resize, (corner_loc1_test[0], corner_loc1_test[1]), 3, (0, 255, 255), -1)
            # # cv2.line(Calibration.raw_calibration_img_resize, (corner_loc1[0], corner_loc1[1]), (end1[0], end1[1]), (37, 167, 150), 1)
            # cv2.circle(Calibration.raw_calibration_img_resize, (corner_loc2[0], corner_loc2[1]), 3, (37, 167, 150), -1)
            # # cv2.line(Calibration.raw_calibration_img_resize, (corner_loc2[0], corner_loc2[1]), (end2[0], end2[1]), (37, 167, 150), 1)
            # cv2.circle(Calibration.raw_calibration_img_resize, (corner_loc3[0], corner_loc3[1]), 3, (37, 167, 150), -1)
            # cv2.circle(Calibration.raw_calibration_img_resize, (corner_loc4[0], corner_loc4[1]), 3, (37, 167, 150), -1)

            # cv2.line(Calibration.raw_calibration_img_resize, (corner_loc1[0], corner_loc1[1]), (corner_loc3[0], corner_loc3[1]),
            #          (37, 167, 150), 1)
            # cv2.line(Calibration.raw_calibration_img_resize, (corner_loc3[0], corner_loc3[1]), (corner_loc2[0], corner_loc2[1]),
            #          (37, 167, 150), 1)
            # cv2.line(Calibration.raw_calibration_img_resize, (corner_loc2[0], corner_loc2[1]), (corner_loc4[0], corner_loc4[1]),
            #          (37, 167, 150), 1)
            # cv2.line(Calibration.raw_calibration_img_resize, (corner_loc1[0], corner_loc1[1]), (corner_loc4[0], corner_loc4[1]),
            #          (37, 167, 150), 1)
            # cv2.circle(Calibration.raw_calibration_img_resize, (maybe_points[0], maybe_points[1]), 3, (200, 167, 150), -1)
            # cv2.circle(Calibration.raw_calibration_img_resize, (TR_Confirmed[0], TR_Confirmed[1]), 5, (0, 167, 0), -1)
            # cv2.circle(Calibration.raw_calibration_img_resize, (BR_Confirmed[0], BR_Confirmed[1]), 5, (0, 167, 0), -1)
            # cv2.circle(Calibration.raw_calibration_img_resize, (TL_Confirmed[0], TL_Confirmed[1]), 5, (0, 167, 0), -1)
            # cv2.circle(Calibration.raw_calibration_img_resize, (BL_Confirmed[0], BL_Confirmed[1]), 5, (0, 167, 0), -1)


            print("top crossings = ", top_crossings)
            print("bottom crossings = ", bottom_crossings)
            print("right crossings = ", right_crossings)
            print("left crossings = ", left_crossings)
            endTime = time.process_time()
            print("end time = ", endTime)

            if (top_crossings == bottom_crossings) and (left_crossings == right_crossings):
                vertical_ratio = len(stretched_img) / (((left_crossings + right_crossings) / 2) * grid_size)
                horizontal_ratio = len(stretched_img) / (((top_crossings + bottom_crossings) / 2) * grid_size)
                ratio = (vertical_ratio + horizontal_ratio) / 2
                Calibration.calibration_success = True

                cv2.circle(Calibration.raw_calibration_img_resize, (int(cornerCheck[0][0]), int(cornerCheck[0][1])), 5, (0, 255, 0), -1)
                cv2.circle(Calibration.raw_calibration_img_resize, (int(cornerCheck[1][0]), int(cornerCheck[1][1])), 5, (0, 255, 0), -1)
                cv2.circle(Calibration.raw_calibration_img_resize, (int(cornerCheck[2][0]), int(cornerCheck[2][1])), 5, (0, 255, 0), -1)
                cv2.circle(Calibration.raw_calibration_img_resize, (int(cornerCheck[3][0]), int(cornerCheck[3][1])), 5, (0, 255, 0), -1)
                cv2.drawContours(Calibration.raw_calibration_img_resize, topLeftContour, -1, (0, 255, 0), 3)
                cv2.drawContours(Calibration.raw_calibration_img_resize, bottomRightContour, -1, (0, 255, 0), 3)
                cv2.drawContours(Calibration.raw_calibration_img_resize, topRightContour, -1, (0, 255, 0), 3)
                cv2.drawContours(Calibration.raw_calibration_img_resize, bottomLeftContour, -1, (0, 255, 0), 3)

                print(((left_crossings + 1) * grid_size))
                vertical_ratio = len(stretched_img) / ((left_crossings) * grid_size)
                horizontal_ratio = len(stretched_img[0]) / ((top_crossings) * grid_size)
                Calibration.pixel2Mm = (vertical_ratio + horizontal_ratio) / 2
                print("vertical_ratio = ", vertical_ratio)
                print("horizontal_ratio = ", horizontal_ratio)
                print("calibration pixel to mmm = ", Calibration.pixel2Mm)
                print("len(stretched_img) = ", len(stretched_img))
                print("len(stretched_img[0]) = ", len(stretched_img[0]))
                gridLength = len(stretched_img) / Calibration.pixel2Mm
                gridWidth = len(stretched_img[0]) / Calibration.pixel2Mm
                print("grid size = ", gridLength, " x ", gridWidth)
            else:
                Calibration.calibration_success = False
                Calibration.pixel2Mm = 0
                cv2.circle(Calibration.raw_calibration_img_resize, (int(cornerCheck[0][0]), int(cornerCheck[0][1])), 5, (0, 0, 255), -1)
                cv2.circle(Calibration.raw_calibration_img_resize, (int(cornerCheck[1][0]), int(cornerCheck[1][1])), 5, (0, 0, 255), -1)
                cv2.circle(Calibration.raw_calibration_img_resize, (int(cornerCheck[2][0]), int(cornerCheck[2][1])), 5, (0, 0, 255), -1)
                cv2.circle(Calibration.raw_calibration_img_resize, (int(cornerCheck[3][0]), int(cornerCheck[3][1])), 5, (0, 0, 255), -1)
                cv2.drawContours(Calibration.raw_calibration_img_resize, topLeftContour, -1, (0, 0, 255), 3)
                cv2.drawContours(Calibration.raw_calibration_img_resize, bottomRightContour, -1, (0, 0, 255), 3)
                cv2.drawContours(Calibration.raw_calibration_img_resize, topRightContour, -1, (0, 0, 255), 3)
                cv2.drawContours(Calibration.raw_calibration_img_resize, bottomLeftContour, -1, (0, 0, 255), 3)
            x = 300
            y = 180
            i = 0
            j = 0
            #print("corner = ", edge_calibration_img[167][294])
            #for i in range(25):
            #    for j in range(25):
            #        if edge_calibration_img[y][x] == 0:
            #            print(edge_calibration_img[y][x], "(", x, ",", y, ")", end="  ")
            #        if edge_calibration_img[y][x] == 255:
            #            print("1 (", x, ",", y, ")", end="  ")
            #        x += 1
            #    print("")
            #    y += 1
            #    x = 290
        else:
            Calibration.calibration_success = False
        # cv2.imshow("Color Image", Calibration.raw_calibration_img_resize)
        # cv2.waitKey(0)

    def findLines(self, image):
        lines_list = [[0,0], [0,0]]
        # Use canny edge detection
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        edges = image

        # Apply HoughLinesP method to
        # to directly obtain line end points
        lines = cv2.HoughLinesP(
            edges,  # Input edge image
            1,  # Distance resolution in pixels
            np.pi / 180,  # Angle resolution in radians
            threshold=100,  # Min number of votes for valid line
            minLineLength=5,  # Min allowed length of line
            maxLineGap=10  # Max allowed gap between line for joining them
        )

        # Iterate over points
        for points in lines:
            # Extracted points nested in the list
            x1, y1, x2, y2 = points[0]
            # Draw the lines joing the points
            # On the original image
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Maintain a simples lookup list for points
            lines_list.append([(x1, y1), (x2, y2)])
        # cv2.imshow("lines", image)

    def find_corner(self, start_len, start_width, img, direction):
        if start_len == 0:
            len_increment = 1
            end_len = self.length
            #len_check = 0
            len_check = self.length*0.75
            start_len1 = 0
        elif start_len == self.length:
            len_increment = -1
            end_len = 0
            start_len = start_len - 1
            start_len1 = start_len - 1
            #len_check = self.length
            #len_check = 0
            len_check = self.length * 0.75
        if start_width == 0:
            width_increment = 1
            end_width = self.width
            width_check = 0
            start_width1 = 0
        elif start_width == self.width:
            width_increment = -1
            end_width = 0
            #width_check = self.width
            width_check = 0
            start_width = start_width - 1
            start_width1 = start_width - 1

        return_val = [0,0]
        break_val = 0
        print("start_len = ", start_len)
        print("start_width = ", start_width)
        print("end_len = ", end_len)
        print("end_width = ", end_width)
        print("len_check = ", len_check)
        print("width_check = ", width_check)
        #while abs(start_len) < abs(end_len):
        #    while abs(start_width) < abs(end_width):
        print("abs(end_len-start_len) = ", abs(end_len-start_len))

        if direction == 1:
            while abs(end_len-start_len) > len_check:
                #print("abs(end_len-(start_len*4)) = ", abs(end_len - (start_len*4)))
                while abs(end_width - start_width) > width_check:
                    if img[start_len][start_width] > 200:
                        return_val[0] = start_width
                        return_val[1] = start_len
                        break_val = 1
                        break
                    else:
                        start_width += width_increment
                if break_val == 1:
                    break
                else:
                    start_width = start_width1
                    start_len += len_increment
        elif direction == 2:
            while abs(end_width - start_width) > width_check:
                while abs(end_len-start_len) > len_check:
                    if img[start_len][start_width] > 200:
                        return_val[0] = start_width
                        return_val[1] = start_len
                        break_val = 1
                        break
                    else:
                        start_len += len_increment
                if break_val == 1:
                    break
                else:
                    start_len = start_len1
                    start_width += width_increment
        print("exit")
        #m, b = self.find_edge_boundary(return_val, 1, img)
        m=0
        b=0
        return return_val, m, b

    def find_all_corners_contours(self, img, colorImg):
        break_val = 0
        length = len(img)
        width = len(img[0])
        cornerFound = [False, False, False, False]
        lenCheck = 0
        widthCheck = 0
        return_val = [[0, 0], [length, 0], [length, width], [0, width]]
        # cv2.imshow("image", img)
        print(np.all(cornerFound))

        kernel = np.ones((2, 2), np.uint8)
        img_erode = cv2.erode(img, kernel, iterations=1)
        # cv2.imshow("dilated Image", img_erode)

        contours, hierarchy = cv2.findContours(img_erode, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        squares = 0
        contourMax = max(contours, key=cv2.contourArea)
        print(cv2.contourArea(contourMax))
        # cv2.erode(contourMax, element)
        # cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
        # cv2.drawContours(img, contourMax, -1, (255, 0, 0), 3)

        centroid = np.zeros((7, len(contours)))
        i = 0
        j = 0
        lowerArea = 800
        upperArea = 3000
        hotContour = 0
        adjustedContour = 0
        numCntSplit = 0
        for c in contours:
            k = 0
            M = cv2.moments(c)
            area = cv2.contourArea(c)
            minCircle = cv2.minEnclosingCircle(c)
            radius = int(minCircle[1])
            minCircleArea = 3.14 * (radius * radius)
            circularity = 0
            if minCircleArea > 0:
                circularity = area / minCircleArea

            # areaLen = 0
            # useSplitContour = False
            # # print("area = ", area)
            # area = [cv2.contourArea(c), -1]
            # # print("circularity = ", circularity)
            # if area[k] > (lowerArea * 4):
            #     area, hotContour, adjustedContour = self.splitContour(c, j, colorImg)
            #     print("split area[k] = ", area)
            #     print(area.any(-1))
            #     # if area.any() == -1:
            #     if -1 in area:
            #         areaLen = -1
            #         useSplitContour = False
            #     else:
            #         areaLen = len(area)-1
            #         useSplitContour = True
            #         numCntSplit += 1
            #         print("hello")
            # # print("len(area) = ", areaLen)
            # while k <= areaLen:
            #     if useSplitContour is False:
            #         currentContour = c
            #         color = (0, 0, 255)
            #     elif useSplitContour is True:
            #         currentContour = hotContour[k]
            #         # currentContour = adjustedContour[k]
            #         color = (255, 0, 0)
            #         # hotM = cv2.moments(currentContour)
            #         # x = int(hotM['m10'] / hotM['m00'])
            #         # y = int(hotM['m01'] / hotM['m00'])
            #         # cv2.circle(colorImg, (int(x), int(y)), 1, color, 3)
            #     print("area[k] = ", area[k])
            #     # print("lowerArea / 3 = ", (lowerArea/3))
            #     M = cv2.moments(currentContour)
            #     minCircle = cv2.minEnclosingCircle(currentContour)
            #     radius = int(minCircle[1])
            #     minCircleArea = 3.14 * (radius * radius)
            #     circularity = 0
            #     if minCircleArea > 0:
            #         circularity = area[k] / minCircleArea
            #     if M['m00'] != 0 and (area[k] > (lowerArea / 2) and area[k] < upperArea) and circularity > 0.6:
            #         print("valid area")
            #         print("area = ", area[k])
            #         print("k = ", k)
            #         centroid[0][i] = int(M['m10'] / M['m00'])
            #         centroid[1][i] = int(M['m01'] / M['m00'])
            #         centroid[2][i] = centroid[0][i] + centroid[1][i]
            #         centroid[3][i] = math.sqrt(math.pow((centroid[0][i] - width), 2) + math.pow((centroid[1][i] - 0), 2))
            #         centroid[4][i] = math.sqrt(math.pow((centroid[0][i] - 0), 2) + math.pow((centroid[1][i] - length), 2))
            #         centroid[5][i] = area[k]
            #         centroid[6][i] = j
            #         text = str(j)
            #         cv2.circle(colorImg, (int(centroid[0][i]), int(centroid[1][i])), 1, color, 3)
            #         # cv2.putText(colorImg, text, 1, 1, "green", 2, 2, (int(centroid[0][i]), int(centroid[1][i])))
            #         i += 1
            #     k += 1

            if M['m00'] != 0 and (area > lowerArea and area < upperArea) and circularity > 0.5:
                # print("valid area")
                centroid[0][i] = int(M['m10'] / M['m00'])
                centroid[1][i] = int(M['m01'] / M['m00'])
                centroid[2][i] = centroid[0][i] + centroid[1][i]
                centroid[3][i] = math.sqrt(math.pow((centroid[0][i] - width), 2) + math.pow((centroid[1][i] - 0), 2))
                centroid[4][i] = math.sqrt(math.pow((centroid[0][i] - 0), 2) + math.pow((centroid[1][i] - length), 2))
                centroid[5][i] = area
                centroid[6][i] = j
                text = str(j)
                cv2.circle(colorImg, (int(centroid[0][i]), int(centroid[1][i])), 1, (0, 0, 255), 3)
                # cv2.putText(colorImg, text, 1, 1, "green", 2, 2, (int(centroid[0][i]), int(centroid[1][i])))
                i += 1
            # elif area > (lowerArea * 2):
            #     self.splitContour(c, j, colorImg)
            j += 1
        print("i = ", i)
        # cv2.imshow("contour image", img)
        arrayLength = i
        tempArray = np.zeros((7, arrayLength))

        # tempArray = centroid[:][:(i-1)]
        print("number of dots = ", arrayLength)
        if arrayLength >= 4:
            for i in range(7):
                tempArray[i][:] = centroid[i][:arrayLength]

            # topLeft = min(centroid[2][:])
            # bottomRight = max(centroid[2][:])
            # topRight = min(centroid[3][:])
            # bottomLeft = min(centroid[4][:])

            topLeft = min(tempArray[2][:])
            bottomRight = max(tempArray[2][:])
            topRight = min(tempArray[3][:])
            bottomLeft = min(tempArray[4][:])
            areaMin = min(tempArray[5][:])
            print("areaMin = ", areaMin)

            # print("tempArray[2][:] = ", tempArray[3])

            topLeftIndex = np.where(centroid[2] == topLeft)
            bottomRightIndex = np.where(centroid[2] == bottomRight)
            topRightIndex = np.where(centroid[3] == topRight)
            bottomLeftIndex = np.where(centroid[4] == bottomLeft)
            areaMinIndex = np.where(centroid[5] == areaMin)

            topLeftCords = [centroid[0][topLeftIndex[0][0]], centroid[1][topLeftIndex[0][0]]]
            topRightCords = [centroid[0][topRightIndex[0][0]], centroid[1][topRightIndex[0][0]]]
            bottomLeftCords = [centroid[0][bottomLeftIndex[0][0]], centroid[1][bottomLeftIndex[0][0]]]
            bottomRightCords = [centroid[0][bottomRightIndex[0][0]], centroid[1][bottomRightIndex[0][0]]]
            cornerPoints = [topLeftCords, bottomRightCords, topRightCords, bottomLeftCords]

            # print("length = ", length)
            # print("width = ", width)
            # print("top right loc = ", tempArray[0][topRightIndex[0][0]], " + ", tempArray[1][topRightIndex[0][0]])
            # print("actual top right point = ", tempArray[0][arrayLength-1], " + ", tempArray[1][arrayLength-1])
            #
            #
            # print("topLeft = ", topLeft)
            # print("topRight = ", topRight)
            # print("bottomLeft = ", bottomLeft)
            # print("bottomRight = ", bottomRight)

            # print("topRightIndex = ", topRightIndex)
            # print("bottomLeftIndex = ", bottomLeftIndex)

            # cv2.circle(colorImg, (int(centroid[0][topRightIndex[0][0]]), int(centroid[1][topRightIndex[0][0]])), 1, (0, 0, 255), 3)
            # cv2.circle(colorImg, (int(centroid[0][areaMinIndex[0][0]]), int(centroid[1][areaMinIndex[0][0]])), 1, (0, 255, 255), 10)


            # cv2.drawContours(img, c, -1, (0, 255, 0), 3)
            # cv2.imshow("contour", img)
            contourIndex = [int(centroid[6][topLeftIndex[0][0]]), int(centroid[6][bottomRightIndex[0][0]]),
                            int(centroid[6][topRightIndex[0][0]]), int(centroid[6][bottomLeftIndex[0][0]])]

            # return contours[centroid[6][topLeftIndex[0][0]]], contours[centroid[6][bottomRightIndex[0][0]]], \
            #        contours[centroid[6][topRightIndex[0][0]]], contours[centroid[bottomLeftIndex[6][0][0]]]

            # return contours[topLeftIndex[0][0]], contours[bottomRightIndex[0][0]], contours[topRightIndex[0][0]], contours[bottomLeftIndex[0][0]]
            cornerContours = [contours[contourIndex[0]], contours[contourIndex[1]], contours[contourIndex[2]], contours[contourIndex[3]]]
            return cornerContours, arrayLength, cornerPoints
        else:
            empty1 = np.zeros((4, 2))
            empty2 = 0
            empty3 = np.zeros((4, 2))
            return empty1, empty2, empty3

    def splitContour(self, contour, contNum, originalImg):
        x, y, w, h = cv2.boundingRect(contour)
        extraMargin = 20
        seedAreas = np.full(2, -1)
        hotSpotContour = np.full(2, -1)
        returnContour = np.full(2, -1)
        if (x - extraMargin) < 0 or (y - extraMargin) < 0 or (x + w + (2 * extraMargin)) >= len(originalImg[0]) or (
                y + h + extraMargin) >= len(originalImg):
            print("touching edge")
        else:
            # emptyBoundRect = np.empty([(x+w), (y+h)])
            emptyBoundRect = np.zeros([h + (2 * extraMargin), w + (2 * extraMargin)])
            maxEdgeDist = 0
            # print("x = ", x)
            # print("y = ", y)
            # print("w = ", w)
            # print("h = ", h)
            maxEdgeDist = 0
            for i in range((x - extraMargin), (w + x + extraMargin)):
                for j in range((y - extraMargin), (h + y + extraMargin)):
                    distToEdge = cv2.pointPolygonTest(contour, (i, j), True)
                    if distToEdge >= 0:
                        emptyBoundRect[j - y][i - x] = distToEdge
            # maxEdgeDist = emptyBoundRect.max()
            # if len(emptyBoundRect) == (h + (2 * extraMargin)) and len(emptyBoundRect[0]) == (w + (2 * extraMargin)):
            # print("(h + (2 * extraMargin)) = ", (h + (2 * extraMargin)))
            # print("len(emptyBoundRect) = ", len(emptyBoundRect))
            # print("(w + (2 * extraMargin)) = ", (w + (2 * extraMargin)))
            # print("len(emptyBoundRect[0]) = ", len(emptyBoundRect[0]))

            maxEdgeDist = np.amax(emptyBoundRect)
            print("maxEdgeDist = ", maxEdgeDist)
            minEdgeDist = 0
            # mapEdgeDist = np.empty([3, len(emptyBoundRect), len(emptyBoundRect[0])])

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

            cv2.drawContours(contourOnlyImg, contour, -1, (200, 0, 0), 3,
                             offset=(-1 * (x - extraMargin), -1 * (y - extraMargin)))

            # print("(y-extraMargin) = ", (y-extraMargin))
            # print("(y+h+extraMargin) = ", (y+h+extraMargin))
            # print("(x-extraMargin) = ", (x-extraMargin))
            # print("(x+w+extraMargin) = ", (x+w+extraMargin))

            realImg[:, :, :] = originalImg[(y - extraMargin):(y + h + extraMargin),(x - extraMargin):(x + w + extraMargin), :]


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
            # hotSpotContour, hotSpotHeirarchy = cv2.findContours(sourceImageBW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            hotSpotContour, hotSpotHeirarchy = cv2.findContours(blackWhiteThresh, cv2.RETR_EXTERNAL,
                                                                cv2.CHAIN_APPROX_NONE)
            seedPos = np.zeros((len(hotSpotContour), 2))
            seedAreas = np.zeros((len(hotSpotContour)))
            print("num hot spots = ", len(hotSpotContour))
            cntNum = 0
            returnContour = np.zeros((len(hotSpotContour), 3), dtype=np.int)
            for cnt in hotSpotContour:
                moment = cv2.moments(cnt)
                seedPos[cntNum][1] = int(moment["m10"] / moment["m00"])
                seedPos[cntNum][0] = int(moment["m01"] / moment["m00"])
                seedAreas[cntNum] = cv2.contourArea(cnt)
                print("cnt = ", cnt[:, 0, 0] + x)
                # returnContour[cntNum][0][:] = cnt[:, 0, 0] + int(x)
                # returnContour[cntNum][1][:] = cnt[:, 0, 1] + y
                cntNum += 1
            print("seed positinos = ", seedPos)
            # MultiFlood.Fill(sourceImageBW, seedPos)

            # cv2.watershed(realImg, marker)
            title = "distance to edge color " + str(contNum)
            title2 = "black white threshold " + str(contNum)
            # title3 = "watershed marker " + str(contNum)
            title4 = "feed to floodfill"

            # cv2.imshow(title, fullImg)
            # cv2.imshow(title2, blackWhiteThresh)
            # cv2.imshow(title4, sourceImageGray)
            # cv2.imshow(title3, realImg)
            # cv2.imshow(title2, mapEdgeDistColor)

        # return (x - extraMargin), (y - extraMargin), (x + w + extraMargin), (y + h + extraMargin)
        return seedAreas, hotSpotContour, returnContour

    def find_all_corners(self, img):
        break_val = 0
        length = len(img)
        width = len(img[0])
        cornerFound = [False, False, False, False]
        lenCheck = 0
        widthCheck = 0
        return_val = [[0, 0], [length, 0], [length, width], [0, width]]
        print("length = ", length)
        print("width = ", width)
        # cv2.imshow("image", img)
        print(np.all(cornerFound))
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        squares = 0
        contourMax = max(contours, key=cv2.contourArea)
        print(cv2.contourArea(contourMax))
        # cv2.erode(contourMax, element)
        cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
        # cv2.drawContours(img, contourMax, -1, (255, 0, 0), 3)
        # cv2.imshow("contour image", img)
        print("Number of Contours found = " + str(len(contours)))
        print("Number of squares found = ", squares)
        while lenCheck < (length / 4):
            while widthCheck < (width / 4):
                breakVar = False
                # x = [lenCheck, (length - lenCheck - 1), lenCheck, (length - lenCheck - 1)]
                # y = [widthCheck, widthCheck, (width - widthCheck - 1), (width - widthCheck - 1)]
                x = [lenCheck, (length - widthCheck - 1), lenCheck, (length - widthCheck - 1)]
                y = [widthCheck, lenCheck, (width - widthCheck - 1), (width - lenCheck - 1)]
                for i in range(4):
                    # print("x = ", x[i])
                    # print("y = ", y[i])
                    # print("img[x[i]][y[i]] = ", img[x[i]][y[i]])
                    if cornerFound[i] is False and img[x[i]][y[i]] > 200:
                        # j = 0
                        # lowerLimit = 200
                        # upperLimit = 5000
                        # while j < len(contours):
                            # cnt = contours[j]
                            # dist = cv2.pointPolygonTest(cnt, (x[i], y[i]), True)
                            # area = cv2.contourArea(cnt)
                            # if dist == 0 and (area > lowerLimit and area < upperLimit):
                            #     print("Found the contour")
                            #     print("area = ", area)
                            #     return_val[i][0] = y[i]
                            #     return_val[i][1] = x[i]
                            #     cornerFound[i] = True
                            #     breakVar = True
                            #     break
                            # j += 1
                        return_val[i][0] = y[i]
                        return_val[i][1] = x[i]
                        cornerFound[i] = True
                        break
                widthCheck += 1
                # if breakVar is True:
                #     break
            if np.all(cornerFound):
                print("break!")
                break
            else:
                widthCheck = 0
                lenCheck += 1
                # print("lenCheck = ", lenCheck)
        # if direction == 1:
        #     while abs(end_len-start_len) > len_check:
        #         #print("abs(end_len-(start_len*4)) = ", abs(end_len - (start_len*4)))
        #         while abs(end_width - start_width) > width_check:
        #             if img[start_len][start_width] > 200:
        #                 return_val[0] = start_width
        #                 return_val[1] = start_len
        #                 break_val = 1
        #                 break
        #             else:
        #                 start_width += width_increment
        #         if break_val == 1:
        #             break
        #         else:
        #             start_width = start_width1
        #             start_len += len_increment
        # elif direction == 2:
        #     while abs(end_width - start_width) > width_check:
        #         while abs(end_len-start_len) > len_check:
        #             if img[start_len][start_width] > 200:
        #                 return_val[0] = start_width
        #                 return_val[1] = start_len
        #                 break_val = 1
        #                 break
        #             else:
        #                 start_len += len_increment
        #         if break_val == 1:
        #             break
        #         else:
        #             start_len = start_len1
        #             start_width += width_increment
        # print("exit")
        # #m, b = self.find_edge_boundary(return_val, 1, img)
        m=0
        b=0
        return return_val, m, b

    def corner_test(self, start_len, start_width, img, direction):
        if start_len == 0:
            len_increment = 1
            end_len = self.length
            #len_check = 0
            len_check = self.length*0.75
            start_len1 = 0
        elif start_len == self.length:
            len_increment = -1
            end_len = 0
            start_len = start_len - 1
            start_len1 = start_len - 1
            #len_check = self.length
            #len_check = 0
            len_check = self.length * 0.75
        if start_width == 0:
            width_increment = 1
            end_width = self.width
            width_check = 0
            start_width1 = 0
        elif start_width == self.width:
            width_increment = -1
            end_width = 0
            #width_check = self.width
            width_check = 0
            start_width = start_width - 1
            start_width1 = start_width - 1

        return_val = [0, 0]
        break_val = 0
        up_down_loc = 0
        right_left_loc = 0
        up_down_hit = 0
        right_left_hit = 0
        if direction == 1:
            x_counter = 0
            y_counter = 0
            break_val = 0
            while break_val == 0:
                x = 0
                y = 0
                while x <= x_counter:
                    if img[x][y_counter] > 200:
                        return_val[0] = x
                        return_val[1] = y_counter
                        break_val = 1
                        # right_left_hit[i] = []
                        break
                    x += 1
                x_counter += width_increment
                while y <= y_counter:
                    if img[x_counter][y] > 200:
                        return_val[0] = x_counter
                        return_val[1] = y
                        break_val = 1
                        # right_left_hit[i] = []
                        break
                    y += 1
                y_counter += len_increment
        m = 0
        b = 0
        return return_val, m, b

    def funky_stuff(self, raw_image, color_image):
        contours, hierarchy = cv2.findContours(raw_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(raw_image, contours, -1, (0, 255, 0), 3)
        print("length of contours = ", len(contours[0]))
        count = 0
        cv2.drawContours(color_image, contours, -1, (0, 255, 0), 3)
        hull_list = []
        for i in range(len(contours)):
            hull = cv2.convexHull(contours[i])
            hull_list.append(hull)
        for i in contours:
            #cv2.drawContours(color_image, hull_list, -1, (255, 255, 0), 3)
            print("count = ", count)
            count += 20
            x, y, w, h = cv2.boundingRect(i)
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (count, 255, 250), 2)
        #cv2.imshow("contour image", color_image)

    def find_edge_boundary(self, corner_pos, clock, img1):
        current_point = [0]*2
        current_point[0] = corner_pos[0]
        current_point[1] = corner_pos[1]
        prev_point = [0,0]
        check_point = [0,0]
        times_loop = 0
        vert_adder = [1,1,0,-1,-1,-1,0,1]
        hor_adder = [0,1,1,1,0,-1,-1,-1]
        average_slope = 0
        current_slope = 0
        num_slope_calcs = 0
        average_i = 0
        blank_img = np.zeros((len(img1), len(img1[0]), 3), dtype = "uint8")
        max_x = 0
        max_y = 0
        test_num_loops = 10
        running_average_values = [0]*5
        running_average_slope = 0
        always_looping = 0
        last_five_slope = 0
        slopes_queue = []
        temp_queue = []
        last_five_points = [[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0]]
        exit = 0
        m1 = 0
        b1 = 0

        if clock == 1:
            i = 0
            j = 0
            print("corner pos = ", corner_pos)
            while True:
                always_looping += 1
                #x position - horizontal
                check_point[0] = current_point[0]+hor_adder[i]
                #y position - vertical
                check_point[1] = current_point[1]+vert_adder[i]

                #print("img1 current = ", img1[current_point[1]][current_point[0]])
                #print("img1 check = ", img1[check_point[1]][check_point[0]])
                #print("current position = ", current_point, " - position value = ", img1[current_point[1]][current_point[0]])
                #print("check position = ", check_point, " - position value = ", img1[check_point[1]][check_point[0]])
                #print("previous position = ", prev_point, " - position value = ", img1[prev_point[1]][prev_point[0]])
                #print("")
                #img1[x][y]
                if img1[check_point[1]][check_point[0]] > 100 and (check_point[1] != prev_point[1] or check_point[0] != prev_point[0]):
                    cv2.circle(blank_img, (check_point[0], check_point[1]), 1, (200, 167, 150), -1)
                    prev_point[0] = current_point[0]
                    prev_point[1] = current_point[1]
                    current_point[0] = check_point[0]
                    current_point[1] = check_point[1]
                    #print("new current position = ", current_point)

                    #if current_point[0] < max_x or current_point[1] > max_y:
                    #    print("Did I hit the corner? i = ", i)
                    times_loop += 1
                    average_i = ((average_i * (times_loop-1)) + i)/times_loop
                    print("times loop = ", times_loop)
                    #print("i = ", i)
                    #print("average_i = ", average_i)
                    i = 0

                    if 1 <= times_loop <= test_num_loops:  # and times_loop > 0
                        print("times loop = ", times_loop)
                        #print("corner pos = ", corner_pos)
                        #current_slope = (current_point[1] - corner_pos[1]) / (current_point[0] - corner_pos[0])
                        #print("current slope = ", current_slope)
                        print("j = ", j)
                        #running_average_values[times_loop] = current_slope
                        last_five_points[0][j] = current_point[0]
                        last_five_points[1][j] = current_point[1]
                        j += 1

                    if times_loop > test_num_loops:
                        #current_slope = (current_point[1] - corner_pos[1]) / (current_point[0] - corner_pos[0])
                        print("last five points before")
                        print(last_five_points)
                        for k in range(test_num_loops-1):
                            #running_average_values[k] = running_average_values[k + 1]
                            last_five_points[0][k] = last_five_points[0][k+1]
                            last_five_points[1][k] = last_five_points[1][k+1]
                        #print("last five points after")
                        #print(last_five_points)
                        #running_average_values[4] = current_slope
                        #running_average_slope = np.average(running_average_values)
                        #print("current point = ", current_point[0])
                        #for i in range(2):
                        #    print("last five position[i][4] = ", last_five_points[i][4])
                        #    last_five_points[i][4] = current_point[i]
                        last_five_points[0][test_num_loops-1] = current_point[0]
                        #print("last five position[0][4] = ", last_five_points[0][4])
                        last_five_points[1][test_num_loops-1] = current_point[1]
                        #print("last five position[0][4] = ", last_five_points[0][4])
                        #print("last five position[1][4] = ", last_five_points[1][4])
                        #print(current_point[1], " - ", last_five_points[1][0])
                        #print(last_five_points)
                        try:
                            last_five_slope = (last_five_points[1][test_num_loops-1] - last_five_points[1][0]) / (last_five_points[0][test_num_loops-1] - last_five_points[0][0])
                            print("last five slope = ", last_five_slope)
                            count = 0
                            slopes_queue.append(last_five_slope)
                            holder = 0
                            neg_reciprocal = 0
                            while count < len(slopes_queue):
                                holder = slopes_queue.pop(0)
                                neg_reciprocal = -1 / last_five_slope
                                if holder == neg_reciprocal:
                                    #go find intersection point
                                    print("found negative reciprocal!")
                                    #y=mx+b
                                    #b=y-mx
                                    m1 = holder
                                    m2 = last_five_slope
                                    b1 = corner_pos[1] - (m1 * corner_pos[0])
                                    b2 = current_point[1] - (m2 * current_point[0])
                                    exit = 1
                                    break
                                else:
                                    slopes_queue.append(holder)
                                    count += 1


                        except:
                            print("vertical position")
                        #print("running average slope = ", running_average_slope)

                else:
                    #print("else condition")
                    i += 1
                    if i > 7:
                        print("goodbye")
                        break

                #if times_loop > 15:
                #    num_slope_calcs += 1
                #    print("corner point = ", corner_pos)
                #    print("current point = ", current_point)
                #    current_slope = (current_point[1] - corner_pos[1])/(current_point[0] - corner_pos[0])
                #    average_slope = ((average_slope*(num_slope_calcs-1)) + current_slope) / num_slope_calcs
                #    print("current slope = ", current_slope)
                #    print("average slope = ", average_slope)
                #print("times_loop = ", times_loop)
                if exit == 1:
                    print("I can exit!")
                    break

                #elif times_loop > 25:
                #    print("hello")
                #    break
        #cv2.imshow("Bank Image", blank_img)
        #print("OUT OF WHILE LOOP", always_looping)
        return m1, b1

    def find_intersect_corners(self, equation1, equation2):
        #things will happen here
        corner_a, slope_a, intercept_a = equation1
        corner_b, slope_b, intercept_b = equation2
        corner_c = [0,0]
        corner_d = [0,0]
        print("intercept_a = ", intercept_a)
        print("slope_a = ", slope_a)
        print("intercept_b = ", intercept_b)
        print("slope_b = ", slope_b)
        try:
            if slope_b == (-1/slope_a):
                print("Slopes are negative reciprocals!")
                #y=mx+b
                #m1x+b1 = m2x+b2
                #(m1-m2)x = b2-b1
                #x = (b2-b1)/(m1-m2)

                #x=(y-b)/m
                #(y-b1)/m1 = (y-b2)/m2
                #(y-b1)m2 = (y-b2)m1
                #m2y-b1m2 = m1y-b2m1
                #y(m2-m1) = b1m2-b2m1
                #y = (b1m2-b2m1)/(m2-m1)

                corner_c[0] = int((intercept_b-intercept_a)/(slope_a-slope_b))
                corner_c[1] = int(((intercept_a*slope_b)-(intercept_b*slope_a))/((slope_b)-(slope_a)))
        except:
            print("I'm not really finding the right stuff")
        print("corner_c = ", corner_c)
        return corner_c

    def define_corners(self, all_corner_points):
        top_left = [all_corner_points[0][0],all_corner_points[1][0]]
        top_right = [all_corner_points[0][1],all_corner_points[1][1]]
        bottom_left = [all_corner_points[0][2],all_corner_points[1][2]]
        bottom_right = [all_corner_points[0][3],all_corner_points[1][3]]
        # print(top_left)
        # print(top_right)
        # print(bottom_left)
        # print(bottom_right)
        right_most = [[0,0],[0,0]]
        bottom_most = [[0,0],[0,0]]
        top_most = [[10000,0],[10000,0]]
        left_most = [[10000,0],[10000,0]]
        for k in range(2):
            for i in range(len(all_corner_points[0])):
                if right_most[k][0] < all_corner_points[0][i]:
                    if (k == 1 and right_most[0][1] != i) or k == 0:
                        right_most[k][0] = all_corner_points[0][i]
                        right_most[k][1] = i
                if bottom_most[k][0] < all_corner_points[1][i]:
                    if (k == 1 and bottom_most[0][1] != i) or k == 0:
                        bottom_most[k][0] = all_corner_points[1][i]
                        bottom_most[k][1] = i
                #print("top most = ", top_most[k][0])
                #print("all corner = ", all_corner_points[1][i])
                #print("k = ", k)
                if top_most[k][0] > all_corner_points[1][i]:
                    if (k == 1 and top_most[0][1] != i) or k == 0:
                        top_most[k][0] = all_corner_points[1][i]
                        top_most[k][1] = i
                if left_most[k][0] > all_corner_points[0][i]:
                    if (k == 1 and left_most[0][1] != i) or k == 0:
                        left_most[k][0] = all_corner_points[0][i]
                        left_most[k][1] = i

        # print("right most = ", right_most)
        # print("left most = ", left_most)
        # print("top most = ", top_most)
        # print("bottom most = ", bottom_most)

        for j in range(2):
            for i in range(2):
                if top_most[i][1] == left_most[j][1]:
                    top_left[0] = all_corner_points[0][top_most[i][1]]
                    top_left[1] = all_corner_points[1][top_most[i][1]]
                if top_most[i][1] == right_most[j][1]:
                    top_right[0] = all_corner_points[0][top_most[i][1]]
                    top_right[1] = all_corner_points[1][top_most[i][1]]
                if bottom_most[i][1] == left_most[j][1]:
                    bottom_left[0] = all_corner_points[0][bottom_most[i][1]]
                    bottom_left[1] = all_corner_points[1][bottom_most[i][1]]
                if bottom_most[i][1] == right_most[j][1]:
                    bottom_right[0] = all_corner_points[0][bottom_most[i][1]]
                    bottom_right[1] = all_corner_points[1][bottom_most[i][1]]

        # print("bottom right = ", bottom_right)
        # print("top right = ", top_right)
        # print("bottom left = ", bottom_left)
        # print("top left = ", top_left)
        return top_left, top_right, bottom_left, bottom_right

    def count_crossings(self, P1, P2, offset, img2, img3):
        crossings = 0
        incriment = 20
        rise = (P1[1] - P2[1])
        run = (P1[0] - P2[0])
        slope = rise/run
        start = [0, 0]
        end = [0, 0]
        # print("P1 = ", P1)
        # print("P2 = ", P2)
        start[0] = P1[0] + offset[0]
        start[1] = P1[1] + offset[1]
        end[0] = P2[0] - offset[0]
        end[1] = P2[1] - offset[1]
        # print("start = ", start)
        # print("end = ", end)
        #y=mx+b
        #b=y-mx
        intercept = P1[1] - (slope*P1[0])
        if slope != 0 and run != 0:
            print("slope = ", slope)
            print("intercept = ", intercept)
            print("offset[0] = ", offset[0])
            print("offset[1] = ", offset[1])
            if offset[0] == -10:
                prev_color = img2[int(P1[1] + offset[1])][int(P1[0] + offset[0])]
                x = int(start[0])
                #print("x = ", x)
                #print("end[0] = ", end[0])
                while x < end[0]:
                    #print("x = ", x)
                    #print("y = ", y)
                    y = int(((slope * x) + intercept) + offset[1])
                    #prev_color = img[P1[0]][y]
                    current_color = img2[y][x]
                    #print("current colors = ", current_color)
                    #print("prev color = ", prev_color)
                    if (current_color) != (prev_color):
                        crossings += 1
                        cv2.circle(img3, (x, y), 2, (140, 17, 50), -1)
                    x += incriment
                    cv2.circle(img3, (x, y), 2, (140, 17, 50), -1)
                    prev_color = current_color
            elif offset[1] == -10:
                prev_color = img2[int(P1[1] + offset[1])][int(P1[0] + offset[0])]
                y = int(start[1])
                #print("x = ", x)
                #print("end[0] = ", end[0])
                while y < end[1]:
                    #print("x = ", x)
                    #print("y = ", y)
                    #y = int(((slope * x) + intercept) + offset[1])
                    x = int(((y - intercept) / slope) + offset[0])
                    #prev_color = img[P1[0]][y]
                    current_color = img2[y][x]
                    #print("current colors = ", current_color)
                    #print("prev color = ", prev_color)
                    if (current_color) != (prev_color):
                        crossings += 1
                        cv2.circle(img3, (x, y), 2, (140, 17, 50), -1)
                    y += incriment
                    cv2.circle(img3, (x, y), 2, (140, 17, 50), -1)
                    prev_color = current_color

        # cv2.imshow("Finding Crossings", img3)
        print("crossings = ", crossings)
        return crossings

    def four_point_transformation(self, rectangle_points, image):
        tl, tr, bl, br = rectangle_points
        top_width = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        bottom_width = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        left_height = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        right_height = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        max_width = max(int(top_width), int(bottom_width))
        # max_height = max(int(left_height), int(right_height))
        max_height = int(max_width * (self.gridLength / self.gridWidth))
        rect_points = np.array(rectangle_points, dtype="float32")
        dst = np.array([[0, 0],[max_width - 1, 0],[0, max_height - 1],[max_width - 1, max_height - 1]], dtype="float32")
        M = cv2.getPerspectiveTransform(rect_points, dst)
        # print("M")
        print(M)
        warped = cv2.warpPerspective(image, M, (max_width, max_height))
        # cv2.imshow("Warped Image", warped)
        return warped

    def confirm_corner(self, point1, point2, image):
        rise = point1[1] - point2[1]
        run = point1[0] - point2[0]
        #print("point1 = ", point1)
        #print("point2 = ", point2)
        #print("rise = ", rise)
        #print("run = ", run)
        slope = 0
        if rise != 0 and run != 0:
            slope = rise/run
        #y=mx+b
        intercept = point1[1] - (slope * point1[0])
        #print("slope = ", slope)
        #print("intercept = ", intercept)
        x1 = point1[0] - 1
        x2 = point2[0] + 1
        y1 = point1[1] - 1
        y2 = point2[1] + 1
        down_on_up = [-1,0,1]
        corner1 = [point1[1], point1[0]]
        corner2 = [point2[1], point2[0]]
        test_counter = 0
        #if rise > run:
        if abs(slope) >= 1:
            #print("slope >= 1")
            while True:
                hit = 0
                #y=mx+b
                #y-b=mx
                #(y-b)/m=x
                #print("y = ", y1)
                #print("b = ", intercept)
                #print("m = ", slope)
                x1 = int((y1 - intercept) / slope)
                for i in range(3):
                    #print("test val = ", x1+down_on_up[i])
                    if image[y1][x1+down_on_up[i]] > 200: # or image[x2][y2+down_on_up[i]] > 200:
                        hit += 1
                    #do something here
                if hit == 0:
                    x1 = int(((y1+1) - intercept) / slope)
                    corner1 = [x1, (y1+1)]
                    break
                y1 -= 1
            while True:
                hit = 0
                x2 = int((y2 - intercept) / slope)
                for i in range(3):
                    if image[y2][x2+down_on_up[i]] > 200: # or image[x2][y2+down_on_up[i]] > 200:
                        hit += 1
                    #do something here
                if hit == 0:
                    x2 = int((y2 - intercept) / slope)
                    corner2 = [x2, (y2-1)]
                    break
                y2 += 1
        elif abs(slope) < 1 and abs(slope) > 0:
            #print("slope < 1 and slope > 0")
            while True:
                test_counter += 1
                #print("test_counter = ", test_counter)
                hit = 0
                y1 = int((slope * x1) + intercept)
                #print("y1 = ", y1)
                #print("x1 = ", x1)
                for i in range(3):
                    if image[y1+down_on_up[i]][x1] > 200: # or image[x2][y2+down_on_up[i]] > 200:
                        hit += 1
                    #do something here
                if hit == 0:
                    y1 = int((slope * (x1+1)) + intercept)
                    corner1 = [(x1+1), y1]
                    break
                x1 -= 1
            while True:
                hit = 0
                y2 = int((slope * x2) + intercept)
                #print("y2 = ", y2)
                #print("x2 = ", x2)
                for i in range(3):
                    if image[y2+down_on_up[i]][x2] > 200: # or image[x2][y2+down_on_up[i]] > 200:
                        hit += 1
                    #do something here
                if hit == 0:
                    y2 = int((slope * (x2-1)) + intercept)
                    #print("y2 = ", y2)
                    #print("x2 = ", x2)
                    corner2 = [(x2-1), y2]
                    break
                x2 += 1
        elif rise == 0:
            while True:
                hit = 0
                for i in range(3):
                    if image[y1+down_on_up[i]][x1] > 200: # or image[x2][y2+down_on_up[i]] > 200:
                        hit += 1
                    #do something here
                if hit == 0:
                    corner1 = [(x1+1), y1]
                    break
                x1 -= 1
            while True:
                hit = 0
                for i in range(3):
                    if image[y2+down_on_up[i]][x2] > 200: # or image[x2][y2+down_on_up[i]] > 200:
                        hit += 1
                    #do something here
                if hit == 0:
                    corner2 = [(x2-1), y2]
                    break
                x2 += 1
        elif run == 0:
            while True:
                test_counter += 1
                #print("test counter = ", test_counter)
                hit = 0
                for i in range(3):
                    if image[y1][x1+down_on_up[i]] > 200: # or image[x2][y2+down_on_up[i]] > 200:
                        hit += 1
                    #do something here
                if hit == 0:
                    corner1 = [x1, (y1+1)]
                    break
                y1 -= 1
                #print("y1 = ", y1)
            test_counter = 0
            while True:
                test_counter += 1
                #print("test counter = ", test_counter)
                hit = 0
                for i in range(3):
                    if image[y2][x2+down_on_up[i]] > 200: # or image[x2][y2+down_on_up[i]] > 200:
                        hit += 1
                    #do something here
                if hit == 0:
                    corner2 = [x2, (y2-1)]
                    break
                y2 += 1
        #print("starting point 1 = ", point1)
        #print("starting point 2 = ", point2)
        #print("corner1 = ", corner1)
        #print("corner2 = ", corner2)
        return corner1, corner2


# cap = cv2.VideoCapture(0)
# while True:
#     grabCal = False
#     _, frame = cap.read()
#     print("enter")
#     frame = cv2.flip(frame, 0)
#     frame = cv2.flip(frame, 1)
#     Calibration(frame, grabCal)
#     print("exit")
#     cv2.imshow("raw image", frame)
#     time.sleep(0.1)
#     if cv2.waitKey(25) & 0xFF == ord('q'):
#         break
#
# cap.release()
# cv2.destroyAllWindows()
# #
# filename = "calibration_grid7.jpg"
# img = cv2.imread(filename)
# Calibration(img, 0)
