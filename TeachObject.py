import cv2
import numpy as np
from tkinter import *
import tkinter.ttk as ttk
from PIL import Image, ImageTk


class ObjectAssist:
    def __init__(self, filename):

        print("start of object assist")
        current_model = 1
        ObjectAssist.center = 0
        self.grey_center = 0
        self.obj_def = Toplevel()
        # self.obj_def = Tk()
        self.obj_def.geometry('642x582')
        self.obj_def.title("Define Vision Object")
        self.canvas = Canvas(self.obj_def, width=640, height=480, bg="black")
        self.canvas.grid(row=0, column=0, rowspan=2, columnspan=4)
        self.save_model_button = Button(self.obj_def, text="Save Model",
                                        command=lambda: self.update_parameters_and_sliders())
                                        #command=lambda: MainProgram.VisionPage.update_sliders(current_model))
        self.save_model_button.grid(row=3, column=1)
        ObjectAssist.black_on_white = True
        ObjectAssist.center_loc = 0
        ObjectAssist.maxArea = 0
        ObjectAssist.Circularity = 0
        ObjectAssist.Circumfence = 0
        self.black_or_white_button = Button(self.obj_def, text="Black on White",
                                            command=lambda: self.switch())
        self.black_or_white_button.grid(row=3, column=3)
        self.pilImage = Image.open(filename)
        #tkimg = ImageTk.PhotoImage(image=pilImage)
        self.tkimg = ImageTk.PhotoImage(file=filename)
        image_canvas = self.canvas.create_image(0, 0, image=self.tkimg, anchor='nw')
        #image_canvas.pack()
        self.count = 1
        self.canvas.bind("<Button-1>", self.start_box)
        self.canvas.bind("<B1-Motion>", self.end_box)
        ObjectAssist.croppedImage = self.canvas.bind("<ButtonRelease-1>", self.compute)
        ObjectAssist.red_blue_green_values = [0, 0, 0, 0, 0, 0]
        ObjectAssist.min_max_area = [100, 10000]
        self.black_white_exist = False
        mainloop()

    def start_box(self, event):
        self.start = [event.x, event.y]
        print("start rectangle")

    def switch(self):
        if ObjectAssist.black_on_white is True:
            self.black_or_white_button.configure(text="White on Black")
            ObjectAssist.black_on_white = False
            max_center = max(self.grey_center)
            ObjectAssist.center_loc = self.grey_center.index(max_center)
        elif ObjectAssist.black_on_white is False:
            self.black_or_white_button.configure(text="Black on White")
            ObjectAssist.black_on_white = True
            min_center = min(self.grey_center)
            ObjectAssist.center_loc = self.grey_center.index(min_center)
        if self.black_white_exist == True:
            self.show_black_white_img(self.res2)

    def end_box(self, event):
        self.end = [event.x, event.y]
        try:
            self.canvas.delete(self.rectangle)
        except:
            pass
        # length = abs(self.end[0] - self.start[0])
        # width = abs(self.end[1] - self.start[1])
        # size = length * width
        # print("rectangle size = ", size)
        self.rectangle = self.canvas.create_rectangle(self.start, self.end)
        #print("end rectangle")

    def compute(self, event):
        new_image = self.pilImage.crop((self.start[0], self.start[1], self.end[0], self.end[1])).convert('RGB')
        # self.new_cv_img = np.array(new_image).astype(np.uint8)
        # self.new_cv_img = self.new_cv_img[:, :, ::-1].copy().astype(np.uint8)
        self.new_cv_img = np.array(new_image).astype(np.uint8)
        self.new_cv_img = self.new_cv_img[:, :, ::-1].copy().astype(np.uint8)
        ObjectAssist.croppedRawImg = self.new_cv_img.copy()
        cv2.imshow("cropped image", self.new_cv_img)
        self.cluster(self.new_cv_img)
        return self.new_cv_img

    def cluster(self, img):
        #cluster function uses K nearest means to determine the two major colors present in the image and grabs rgb or hsv

        #code for RGB
        flat_img = img.reshape((-1, 3))
        # convert to np.float32
        # flat_img = np.float32(flat_img)
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        # k = 2
        # ret, label, self.center = cv2.kmeans(flat_img, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        #
        # ObjectAssist.center = np.uint8(self.center)
        # res = ObjectAssist.center[label.flatten()]
        # self.res2 = res.reshape((img.shape))
        # self.grey_center = [0] * len(ObjectAssist.center)
        #
        # for i in range(0, k):
        #     self.grey_center[i] = sum(ObjectAssist.center[i]) / 3
        #
        # self.show_black_white_img(self.res2)
        # cv2.imshow('res2', self.res2)

        #code for HSV
        #img_hsv = cv2.cvtColor(flat_img, cv2.COLOR_BGR2HSV)
        k = 2
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        flat_img_hsv = img_hsv.reshape((-1, 3))
        # convert to np.float32
        flat_img_hsv = np.float32(flat_img_hsv)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret_hsv, label_hsv, self.center_hsv = cv2.kmeans(flat_img_hsv, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        ObjectAssist.center = np.uint8(self.center_hsv)
        res_hsv = ObjectAssist.center[label_hsv.flatten()]
        self.res2 = res_hsv.reshape((img_hsv.shape))
        self.grey_center = [0] * len(ObjectAssist.center)
        print(ObjectAssist.center)

        for i in range(0, k):
            self.grey_center[i] = sum(ObjectAssist.center[i])/3

        self.show_black_white_img(self.res2)
        cv2.imshow('res2_hsv', cv2.cvtColor(self.res2, cv2.COLOR_HSV2RGB))

        # self.show_black_white_img(self.res2_hsv)
        # cv2.imshow('res2', self.res2_hsv)

    def show_black_white_img(self, input_img):
        try:
            cv2.destroyWindow("black_white_img")
        except:
            pass
        margin = 10
        upperMax = 255
        lowerMin = 0
        upperMargin = (upperMax - margin) * (margin / 100)
        lowerMargin = (margin - lowerMin) * (margin / 100)

        #use for RGB
        # upper = ObjectAssist.center[ObjectAssist.center_loc] + upperMargin
        # lower = ObjectAssist.center[ObjectAssist.center_loc] - lowerMargin
        # print("upper = ", upper)
        # print("lower = ", lower)

        #use for HSV
        upper = ObjectAssist.center[ObjectAssist.center_loc] + upperMargin
        lower = ObjectAssist.center[ObjectAssist.center_loc] - lowerMargin
        print("upper = ", upper)
        print("lower = ", lower)

        # upper = ObjectAssist.center[ObjectAssist.center_loc] + margin
        # lower = ObjectAssist.center[ObjectAssist.center_loc] - margin
        # print("upper = ", upper)
        # print("lower = ", lower)

        intermediate = cv2.inRange(input_img, lower, upper)
        #intermediate = cv2.inRange(input_img, lower, upper)
        ret, black_white_img = cv2.threshold(intermediate, 127, 255, cv2.THRESH_BINARY_INV)
        self.black_white_exist = True
        contours, hierarchy = cv2.findContours(np.array(intermediate),
                                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(self.new_cv_img, contours, -1, (0, 255, 0), 3)

        c = max(contours, key=cv2.contourArea)
        ObjectAssist.maxArea = cv2.contourArea(c)
        print("area = ", ObjectAssist.maxArea)
        minCircle = cv2.minEnclosingCircle(c)
        print(ObjectAssist.Circularity)
        center = [0, 0]
        center[0] = int(minCircle[0][0])
        center[1] = int(minCircle[0][1])
        radius = int(minCircle[1])
        minCircleArea = 3.14 * (radius * radius)
        ObjectAssist.Circularity = ObjectAssist.maxArea / minCircleArea
        print("circularity = ", ObjectAssist.Circularity)
        cv2.circle(self.new_cv_img, (center[0], center[1]), radius, (255, 0, 0))
        cv2.imshow('contour image', self.new_cv_img)
        cv2.imshow('black_white_img', black_white_img)

    def update_parameters_and_sliders(self):
        #red_blue_green_values = [0,0,0,0,0,0]
        #min_max_area = [100,10000]
        j = 0
        #red-blue-green, upper-lower

        for i in range(6):
            if i % 2 == 0:
                print(self.center[ObjectAssist.center_loc][j])
                ObjectAssist.red_blue_green_values[i] = ObjectAssist.center[ObjectAssist.center_loc][j] + int(self.center[ObjectAssist.center_loc][j] * 0.2)
            elif i % 2 == 1:
                ObjectAssist.red_blue_green_values[i] = ObjectAssist.center[ObjectAssist.center_loc][j] - int(self.center[ObjectAssist.center_loc][j] * 0.2)
                j += 1
        self.obj_def.destroy()
        #cv2.destroyAllWindows()
        cv2.destroyWindow("black_white_img")
        cv2.destroyWindow("cropped image")
        cv2.destroyWindow("res2")

        #MainProgram.VisionPage.set_master_vision_model(model_selection, min_max_area, red_blue_green_values)
        #MainProgram.VisionPage.update_sliders(model_selection)


# filename = "findImagesFolder/opencv_frame_0.png"
# ObjectAssist(filename)