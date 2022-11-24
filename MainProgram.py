import cv2
import numpy as np
import sys
from queue import Queue
import math
from tkinter import *
import tkinter.ttk as ttk
from PIL import Image, ImageTk
import threading
import sys
import serial
import serial.tools.list_ports
import time
import InverseKinematics
import Calibrate
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import VisionSlidersHelper
import TeachObject
import PatternBuilder
import json
import ProgramInfo



def popupimg(img1, msg):
    try:
        print("inside the try statement")
        if Toplevel.winfo_exists(popup) == 1:
            print("inside the if statement")
            popup.img1 = img1
            label.configure(image=img1)
            popup.mainloop()
    except:
        popup = Toplevel()
        popup.wm_title(msg)
        popup.img1 = img1
        label = Label(popup, image=img1)
        #label.configure(image=img1)
        label.pack(side="top", fill="x", pady=10)
        popup.mainloop()


def popupmsg(tit, msg1):
    popmsg = Toplevel()
    popmsg.wm_title(tit)
    label = Label(popmsg, text=msg1)
    #label.configure(image=img1)
    label.pack(side="top", fill="x", pady=10)
    popmsg.mainloop()


class MaterRobotProgram(Tk):

    def __init__(self):
        Tk.__init__(self)
        self.container = Frame(self)
        self.container.pack(sid="top", fill="both", expand=True)
        self.container.grid_rowconfigure(0, weight=1)
        self.container.grid_columnconfigure(0, weight=1)
        self.vision_model_list = Listbox(self, bg='white', fg='black', bd=2, width=50, height=10, relief='solid')
        #self.start_joint_pos[0] = 0
        #self.start_joint_pos[1] = -75
        #self.start_joint_pos[2] = 90
        #self.start_joint_pos[3] = 0
        #self.start_joint_pos[4] = 75
        #self.start_joint_pos[5] = 0
        self.times_through_vision_loop = 0
        self.angles = np.zeros(6)
        self.angles[0] = 0
        self.angles[1] = 60
        self.angles[2] = 0
        self.angles[3] = 0
        self.angles[4] = -60
        self.angles[5] = 0
        self.con_decide = -1
        self.arduino_data = ''
        self.frames = {}
        self.vision_save = 0
        self.vision_model_index = 0
        self.good_calibration = 0
        self.vision_area_min = 100
        self.vision_area_max = 5000
        self.button_held_down = 0
        self.vision_rbg_values = [255, 0, 255, 0, 255, 0]
        InverseKinematics.Kinematics(2, self.angles)
        self.reverse_start = InverseKinematics.Kinematics.position
        self.all_pos = InverseKinematics.Kinematics.all_positions
        #self.vision_list_save_var = StringVar()
        self.vision_list_save_var = []
        self.manual_programming_list_save_var = []
        #self.reverse_start = [195, 0.04, 150, math.radians(0), math.radians(180), math.radians(180)]
        #InverseKinematics.Kinematics(1, self.reverse_start)
        #self.angles = InverseKinematics.Kinematics.joints
        print("reverse start = ", self.reverse_start)
        InverseKinematics.Kinematics(1, self.reverse_start)
        print("reverse start = ", self.reverse_start)
        self.prev_angles = InverseKinematics.Kinematics.joints
        self.vision_model = [0]*10
        self.vision_model_master = np.array([[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]])
        self.save_capture_position = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])
        self.save_place_position = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])
        self.save_program_positions = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])
        self.save_position_location = np.array([0,0])
        self.save_position_dictionary = {}
        self.capture_position_counter = 1
        self.place_position_counter = 1
        self.frame_2 = {}
        self.cameraCalibrated = False
        self.calibrationPixelPerMm = 0
        self.jsonFileName = "robotSettings.json"
        self.jsonVisionFileName = "visionModels.json"
        self.programVariables = {}
        self.visionModel = {}
        self.arduinoConnected = False
        self.liveVisionTestMode = False
        self.calLocations = ['xCal', 'yCal', 'zCal', 'wCal', 'pCal', 'rCal']
        # self.programVariables['calibration'] = {'calibrationPixelPerMm': 0,
        #                                         'calibrationStatus': False,
        #                                         'robotCalibration': {'xCal': 0,
        #                                                              'yCal': 0,
        #                                                              'zCal': 0,
        #                                                              'wCal': 0,
        #                                                              'pCal': 0,
        #                                                              'rCal': 0}}
        fileOpen = open(self.jsonFileName)
        visionFileOpen = open(self.jsonVisionFileName)

        self.visionModel['model1'] = {
            'minArea': -1,
            'maxArea': -1,
            'blackOrWhite': True,
            'upperR': 255,
            'lowerR': 0,
            'upperG': 255,
            'lowerG': 0,
            'upperB': 255,
            'lowerB': 0
        }

        self.programVariables['calibration'] = {
            'calibrationPixelPerMm': 0,
            'calibrationStatus': False,
            'robotCalibration': {
                self.calLocations[0]: 0,
                self.calLocations[1]: 0,
                self.calLocations[2]: 0,
                self.calLocations[3]: 0,
                self.calLocations[4]: 0,
                self.calLocations[5]: 0,
                'robotCalibrationStatus': FALSE
            },
            'jointPotZeroValues': {
                'J1': 0,
                'J2': 0,
                'J3': 0,
                'J4': 0,
                'J5': 0,
                'J6': 0
            }
        }

        self.programVariables = json.load(fileOpen)
        print(self.programVariables['calibration'])
        for i in self.programVariables:
            print(self.programVariables)
        print("calibration status = ", self.programVariables['calibration']['calibrationStatus'])

        #self.reverse_change = np.zeros(6)

        #for F in ((HomePage), (VisionPage), (ProgramingPage), (SetIO)):
        #    frame = F(self.container, self)
        #    self.frame_2[F] = frame
        #    frame.grid(row=0, column=0, sticky="nsew")

        self.frames = {
            "HomePage": HomePage,
            "VisionPage": VisionPage,
            "ProgramingPage": ProgramingPage,
            "SetIO": SetIO
        }

        #self.show_frame(HomePage, None)
        self.show_frame_1("HomePage")

    def show_frame(self, cont, arg=None):
        frame = self.frames[cont]
        print("cont = ", cont)
        print("frame = ", frame)
        frame.tkraise()
        if arg:
            print("there is an argument")
            print(arg)
            frame.arg = arg
        if arg == 2:
            print("this is working")
            a = ProgramingPage(None, None)
            a.green_circle(self.con_decide)

    def show_frame_1(self, page_name):
        print("button pressed!")
        # destroy the old frame
        for child in self.container.winfo_children():
            child.destroy()
        print("past for loop")

        # create the new frame
        frame_class = self.frames[page_name]
        print("frame_class created")
        frame = frame_class(parent=self.container, controller=self)
        print("frame_class definition")
        frame.pack(fill="both", expand=True)
        print("end of method")

    def get_frame(self, page_class):
        page = self.frames[page_class](parent=self.container, controller=self)
        #return self.frames[page_class]
        return page


class HomePage(Frame):

    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        home_label = Label(self, text = "This is the Home Page!")
        home_label.grid(row=1, column=1)
        vision_button = Button(self, text="Vision Program", command=lambda: controller.show_frame_1("VisionPage"))
        vision_button.grid(row=2, column=1)
        program_button = Button(self, text="Manual Programing", command=lambda: controller.show_frame_1("ProgramingPage"))
        program_button.grid(row=3, column=1)
        io_button = Button(self, text="Setup I/O", command=lambda: controller.show_frame_1("SetIO"))
        io_button.grid(row=4, column=1)
        self.connect_button = Button(self, text='Connect', border=0, bg='grey', command=self.connect_press)  #
        self.connect_button.grid(row=40, column=10, padx=2, pady=2)
        # holder = []
        com_selection = StringVar()
        com_port = ttk.Combobox(self, textvariable=com_selection, values=self.serial_ports())  #
        com_port.grid(row=40, column=9, padx=2, pady=2)
        self.holder = com_selection
        print("create canvas")

        ProgramingPage.green_circle(self, 0)
        # calX1Dot = 2
        # calX2Dot = 2
        # calY1Dot = 13
        # calY2Dot = 13
        # spacing = 12
        # connected_label = Label(self, text="Connection Status: ")
        # connected_label.grid(row=1, column=15)
        # cameraCalLabel = Label(self, text="Camera Calibration Status: ")
        # cameraCalLabel.grid(row=2, column=15)
        # robotCalLabel = Label(self, text="Robot Calibration Status: ")
        # robotCalLabel.grid(row=3, column=15)
        # self.canvas = Canvas(self, width=12, height=(calY2Dot+(calY1Dot+spacing)*2), borderwidth=0, bg="white")
        # self.canvas.grid(row=1, column=20, columnspan=2, rowspan=3)
        # self.connected_status_circle = self.canvas.create_oval(calX1Dot, calX2Dot, calY1Dot, calY2Dot, fill="red", outline="black")
        # self.cameraCalibrationStatus = self.canvas.create_oval(calX1Dot, (calX2Dot+calY1Dot+spacing), calY1Dot, (calY2Dot+calY1Dot+spacing), fill="red", outline="black")
        # self.robotCalibrationStatus = self.canvas.create_oval(calX1Dot, (calX2Dot+(calY1Dot+spacing)*2), calY1Dot, (calY2Dot+(calY1Dot+spacing)*2), fill="red", outline="black")
        # if self.controller.arduinoConnected is True:
        #     self.canvas.itemconfig(self.connected_status_circle, fill="green")
        # if self.controller.programVariables['calibration']['calibrationStatus'] is True:
        #     self.canvas.itemconfig(self.cameraCalibrationStatus, fill="green")

        #self.com_number = 'COM' + str(self.holder.get())[3]

    def serial_ports(self):
        time.sleep(0.1)
        return serial.tools.list_ports.comports()

    def connect_press(self):
        self.controller.con_decide = 3
        num = str(self.holder.get())[3]
        self.com_number = 'COM' + str(num)
        self.test = self.connect_test(self.com_number)

    def connect_test(self, com):
        connectMessage = "connect"
        jointString = "ABCDEF"
        for i in range(0,6):
            currentJoint = "J" + str(i+1)
            potLoc = self.controller.programVariables['calibration']['jointPotZeroValues'][currentJoint]
            connectMessage = connectMessage + jointString[i] + str(potLoc)
        print("connectMessage = ", connectMessage)
        timeout = 10
        kill = 0
        print(com)
        self.controller.arduino_data = serial.Serial(port=com, baudrate=9600, timeout=0.1)
        time.sleep(2)
        self.controller.arduino_data.write('connect'.encode())
        time.sleep(1)
        con_test = self.controller.arduino_data.readline().decode()
        print(con_test)
        while "Connected" not in con_test:
            time.sleep(0.3)
            self.controller.arduino_data.write('connect'.encode())
            time.sleep(0.3)
            con_test = self.controller.arduino_data.readline().decode()
            kill += 1
            if kill > timeout:
                print("turn button grey")
                self.connect_button.configure(bg='grey')
                self.controller.con_decide = 2
                return False
        self.connect_button.configure(bg='blue')
        print("turn button blue")
        self.controller.con_decide = 1
        self.controller.arduinoConnected = True
        print("turn green")
        self.canvas.itemconfig(self.connected_status_circle, fill="green")
        return True

        #if "Connected" in con_test:
        #    self.connect_button.configure(bg='blue')
        #    print("turn button blue")
        #    self.controller.con_decide = 1
        #    print("turn green")
        #    self.canvas.itemconfig(self.connected_status_circle, fill="green")
        #    return True
        #else:
        #    print("turn button grey")
        #    self.connect_button.configure(bg='grey')
        #    self.controller.con_decide = 2
        #    return False

    def arduino_access(self, message):
        print(message)
        response = ''
        if self.con_decide == 1:
            self.controller.arduino_data.write(message)
            time.sleep(2)
            response = self.controller.arduino_data.readline().decode()
            while response != "done":
                time.sleep(2)
                response = self.controller.arduino_data.readline().decode()
                time.sleep(2)
        #else:
        #    popupmsg("Connection Error", "You are not connected to the arduino")
        print("con_decide = ", self.con_decide)


class ProgramingPage(Frame):

    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        title_label = Label(self, text="Manual Programing Page", bg="white")
        title_label.grid(row=1, column=1)
        home_button = Button(self, text="Home Screen", bg="grey", border=0, command=lambda: self.go_home_save())

        #controller.show_frame_1("HomePage")
        home_button.grid(row=2, column=1, padx=10, pady=2)
        joint_label = Label(self, text="Joint Jogging")
        joint_label.grid(row=3, column=1, columnspan=3)
        linear_label = Label(self, text="Linear Jogging")
        linear_label.grid(row=4, column=5, padx=10)
        self.program_text_xyz = f"x= {str(self.controller.reverse_start[0])}  y= {str(self.controller.reverse_start[1])}" \
                            f"   z= {str(self.controller.reverse_start[2])}"
        self.program_text_wpr = f"w= {str(self.controller.reverse_start[3])}  p= {str(self.controller.reverse_start[4])}" \
                                f"   r= {str(self.controller.reverse_start[5])}"
        # self.program_text_wpr = f"w= {str(math.degrees(self.controller.reverse_start[3]))}  p= {str(math.degrees(self.controller.reverse_start[4]))}" \
        #                     f"   r= {str(math.degrees(self.controller.reverse_start[5]))}"
        #self.program_text_xyz = "x=0  y=0   z=0"
        self.text_color = "blue"
        self.cover = Frame(self, width=325, height=450, relief='solid')
        self.local_manual_programming_list = StringVar(value=self.controller.manual_programming_list_save_var)
        self.program_label = Listbox(self, bg='white', fg=self.text_color, listvariable=self.local_manual_programming_list, bd=2, width=60, height=30, relief='solid')
        self.program_label.grid(row=2, column=25, padx=10, rowspan=100, columnspan=100)
        self.program_label.bind('<Double-1>', lambda x: self.openPositionInfo())

        #fig = plt.figure()
        #ax = fig.add_subplot(111, projection='3d')
        #self.sim_plot = FigureCanvasTkAgg(fig, controller)
        #self.sim_plot.get_tk_widget().pack(side=RIGHT) #, fill=BOTH
        #self.sim_plot.draw()

        self.cnt_var = 0
        self.current_loc_label = Label(self, text="Current Position: ", bg="white")
        self.current_loc_label.grid(row=2, column=3)
        self.current_loc = Label(self, text=self.program_text_xyz, bg="white")
        self.current_loc.grid(row=2, column=4, columnspan=2)
        self.current_loc_wpr = Label(self, text=self.program_text_wpr, bg="white")
        self.current_loc_wpr.grid(row=3, column=4, columnspan=2)

        self.save_label = [None]*20
        self.saved_positions = np.zeros((20, 3), np.uint8)
        empty = None
        self.program_text_xyz = ""
        self.program_text_wpr = ""
        self.save_loc = 0
        self.job = None
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.joint_pos = np.zeros(6)
        self.joint_pos[0] = 0
        self.joint_pos[1] = -60
        self.joint_pos[2] = 0
        self.joint_pos[3] = 0
        self.joint_pos[4] = 60
        self.joint_pos[5] = 0
        self.cartesian_pos = np.zeros(6)
        self.temp_capture_position = np.array([0, 0, 0, 0, 0, 0])
        self.temp_place_position = np.array([0, 0, 0, 0, 0, 0])

        #self.save_label = Label(self)
        #linear_buttons("direction", y_placement, x_placement)
        self.linear_buttons("x+", 8, 6)
        self.linear_buttons("x-", 8, 4)
        self.linear_buttons("y+", 6, 5)
        self.linear_buttons("y-", 10, 5)
        self.linear_buttons("z+", 6, 9)
        self.linear_buttons("z-", 10, 9)
        self.joint_buttons("J1-", 4, 1)
        self.joint_buttons("J1+", 4, 3)
        self.joint_buttons("J2-", 5, 1)
        self.joint_buttons("J2+", 5, 3)
        self.joint_buttons("J3-", 6, 1)
        self.joint_buttons("J3+", 6, 3)
        self.joint_buttons("J4-", 7, 1)
        self.joint_buttons("J4+", 7, 3)
        self.joint_buttons("J5-", 8, 1)
        self.joint_buttons("J5+", 8, 3)
        self.joint_buttons("J6-", 9, 1)
        self.joint_buttons("J6+", 9, 3)

        self.joint_position_readout(self.controller.angles)
        act1 = 1
        act2 = 2
        act3 = 3
        self.y = -1
        self.work_around = 1
        self.kill = 0
        self.playback_loop = IntVar()
        self.playback_loop.set(value=0)
        self.record_button = Button(self, text="Record Position", command=lambda: self.save_position(act1, empty)) #
        self.record_button.grid(row=16, column=4, columnspan=2)
        self.delete_button = Button(self, text='Delete Position', command=lambda: self.save_position(act3, empty)) # self.program_label.delete(self.y)
        self.delete_button.grid(row=16, column=2, columnspan=2)
        self.insert_button = Button(self, text='Insert Position', command=lambda: self.save_position(act2, empty))  # self.program_label.delete(self.y)
        self.insert_button.grid(row=16, column=6, columnspan=2)

        #self.play_button = Button(self, text='Play', command=lambda: self.run_program())
        self.play_button = Button(self, text='Play', command=lambda: threading.Thread(target=self.run_program).start())
        self.play_button.grid(row=18, column=4, columnspan=2)
        self.loop_checkbox = Checkbutton(self, text="Loop Program", variable=self.playback_loop, onvalue=1, offvalue=0)
        self.loop_checkbox.grid(row=18, column=6, columnspan=2)
        self.reverse = [195, 0.04, 150, math.radians(0), math.radians(180), math.radians(180)]
        self.reverse_change = np.zeros(6)
        self.moveL_button = Button(self, text="moveL", bg="blue", command=lambda: self.move_type(act2))
        self.moveL_button.grid(row=14, column=4, columnspan=2)
        self.moveJ_button = Button(self, text="moveJ", command=lambda: self.move_type(act1))
        self.moveJ_button.grid(row=14, column=2, columnspan=2)
        self.moveP_button = Button(self, text="moveL", command=lambda: self.move_type(act3))
        self.moveP_button.grid(row=14, column=6, columnspan=2)
        self.velocity = 5
        self.out = 0
        self.start_press = 0
        self.window_open = 0
        self.update_ready = 0
        self.sim_button = Button(self, text="Simulation View", command=lambda: self.simulate(self.controller.all_pos))
        self.sim_button.grid(row=2, column=6)
        self.sim_button.configure(state=DISABLED)
        self.calibrationButton = Button(self, text="Calibrate Robot", command=lambda: self.storeRobotCalibration()) #
        self.calibrationButton.grid(row=3, column=6)
        self.zeroMotorsButton = Button(self, text="Zero Motors", command=lambda: self.zeroMotorPositions())
        if self.controller.arduinoConnected is True:
            self.zeroMotorsButton.configure(state=NORMAL)
        else:
            self.zeroMotorsButton.configure(state=DISABLED)
        self.zeroMotorsButton.grid(row=4, column=6)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.kill_button = Button(self, text="Stop", command=lambda: self.change_variable()) #threading.Thread(target=self.run_program).join()
        self.kill_button.grid(row=18, column=2, columnspan=2)

        vision_page = self.controller.get_frame('VisionPage')
        #self.num_vision_options = vision_page.vision_model_list.size()
        #self.num_vision_options = vision_page.vision_size.get()
        #self.num_vision_options = vision_page.vision_model_list.size()
        self.num_vision_options = len(self.controller.vision_list_save_var)
        print("num_vision_options = ", self.num_vision_options)

        self.add_vision_model_button = Button(self, text="Add Vision", command=lambda: self.save_position(act1, 2)) #
        self.add_vision_model_button.grid(row=20, column=6, columnspan=2)
        self.modify_button = Button(self, text="Modify Vision", command=lambda: self.save_position(act1, 3))
        self.modify_button.grid(row=22, column=6, columnspan=2)

        if self.num_vision_options >= 1:
            self.vision_option_selected = StringVar()
            self.vision_options = [0] * self.num_vision_options
            for i in range(self.num_vision_options):
                self.vision_options[i] = str(vision_page.vision_model_list.get(i))
            #self.vision_dropdown = ttk.Combobox(self, self.vision_option_selected, *self.vision_options)
            self.vision_dropdown = ttk.Combobox(self, values=self.vision_options) #, self.vision_option_selected
            #self.vision_dropdown['values'] = [self.vision_options]
            self.vision_dropdown.grid(row=20, column=4, columnspan=2)
            self.add_vision_model_button.configure(state=ACTIVE)
            self.modify_button.configure(state=ACTIVE)
        elif self.num_vision_options == 0:
            self.no_vision_model_label = Label(self, text="No Vision Models Available", bg='grey')
            self.no_vision_model_label.grid(row=20, column=4, columnspan=2)
            self.add_vision_model_button.configure(state=DISABLED)
            self.modify_button.configure(state=DISABLED)

        #if self.program_label.curselection() == None:
        #    self.modify_button.configure(state=DISABLED)
        #else:
        #    self.modify_button.configure(state=ACTIVE)

        self.vision_dropdown_label = Label(self, text="Selection Vision Model")
        self.vision_dropdown_label.grid(row=20, column=2, columnspan=2)

        # self.pattern_button = Button(self, text="Pattern Tool", command=lambda: PatternBuilder.Pattern(self.controller.reverse_start)) #
        self.pattern_button = Button(self, text="Pattern Tool",
                                     command=lambda: patternFunctions)  #
        self.pattern_button.grid(row=22, column=2, columnspan=2)
        self.green_circle(0)
        # mainloop()

        #self.num_vision_options = VisionPage.v
        #self.vision_options = VisionPage.
        #self.vision_dropdown = OptionMenu(self,)
        #self.t2 = threading.Thread(target=self.monitor1())
        # self.t1.start()
        #self.t2.start()

    def openPositionInfo(self):
        print("should be trying to open")
        # ProgramInfo.PositionInfo(self.program_label.curselection()[0], self.controller.reverse_start)
        # newProgThread = threading.Thread(target=ProgramInfo.PositionInfo, args=(self.program_label.curselection()[0], self.controller.reverse_start))
        monitorNewProgThread = threading.Thread(target=self.monitorPositionProg)
        monitorNewProgThread.start()
        print("post thread")
        # newProgThread = threading.Thread(target=ProgramInfo.PositionInfo(self.program_label.curselection()[0], self.controller.reverse_start))
        # newProgThread.start()
        ProgramInfo.PositionInfo(self.program_label.curselection()[0], self.controller.reverse_start)
        # time.sleep(1)
        # while not ProgramInfo.PositionInfo.isOpen:
        #     print("not open yet")
        self.monitorPositionProg()
        print("came here next")
        # print("is it open = ", ProgramInfo.isOpen)
        # while ProgramInfo.isOpen:
        #     print("still open")

    def monitorPositionProg(self):
        onlyPrint = True
        # time.sleep(0.25)
        # print("is it open = ", ProgramInfo.PositionInfo.isOpen)
        try:
            while ProgramInfo.PositionInfo.isOpen == 1:
                time.sleep(1)
                print("still open")
                print("saved? = ", ProgramInfo.PositionInfo.valueSaved)
                if ProgramInfo.PositionInfo.valueSaved == True:
                    self.updatePositionValues()
                    ProgramInfo.PositionInfo.valueSaved = False
        except:
            if onlyPrint:
                print("doesn't work")
                onlyPrint = False
            # self.monitorPositionProg()

    def updatePositionValues(self):
        print('updating all position values')


    def patternFunctions(self):
        recordAct = 1
        patternEvent = 4
        PatternBuilder.Pattern(self.controller.reverse_start)  #
        if PatternBuilder.Pattern.completePattern:
            self.save_position(recordAct, patternEvent)

    def go_home_save(self):
        print("going home?")
        self.controller.manual_programming_list_save_var = [0] * self.program_label.size()
        #self.local_manual_programming_list = StringVar(value=self.controller.manual_programming_list_save_var)
        for i in range(0 , self.program_label.size()):
            self.controller.manual_programming_list_save_var[i] = self.program_label.get(i)
        self.controller.show_frame_1("HomePage")

    def green_circle(self, decide):
        print("inside method")

        calX1Dot = 2
        calX2Dot = 2
        calY1Dot = 13
        calY2Dot = 13
        spacing = 12

        connected_label = Label(self, text="Connection Status: ")
        connected_label.grid(row=1, column=15)
        cameraCalLabel = Label(self, text="Camera Calibration Status: ")
        cameraCalLabel.grid(row=2, column=15)
        robotCalLabel = Label(self, text="Robot Calibration Status: ")
        robotCalLabel.grid(row=3, column=15)

        self.canvas = Canvas(self, width=12, height=(calY2Dot + (calY1Dot + spacing) * 2), borderwidth=0, bg="white")
        self.canvas.grid(row=1, column=20, columnspan=2, rowspan=3)

        self.connected_status_circle = self.canvas.create_oval(calX1Dot, calX2Dot, calY1Dot, calY2Dot, fill="red",
                                                               outline="black")
        self.cameraCalibrationStatus = self.canvas.create_oval(calX1Dot, (calX2Dot + calY1Dot + spacing), calY1Dot,
                                                               (calY2Dot + calY1Dot + spacing), fill="red",
                                                               outline="black")
        self.robotCalibrationStatus = self.canvas.create_oval(calX1Dot, (calX2Dot + (calY1Dot + spacing) * 2), calY1Dot,
                                                              (calY2Dot + (calY1Dot + spacing) * 2), fill="red",
                                                              outline="black")
        if self.controller.arduinoConnected is True:
            self.canvas.itemconfig(self.connected_status_circle, fill="green")
            # self.zeroMotorsButton.configure(state=NORMAL)
        if self.controller.programVariables['calibration']['calibrationStatus'] is True:
            self.canvas.itemconfig(self.cameraCalibrationStatus, fill="green")
        if self.controller.programVariables['calibration']['robotCalibration']['robotCalibrationStatus'] is True:
            self.canvas.itemconfig(self.robotCalibrationStatus, fill="green")

    def zeroMotorPositions(self):
        print("zeroing out motors")
        send = "ZeroMotors"
        back = "CalibrationComplete"
        sleep = 0.5
        timeout = 10
        zeroMotorResponse = self.communicate(send, back, sleep, timeout)
        while "cal" not in zeroMotorResponse:
            zeroMotorResponse = self.controller.arduino_data.readline().decode()
            print("zeroMotorResponse = ", zeroMotorResponse)
        storePotPositions = self.decodeZeroMotorString(zeroMotorResponse)
        for i in range(0,6):
            currentJoint = "J" + str(i+1)
            self.controller.programVariables['calibration']['jointPotZeroValues'][currentJoint] = storePotPositions[i]
        with open(self.controller.jsonFileName, "w") as file:
            json.dump(self.controller.programVariables, file, indent=4)

    def decodeZeroMotorString(self, zeroMotorString):
        loc = zeroMotorString.index('l')
        new_joint_pos = zeroMotorString[(loc + 1):len(zeroMotorString)]
        tracking_string = 'ABCDEF'
        # I am getting back joint positions here after a linear move
        # I need to take this joint positions and convert them to cartesian coordinates
        pos = 0
        j = 0
        start = 0
        potPositions = [0,0,0,0,0,0]
        for i in range(0, 6):
            if i == 0:
                start = 0
            else:
                letterIndexStart = tracking_string[i - 1]
                start = new_joint_pos.index(letterIndexStart) + 1
            letterIndexStop = tracking_string[i]

            stop = new_joint_pos.index(letterIndexStop)
            potPositions[i] = float(new_joint_pos[start:stop])

            print("potPositions [", i, "] = ", potPositions[i])
            start = pos + 1
            j += 1
        return potPositions

    def storeRobotCalibration(self):
        print("hello?")
        print(self.controller.programVariables['calibration']['robotCalibration'])
        self.controller.programVariables['calibration']['robotCalibration']['robotCalibrationStatus'] = True
        for i in range(len(self.controller.calLocations)):
            self.controller.programVariables['calibration']['robotCalibration'][self.controller.calLocations[i]] = self.controller.reverse_start[i]
            print(self.controller.programVariables['calibration']['robotCalibration'][self.controller.calLocations[i]])
        with open(self.controller.jsonFileName, "w") as file:
            json.dump(self.controller.programVariables, file, indent=4)
        self.green_circle(0)

    def linear_buttons(self, direction, r, c):
        self.lin_button = Button(self, text=direction, bg="white") #, command=lambda: self.movement(direction)
        self.lin_button.grid(row=r, column=c)
        self.lin_button.bind('<ButtonPress-1>', lambda event, direction=direction: self.start(direction))
        self.lin_button.bind('<ButtonRelease-1>', lambda event: self.stop1(direction))
        #self.lmove_thread = threading.Thread(target=self.start, args=(direction,))
        #self.lin_button.bind('<ButtonPress-1>', lambda event: self.lmove_thread.start())
        #self.lin_button.bind('<ButtonPress-1>', lambda event: threading.Thread(target=self.start, args=(direction,)).start())
        #self.lin_button.bind('<ButtonRelease-1>', lambda event: self.stop(direction))

    def capture_and_place(self, decider): #, capture_position, place_position
        self.capture_place = Toplevel()
        self.capture_place.geometry('400x200')
        self.capture_place.wm_title("Define Vision Parameters")
        print("Should be a capture press popup")
        vision_directions = "Vision Model requires two additional positions: \n " \
                            "First requirement is location to capture picture \n " \
                            "Second required position is location to place"
        self.directions_label = Label(self.capture_place, text=vision_directions, bg='white')
        self.directions_label.grid(row=0, column=0, columnspan=5, rowspan=5)

        self.vision_capture_position_label_title = Label(self.capture_place, text="Robot Capture Position:  ", bg='white')
        self.vision_place_position_label_title = Label(self.capture_place, text="Robot Place Position:  ", bg='white')
        self.vision_capture_position_label = Label(self.capture_place, text="  ", bg='white')
        self.vision_place_position_label = Label(self.capture_place, text="  ", bg='white')

        self.vision_capture_position_label_title.grid(row=10, column=0)
        self.vision_place_position_label_title.grid(row=12, column=0)
        self.vision_capture_position_label.grid(row=10, column=5)
        self.vision_place_position_label.grid(row=12, column=5)
        #capture_string = "Capture Position: " + str(capture_position)
        #place_string = "Place String: ", str(place_position)
        #self.capture_location_label = Label(self.capture_place, text=capture_string)
        #self.place_location_label = Label(self.capture_place, text=place_string)
        self.save_exit_button = Button(self.capture_place, text="Save and Exit", command=lambda: self.save_and_exit(1))
        self.save_exit_button.grid(row=16, column=2)
        #for clear selection can I somehow use the save and exit method similar to if i closed out of the window without saving?
        self.clear_selection_button = Button(self.capture_place, text="Clear Positions", command=lambda: self.save_and_exit(2)) #
        self.clear_selection_button.grid(row=16, column=3)

        if decider == 1:
            current_selection = self.program_label.curselection()
            print("current selection = ", current_selection[0])
            temp_index = self.controller.save_position_dictionary[current_selection[0]]
            capture_location_text = "x=" + str(self.controller.save_capture_position[temp_index][0]) + "  y=" + \
                                    str(self.controller.save_capture_position[temp_index][1]) + "   z=" + \
                                    str(self.controller.save_capture_position[temp_index][2])
            self.vision_capture_position_label.configure(text=capture_location_text)
            place_location_text = "x=" + str(self.controller.save_place_position[temp_index][0]) + "  y=" + \
                                    str(self.controller.save_place_position[temp_index][1]) + "   z=" + \
                                    str(self.controller.save_place_position[temp_index][2])
            self.vision_place_position_label.configure(text=place_location_text)
        #try:
        #    self.capture_place.protocol("WM_DELETE_WINDOW", self.save_and_exit(2))
        #except:
        #    pass

    def save_and_exit(self, save_action):
        #This function should close out of the popup window for capture and place positions
        #This will also save the capture and place positions
        #need to look up how to make this close a toplevel window
        if save_action == 1 and self.controller.times_through_vision_loop == 0:
            self.program_label.itemconfig(self.current_position_holder, fg='black', bg='white')

            self.controller.save_capture_position[self.controller.capture_position_counter-1] = self.temp_capture_position
            self.controller.capture_position_counter += 1
            self.controller.save_capture_position.resize((self.controller.capture_position_counter, 6), refcheck=False)

            self.controller.save_place_position[self.controller.place_position_counter - 1] = self.temp_place_position
            self.controller.place_position_counter += 1
            self.controller.save_place_position.resize((self.controller.place_position_counter, 6), refcheck=False)

            #self.controller.save_position_location[self.controller.capture_position_counter-1] = self.save_loc
            #self.controller.save_position_location.resize(self.controller.capture_position_counter, refcheck=False)

            self.controller.save_position_dictionary[self.save_loc] = self.controller.place_position_counter - 2

            self.save_loc += 1
            self.capture_place.destroy()

        if save_action == 2:
            self.controller.capture_position_counter -= 1
            self.controller.save_capture_position.resize((self.controller.capture_position_counter, 6), refcheck=False)
            self.controller.place_position_counter -= 1
            self.controller.save_place_position.resize((self.controller.place_position_counter, 6), refcheck=False)
            delete_name = self.controller.save_position_dictionary[self.controller.place_position_counter - 1]
            #del self.controller.save_position_dictionary[self.controller.place_position_counter - 2]
            del self.controller.save_position_dictionary[delete_name]
            self.vision_capture_position_label.configure(text="  ")
            self.vision_place_position_label.configure(text="  ")
            self.save_loc -= 1

        print("save capture positions")
        print(self.controller.save_capture_position)
        print("save place positions")
        print(self.controller.save_place_position)
        print("save position dictionary")
        print(self.controller.save_position_dictionary)
        print("save_loc = ", self.save_loc)
        #self.capture_place.destroy()

    def joint_buttons(self, joint, r, c):
        self.j_button = Button(self, text=joint, bg="white") #, command=lambda self.start(joint)
        self.j_button.grid(row=r, column=c)
        self.j_button.bind('<ButtonPress-1>', lambda event, direction=joint: self.start(joint))
        self.j_button.bind('<ButtonRelease-1>', lambda event, direction=joint: self.stop1(joint))

    def joint_position_readout(self, angle):
        self.joint_pos_label = [0,0,0,0,0,0]
        for i in range(0,6):
            print("angle[i] = ", angle[i])
            if angle[i] > 0:
                angleString = "+" + str(angle[i])
            elif angle[i] == 0:
                angleString = "0.00"
            else:
                angleString = angle[i]
            self.joint_pos_label[i] = Label(self, text=angleString, bg='white')
            self.joint_pos_label[i].grid(row=(i+4), column=2)

    def test_method(self):
        self.test_label = [0,0,0,0,0]
        self.test_label[0] = Label(self, text="Here I am!")
        self.test_label[0].grid(row=10, column=10)

    def save_position(self, action, event):
        self.green_circle(self.controller.con_decide)

        if self.controller.con_decide == 1:
            selection = self.program_label.curselection()
            if event == 2:
                print("times through vision loop = ", self.controller.times_through_vision_loop)
                if self.controller.times_through_vision_loop == 0:
                    self.program_text_xyz = "Vision Model: " + str(self.vision_options[self.vision_dropdown.current()])
                    self.program_label.insert(END, self.program_text_xyz)
                    self.program_label.itemconfig(self.save_loc, fg='black', bg='red')
                    self.capture_and_place(0)
                    print("vision model master")

                    vision_model_name = self.vision_options[self.vision_dropdown.current()]
                    vision_parameter_index = self.vision_options.index(str(vision_model_name))
                    print(self.controller.vision_model_master[vision_parameter_index])

                    self.controller.times_through_vision_loop += 1
                #self.save_loc += 1
            elif event == 3:
                if not selection:
                    popupmsg("Error", "Select a vision position to modify")
                else:
                    self.capture_and_place(1)
                    self.controller.times_through_vision_loop += 1
            elif event == 4:
                self.program_text_xyz = "Pattern Loop Start"

            elif event == None:
                #InverseKinematics.Kinematics(1, self.reverse)
                print("reverse input = ", self.controller.reverse_start)
                # wpr = [0.0, 0.0, 0.0]
                # for i in range(3,6):
                #     self.controller.reverse_start[i] = math.degrees(self.controller.reverse_start[i])
                # InverseKinematics.Kinematics(1, self.controller.reverse_start)
                # angles = InverseKinematics.Kinematics.joints
                # print("angles = ", angles)
                print("reverse input = ", self.controller.reverse_start)
                #self.program_text_xyz = "x=" + str(self.x_pos) + "  y=" + str(self.y_pos) + "   z=" + str(self.z_pos)
                self.program_text_xyz = "x=" + str(self.controller.reverse_start[0]) + "  y=" + \
                                    str(self.controller.reverse_start[1]) + "   z=" + str(self.controller.reverse_start[2])
                # self.program_text_wpr = "w=" + str(round(math.degrees(self.controller.reverse_start[3]), 2)) + "  p=" + \
                #                     str(round(math.degrees(self.controller.reverse_start[4]), 2)) + "   r=" + str(round(math.degrees(self.controller.reverse_start[5]), 2))
                self.program_text_wpr = "w=" + str(round(self.controller.reverse_start[3], 2)) + "  p=" + \
                                        str(round(self.controller.reverse_start[4], 2)) + "   r=" + \
                                        str(round(self.controller.reverse_start[5], 2))

                self.program_text_xyzwpr = self.program_text_xyz + "  " + self.program_text_wpr
                print("wpr positions = ", self.program_text_wpr)
                self.controller.save_program_positions[self.save_loc,:] = self.controller.reverse_start[:]
                print("new saved positions = ", self.controller.save_program_positions[self.save_loc])
                if action == 1:
                    if self.save_loc < 20 and self.controller.times_through_vision_loop == 0:
                        # self.program_label.insert(END, self.program_text_xyz)
                        self.program_label.insert(END, self.program_text_xyzwpr)
                        self.program_label.itemconfig(self.save_loc, fg=self.text_color)
                        self.save_loc += 1
                        self.controller.save_program_positions.resize(((self.save_loc+1), 6), refcheck=False)
                    elif self.controller.times_through_vision_loop == 1:
                        self.current_position_holder = self.save_loc
                        self.temp_capture_position = self.controller.reverse_start
                        #self.controller.save_capture_position[self.controller.capture_position_counter-1] = self.controller.reverse_start
                        #self.controller.capture_position_counter += 1
                        #self.controller.save_capture_position.resize((self.controller.capture_position_counter, 6), refcheck=False)
                        self.vision_capture_position_label.configure(text=self.program_text_xyz)
                        self.controller.times_through_vision_loop += 1
                        #print("capture positions:")
                        #print(self.controller.save_capture_position)
                    elif self.controller.times_through_vision_loop == 2:
                        self.temp_place_position = self.controller.reverse_start
                        #self.controller.save_place_position[self.controller.place_position_counter - 1] = self.controller.reverse_start
                        #self.controller.place_position_counter += 1
                        #self.controller.save_place_position.resize((self.controller.place_position_counter, 6),refcheck=False)
                        self.vision_place_position_label.configure(text=self.program_text_xyz)
                        self.controller.times_through_vision_loop = 0
                        #self.program_label.itemconfig(self.save_loc, fg='black', bg='white')
                        #print("place positions:")
                        #print(self.controller.save_place_position)
                    elif self.save_loc >= 20:
                        popupmsg("Too many positions", "Only 20 saved positions available")
                elif action == 2:
                    if self.save_loc < 20:
                        self.program_label.insert(selection[0]+1, self.program_text_xyz)
                        self.program_label.itemconfig(selection[0]+1, fg=self.text_color)
                        self.save_loc += 1
                    elif self.save_loc >= 20:
                        popupmsg("Too many positions", "Only 20 saved positions available")
                elif action == 3:
                    print("I am deleting")
                    print("selection = ", selection)
                    item_to_delete = self.program_label.get(selection)
                    print("item to delete = " , item_to_delete)
                    if "Vision" in item_to_delete:
                        print("deleting save position")
                        self.controller.capture_position_counter -= 1
                        self.controller.save_capture_position.resize((self.controller.capture_position_counter, 6),
                                                                     refcheck=False)
                        self.controller.place_position_counter -= 1
                        self.controller.save_place_position.resize((self.controller.place_position_counter, 6),
                                                                   refcheck=False)
                        delete_name = self.controller.save_position_dictionary[
                            self.controller.place_position_counter - 1]
                        # del self.controller.save_position_dictionary[self.controller.place_position_counter - 2]
                        del self.controller.save_position_dictionary[delete_name]
                        self.save_loc -= 1
                        print("save_loc = ", self.save_loc)
                    self.program_label.delete(selection[0])

            self.program_text_xyz = ""
            self.program_text_wpr = ""
            self.program_text_xyzwpr = ""
            #if event == 2 and self.controller.times_through_vision_loop == 0:
            #    self.controller.times_through_vision_loop += 1
            #    vision_message = "Vision Model requires two additional positions: \n First requirement is location to capture picture \n Second required position is location to place"
            #    popupmsg("Vision Model Requirements", vision_message)
        else:
            title = "No Connection"
            message = "No USB connection found, connect to arduino."
            popupmsg(title, message)

    def start(self, direction):
        print("start")
        self.start_press = 1
        if self.controller.con_decide == 1:
            self.change = 0
            #if direction[0] == "J":
                #send = "start" + str(direction[1]) + str(direction[2])
                #print(send)
                #self.communicate(send, "s!", 5)
                #print("am i hear?")

            self.movement1(direction)
            # using threading to do monitoring of current joint position
            self.t1 = threading.Thread(target=self.monitor, args=(direction,))
            #self.t4 = threading.Thread(target=self.simulation_update, args=(self.controller.all_pos,))
            #self.t3 = threading.Thread(target=self.simulate, args=(self.controller.all_pos,))
            #self.t2 = threading.Thread(target=self.movement1, args=(direction,))
            self.t1.start()
            #self.t4.start()
            #self.t3.start()
            #self.t2.start()
            #self.simulation_update(self.controller.all_pos)

            print("post thread start")
            #self.movement1(direction)
            #self.job = self.after(50, self.start, direction)
        else:
            title = "No Connection"
            message = "No USB connection found, connect to arduino."
            popupmsg(title, message)

    def decodeTrackingString(self, trackingString):
        loc = trackingString.index('g')
        new_joint_pos = trackingString[(loc + 1):len(trackingString)]
        print("new joint position = ", new_joint_pos)

        tracking_string = 'ABCDEF'
        # I am getting back joint positions here after a linear move
        # I need to take this joint positions and convert them to cartesian coordinates
        pos = 0
        j = 0
        start = 0
        for i in range(0, 6):
            if i == 0:
                start = 0
            else:
                letterIndexStart = tracking_string[i - 1]
                start = new_joint_pos.index(letterIndexStart) + 1
            letterIndexStop = tracking_string[i]

            stop = new_joint_pos.index(letterIndexStop)
            self.controller.angles[i] = float(new_joint_pos[start:stop])

            print("angle [", i, "] = ", self.controller.angles[i])
            start = pos + 1
            j += 1
        self.joint_position_readout(self.controller.angles)
            # self.joint_pos_label[i].configure(text=str(self.controller.angles[i]))
        # self.updateKinematics()

    def decodeJointReportString(self, jointReportString):
        j_avail = 0
        for i in range(len(jointReportString)):
            if jointReportString[i] == 'J':
                j_avail = j_avail + 1
        j_array = [0] * j_avail
        j_locs = [0] * j_avail
        j_vals = [0] * j_avail
        j = 0
        # get joint number being reported
        for i in range(len(jointReportString)):
            if jointReportString[i] == 'J':
                j_array[j] = int(jointReportString[i + 1])
                j_locs[j] = i
                j = j + 1
        # print("j_locs = ", j_locs)
        # print("j_array = ", j_array)
        # print("respond = ", respond)

        # get actual joint positions being reported
        for i in range(0, j_avail):
            if i == j_avail - 1:
                # j_vals[i] = float(respond[(j_locs[i] + 5):len(respond)])
                # print("i = ", i)
                # print("j_array[i]-1 = ", j_array[i]-1)
                # print("(j_locs[i]+5) = ", (j_locs[i]+5))
                self.controller.angles[j_array[i] - 1] = float(jointReportString[(j_locs[i] + 5):len(jointReportString)])
            else:
                # j_vals[i] = float(respond[j_locs[i]+5:j_locs[i+1]-1])
                # print("(j_locs[i]+5) = ", (j_locs[i]+5))
                # print("(j_locs[i+1]-1) = ", (j_locs[i+1]-1))
                self.controller.angles[j_array[i] - 1] = float(jointReportString[(j_locs[i] + 5):(j_locs[i + 1] - 1)])
            if self.controller.angles[j_array[i]-1] > 0:
                jointPosString = "+" + str(self.controller.angles[j_array[i] - 1])
            elif self.controller.angles[j_array[i]-1] == 0:
                jointPosString = "0.00"
            else:
                jointPosString = str(self.controller.angles[j_array[i] - 1])
            self.joint_pos_label[j_array[i] - 1].configure(text=jointPosString)
            # self.joint_pos_label[j_array[i] - 1].configure(text=str(self.controller.angles[j_array[i] - 1]))
        # self.joint_position_readout(self.controller.angles)

    def updateKinematics(self):
        # print("r_pos before = ", self.controller.reverse_start[5])
        InverseKinematics.Kinematics(2, self.controller.angles)
        # print("r_pos after = ", self.controller.reverse_start[5])
        self.controller.reverse_start = InverseKinematics.Kinematics.position
        self.controller.all_pos = InverseKinematics.Kinematics.all_positions
        self.x_pos = self.controller.reverse_start[0]
        self.y_pos = self.controller.reverse_start[1]
        self.z_pos = self.controller.reverse_start[2]
        self.w_pos = self.controller.reverse_start[3]
        self.p_pos = self.controller.reverse_start[4]
        self.r_pos = self.controller.reverse_start[5]
        self.program_text_xyz = "x=" + str(self.x_pos) + "  y=" + str(self.y_pos) + "   z=" + str(self.z_pos)
        self.program_text_wpr = "w=" + str(self.w_pos) + "  p=" + str(self.p_pos) + "   r=" + str(self.r_pos)
        self.current_loc.config(text=self.program_text_xyz)
        self.current_loc_wpr.configure(text=self.program_text_wpr)
        print("current location = ", self.controller.reverse_start)
        print("current wpr = ", self.program_text_wpr)

    def lastGoodResponse(self, oldMessage, newMessage):
        if newMessage.len() > 0:
            return newMessage
        else:
            return oldMessage

    def monitor(self, argument):
        timesThroughWhileLoop = 0
        posDelta = [-1, -1, -1, -1, -1, -1]
        if type(argument) is str:
            if 'Running' in argument and 'stopping' in argument:
                respond = argument
            else:
                respond = self.controller.arduino_data.readline().decode()
        else:
            respond = self.controller.arduino_data.readline().decode()
        # print("am i in the monitor function?")
        break_loop = False
        while True:
            cnt_loop_break = 0
            print("respond at start of loop = ", respond)
            timesThroughWhileLoop += 1
            while "cnt" not in respond and "stopping" not in respond:
                respond = self.controller.arduino_data.readline().decode()
                print('cnt loop respond = ', respond)
                # print('self.out = ', self.out)
                cnt_loop_break += 1
                if cnt_loop_break == 10 or "stopping" in respond:
                    print('monitor continue break loop')
                    break
                # print('cnt loop break = ', cnt_loop_break)
            # print("I have broken out of monitor continue")
            loc = respond.find("cnt")
            loc2 = respond[(loc+3):len(respond)].find("cnt")
            # variable updated that breaks out of monitor loop
            if self.out == 1:
                print("stopping because of self.out")
                # monitor sets self.out to 1, now inside this loop I will send stop message
                send = "stop"
                self.controller.arduino_data.write(send.encode())
                respond = self.controller.arduino_data.readline().decode()
                while "stopping" not in respond:
                    print("self.out = 1, respond = ", respond)
                    respond = self.controller.arduino_data.readline().decode()
                print("respond = ", respond)
                # get joint values from return string
                self.decodeTrackingString(respond)
                self.updateKinematics()
                break
            else:
                # print("am i in else?")
                # get single point position from return string
                self.decodeJointReportString(respond)
                self.updateKinematics()
                # if len(argument) == 6:
                #     posDelta, atEndPos = self.compareCartesianCoordinates(argument, posDelta)
            print("respond after kinematics update = ", respond)
            if "stopping" not in respond:
                respond = self.controller.arduino_data.readline().decode()
            print("respond in monitor loop = ", respond)
            if "stopping" in respond:
                break
            # print("end of loop")
            #respond = self.controller.arduino_data.readline().decode()
        self.out = 0
        # for i in range(3,6):
        #     self.controller.reverse_start[i] = math.radians(self.controller.reverse_start[i])
        print('break')
        print("times through while loop = ", timesThroughWhileLoop           )
        #return new_joint_pos

    def simulate(self, spots):
        x_spot = np.zeros([7])
        y_spot = np.zeros([7])
        z_spot = np.zeros([7])

        holder = spots[0]
        # print('holder = ', holder[6][2])
        # print('len(x_spots[0]) = ', len(x_spot))
        for i in range(len(spots)):
            x_spot[i] = spots[i][0]
            y_spot[i] = spots[i][1]
            z_spot[i] = spots[i][2]
            # x_spot[i] = spots[0][i][0]
            # y_spot[i] = spots[0][i][1]
            # z_spot[i] = spots[0][i][2]
        print('z_spot = ', z_spot)
        #self.fig = plt.figure()
        #self.ax = self.fig.add_subplot(111, projection='3d')
        sc = self.ax.scatter(x_spot, y_spot, z_spot, c='r', marker='o')
        for i in range(1, len(x_spot)):
            pl = self.ax.plot([x_spot[i - 1], x_spot[i]], [y_spot[i - 1], y_spot[i]], [z_spot[i - 1], z_spot[i]], c='r')
        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')
        self.ax.set_zlabel('Z Label')
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(-50, 50)
        self.ax.set_zlim(-5, 40)

        #anim = FuncAnimation(fig, self.update, repeat=False, fargs=(ax, spots, sc, pl), frames=np.arange(0, len(spots)),
        #                     interval=50, blit=False)

        #plt.show()
        if self.window_open == 0:
            x = self.winfo_x()
            y = self.winfo_y()
            self.window_open = 1
            self.sim_window = Toplevel()
            self.sim_window.geometry("+%d+%d" % (x + 100, y + 200))
            print('hello')
            self.sim_window.wm_title('Simulated View')
            canvas = FigureCanvasTkAgg(self.fig, self.sim_window)
            #canvas.show()
            canvas.get_tk_widget().pack(side=BOTTOM, fill=BOTH, expand=True)

        #if self.update_ready == 1:
        #    print("test, am I here?")
        #    FuncAnimation(fig, self.update, repeat=False, fargs=(ax, spots, sc, pl),
        #                  frames=np.arange(0, len(spots)),interval=50, blit=False)
        #    self.update_ready = 0

        try:
            self.sim_window.protocol("WM_DELETE_WINDOW", self.on_close)
        except:
            pass

    def on_close(self):
        self.window_open = 0
        self.sim_window.destroy()

    def simulation_update(self, spots):
        x_spot = np.zeros([7])
        y_spot = np.zeros([7])
        z_spot = np.zeros([7])

        holder = spots[0]
        # print('holder = ', holder[6][2])
        # print('len(x_spots[0]) = ', len(x_spot))
        for i in range(len(spots)):
            x_spot[i] = spots[i][0]
            y_spot[i] = spots[i][1]
            z_spot[i] = spots[i][2]

        while self.out == 0:
            print("simulation update")
            for i in range(1, len(x_spot)):
                pl = self.ax.plot([x_spot[i - 1], x_spot[i]], [y_spot[i - 1], y_spot[i]], [z_spot[i - 1], z_spot[i]], c='r')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        self.out = 1

    def stop1(self, direction_pos):
        print("stop1")
        self.out = 1
        #self.t2.join()

        # Doing this inside monitor instead
        # if self.controller.con_decide == 1:
        #     print("beginning of stop")
        #     send = "stop"
        #     self.controller.arduino_data.write(send.encode())
        #     self.communicate("stop", "stopping", 0.5, 5)
        # print('before join')
        # self.t1.join()

    def movement1(self, direction_move):
        print('direction_move = ', direction_move)
        if direction_move[0] == 'J':
            send = 'JMove' + direction_move
            print('spend = ', send)
            self.communicate(send, "Spinning", 0.3, 5)
        elif direction_move[0] == 'x' or direction_move[0] == 'y' or direction_move[0] == 'z':
            send = 'LMove' + direction_move
            print("send = ", send)
            self.communicate(send, "Spinning", 0.3, 5)

    def compareJointPos(self, endJoints):
        currentJointPos = [self.controller.angles[0], self.controller.angles[1], self.controller.angles[2],
                           self.controller.angles[3], self.controller.angles[4], self.controller.angles[5]]
        # endXYZWPR = [endXYZ[0], endXYZ[1], endXYZ[2], self.controller.reverse_start[3], self.controller.reverse_start[4],
        #              self.controller.reverse_start[5]]
        # InverseKinematics.Kinematics(1, endXYZWPR)
        # endJointPos = InverseKinematics.Kinematics.joints
        if (endJoints == currentJointPos):
            inPos = True
        else:
            inPos = False
        return inPos

    def compareCartesianCoordinates(self, endPos, lastPosDiff):
        currentCartesian = [self.controller.reverse_start[0], self.controller.reverse_start[1], self.controller.reverse_start[2],
                            self.controller.reverse_start[3], self.controller.reverse_start[4], self.controller.reverse_start[5]]
        posDifference = abs(np.array(currentCartesian) - np.array(endPos))
        # posDifference = abs(np.array(currentCartesian[:3]) - np.array(endPos))
        changeInDiff = np.greater_equal(posDifference, lastPosDiff)
        print("currentCartesian = ", currentCartesian)
        print("posDifference = ", posDifference)
        print("changeInDiff = ", changeInDiff)
        if lastPosDiff[0] == -1:
            inPos = False
        else:
            if all(i for i in changeInDiff) is True:
                inPos = False
            else:
                inPos = True
        return posDifference, inPos

    def run_program(self):
        if self.controller.con_decide == 1:
            program_proceed = False
            mover = "moveL"
            elements = self.program_label.size()
            programIndex = 0
            incriment = True
            run_thread = threading.currentThread()
            vision_pos_count = 0
            in_capture_pos = False
            picks_available = False
            atEndPos = False
            posDelta = [-1, -1, -1, -1, -1, -1]
            while programIndex < self.program_label.size() and self.kill == 0:
                print("--------------------------------------------------------------------")
                print("START OF WHILE LOOP IN RUN PROGRAM")
                print("--------------------------------------------------------------------")
                #self.a.arduino_access(self.program_label.get(i))
                position_list = str(self.program_label.get(programIndex))
                # position based movements
                if position_list[0] == "x":
                    # decode string to get next xyz positions
                    for j in range(0,len(position_list)):
                        if position_list[j] == 'y':
                            y_start = j+2
                        elif position_list[j] == 'z':
                            z_start = j+2
                        elif position_list[j] == 'w':
                            w_start = j+2
                        elif position_list[j] == 'p':
                            p_start = j+2
                        elif position_list[j] == 'r':
                            r_start = j+2
                    next_x_pos = float(position_list[2:(y_start-4)])
                    next_y_pos = float(position_list[y_start:(z_start-4)])
                    next_z_pos = float(position_list[z_start:(w_start-4)])
                    next_w_pos = float(position_list[w_start:(p_start-4)])
                    next_p_pos = float(position_list[p_start:(r_start-4)])
                    next_r_pos = float(position_list[r_start:])
                    endPosXYZ = [next_x_pos, next_y_pos, next_z_pos]
                    # endPosXYZWPR = [endPosXYZ[0], endPosXYZ[1], endPosXYZ[2], self.controller.reverse_start[3],
                    #              self.controller.reverse_start[4],self.controller.reverse_start[5]]
                    # endPosXYZWPR = self.controller.save_program_positions[programIndex, :]
                    endPosXYZWPR = np.zeros(6)
                    for i in range(6):
                        endPosXYZWPR[i] = round(self.controller.save_program_positions[programIndex][i], 4)
                        # if i>= 3:
                        #     endPosXYZWPR[i] = round(math.degrees(self.controller.save_program_positions[programIndex][i]), 4)
                        # else:
                        #     endPosXYZWPR[i] = round(self.controller.save_program_positions[programIndex][i], 4)
                    print("endPosXYZWPR = ", endPosXYZWPR)
                    # convert xyzwpr to joint positions
                    InverseKinematics.Kinematics(1, endPosXYZWPR)
                    endJointPos = InverseKinematics.Kinematics.joints
                    # for i in range(3,6):
                    #     endPosXYZWPR[i] = round(math.degrees(self.controller.save_program_positions[programIndex][i]),4)
                    currentXYZ = [self.controller.reverse_start[0], self.controller.reverse_start[1],
                                  self.controller.reverse_start[2]]
                    currentXYZWPR = [self.controller.reverse_start[0], self.controller.reverse_start[1],
                                  self.controller.reverse_start[2], self.controller.reverse_start[3],
                                  self.controller.reverse_start[4],self.controller.reverse_start[5]]
                    print("endPosXYZWPR = ", endPosXYZWPR)
                    print("next_x_pos = ", next_x_pos)
                    print("next_y_pos = ", next_y_pos)
                    print("next_z_pos = ", next_z_pos)
                    print("endJointPos = ", endJointPos)
                    incriment = True
                #Vision based movements
                if position_list[0] == "V":
                    #Vision based moves
                    visionName = position_list[14:]
                    vision_parameter_index = self.vision_options.index(str(visionName))
                    #next position should be vision capture position
                    # self.controller.save_capture_position[vision_pos_count]

                # check if robot is already at starting position, if not move to starting position
                # once at starting position, give message for user to confirm moving forward

                sending = self.make_run_string(endPosXYZWPR, position_list)
                self.communicate(sending, "Running", 0.3, 5)
                print("In run between communicate and monitor thread")
                # self.monitorThread = threading.Thread(target=self.monitor, args=(endPosXYZWPR,))
                # self.monitorThread.start()
                self.monitor(endPosXYZWPR)
                if programIndex == 0 and program_proceed == False:
                    self.program_label.selection_set(i)
                    self.continue_popup("Press Continue to Start Program", "Proceed")
                    program_proceed = True
                programIndex += 1

    def run_program_removed(self):
        capture = cv2.VideoCapture(0)
        _, manual_frame = capture.read()
        manual_frame = cv2.flip(manual_frame, 1)
        save_filename = "savedimage.jpg"
        cv2.imwrite(save_filename, manual_frame)
        loaded_img = Image.open(save_filename)

        if self.controller.con_decide == 1:
            program_proceed = 0
            mover = "moveL"
            elements = self.program_label.size()
            #self.communicate("Running", "Start", 0.3, 5)
            #self.program_label.selection_set(2)
            print('elements = ', elements)
            i=0
            incriment = True
            run_thread = threading.currentThread()
            #for i in range(elements):
            vision_pos_count = 0
            in_capture_pos = False
            picks_available = False
            while i < self.program_label.size() and self.kill == 0:
                #self.a.arduino_access(self.program_label.get(i))
                position_list = str(self.program_label.get(i))

                print('position list = ', position_list)
                print('i = ', i)
                #Moving to x,y,z position based on coordinates
                if position_list[0] == "x":
                    for j in range(0,len(position_list)):
                        if position_list[j] == 'y':
                            y_start = j+2
                        if position_list[j] == 'z':
                            z_start = j+2
                    next_x_pos = float(position_list[2:(y_start-4)])
                    next_y_pos = float(position_list[y_start:(z_start-4)])
                    next_z_pos = float(position_list[z_start:])
                    print("next_x_pos = ", next_x_pos)
                    print("next_y_pos = ", next_y_pos)
                    print("next_z_pos = ", next_z_pos)
                    incriment = True
                #moving to a vision based location
                elif position_list[0] == "V":
                    # Get the vision model info
                    #vision_page = self.controller.get_frame('VisionPage')
                    vision_model_name = position_list[14:]
                    #selected_vision_model = vision_page.vision_model_list

                    vision_parameter_index = self.vision_options.index(str(vision_model_name))
                    print("vision_parameter_index = ", vision_parameter_index)

                    #go to capture position
                    if in_capture_pos is True:
                        print("in capture position")
                        empty_arg = None
                        VisionPage.capture_press(self, empty_arg, manual_frame)
                        #vision_page.capture_press(empty_arg)
                        area_min_max = [self.controller.vision_model_master[vision_parameter_index][1],
                                        self.controller.vision_model_master[vision_parameter_index][0]]

                        if self.controller.vision_model_master[vision_parameter_index][2] == 1:
                            black_white_search = True
                        elif self.controller.vision_model_master[vision_parameter_index][2] == 2:
                            black_white_search = False
                        index = 0
                        rgb_lower_vals = np.array([0, 0, 0])
                        rgb_upper_vals = np.array([0, 0, 0])
                        for k in range(6):
                            if (k+1) % 2 == 1:
                                rgb_upper_vals[index] = (self.controller.vision_model_master[vision_parameter_index][k + 3])
                            if (k+1) % 2 == 0:
                                rgb_lower_vals[index] = (self.controller.vision_model_master[vision_parameter_index][k + 3])
                                index += 1

                        print("rgb lower values")
                        print(rgb_lower_vals)
                        print("rgb upper values")
                        print(rgb_upper_vals)
                        center_x, center_y = VisionPage.search_for_items(self, rgb_upper_vals, rgb_lower_vals,
                                                                         area_min_max, black_white_search, False,
                                                                         save_filename)
                        print("center_x = ", center_x)
                        print("center_y = ", center_y)
                        check_zero_x = np.all((center_x == 0))
                        check_zero_y = np.all((center_y == 0))
                        print("check_zero_x = ", check_zero_x)
                        print("check_zero_y = ", check_zero_y)
                        if check_zero_x == False and check_zero_y == False:
                            print("found pick positions")
                            next_x_pos = center_x[0]
                            next_y_pos = center_y[0]
                            next_z_pos = self.controller.save_capture_position[vision_pos_count][2]
                            picks_available = True
                        else:
                            print("no pick positions")
                            picks_available = False
                            incriment = True
                            next_x_pos = self.controller.save_place_position[vision_pos_count][0]
                            next_y_pos = self.controller.save_place_position[vision_pos_count][1]
                            next_z_pos = self.controller.save_place_position[vision_pos_count][2]
                        #vision_page.search_for_items(rgb_upper_vals, rgb_lower_vals, area_min_max, False)

                    #go to place position
                    elif in_capture_pos is False:
                        print("define capture positions as next positions")
                        next_x_pos = self.controller.save_capture_position[vision_pos_count][0]
                        next_y_pos = self.controller.save_capture_position[vision_pos_count][1]
                        next_z_pos = self.controller.save_capture_position[vision_pos_count][2]
                        incriment = False

                # check if robot is already at starting position, if not move to starting position
                # once at starting position, give message for user to confirm moving forward
                if i == 0 and program_proceed == 0:
                    if self.controller.reverse_start[0] != next_x_pos or self.controller.reverse_start[1] != next_y_pos or self.controller.reverse_start[2] != next_z_pos:
                        #do something
                        sending = self.make_run_string(next_x_pos, next_y_pos, next_z_pos, position_list)

                        # x_vector = self.controller.reverse_start[0] - next_x_pos
                        # y_vector = self.controller.reverse_start[1] - next_y_pos
                        # z_vector = self.controller.reverse_start[2] - next_z_pos
                        # distance = math.sqrt(math.pow(x_vector,2) + math.pow(y_vector,2) + math.pow(z_vector,2))
                        # #move_time = self.velocity / distance
                        # move_time = distance / self.velocity
                        # x_velocity = round((x_vector/move_time), 4)
                        # y_velocity = round((y_vector/move_time), 4)
                        # z_velocity = round((z_vector/move_time), 4)
                        # sending = "RunX" + str(x_velocity) + "Y" + str(y_velocity) + "Z" + str(z_velocity) + "V" + str(self.velocity) + \
                        #           "EndX" + str(next_x_pos) + "Y" + str(next_y_pos) + "Z" + str(next_z_pos)
                        print(sending)
                        #MISSING ACTUAL COMMUNICATION WITH ARDUINO TO MOVE TO STARTING POSITION

                        ### Without monitoring for the current position the below is used
                        moving = self.communicate(sending, "Running", 0.5, 10)

                        #self.controller.arduino_data.write(sending.encode())
                        #self.run_monitor = threading.Thread(target=self.monitor, args=(self.controller.con_decide,))
                        self.monitor(self.controller.con_decide)
                        #moving = self.communicate(sending, "Reached", 0.5, 10)

                    # load popup message stating robot is at starting position, closing starts the program
                    self.program_label.selection_set(i)
                    self.continue_popup("Press Continue to Start Program", "Proceed")
                    print('program proceed set to 1')
                    program_proceed = 1

                #Go here after robot is confirmed to be at starting position
                #Past starting position, move to next position
                else:
                    print('past starting position')

                    # I AM TAKING THIS OUT BECAUSE I THINK IT IS REDUDENT, HOPEFULLY NOTHING BREAKS
                    # if moving.startswith('stopping'):
                    #     tracking_string = 'ABCDEF'
                    #     # I am getting back joint positions here after a linear move
                    #     # I need to take this joint positions and convert them to cartesian coordinates
                    #     pos = 0
                    #     j = 7
                    #     start = 7
                    #     for k in range(0, 6):
                    #         while moving[j] != tracking_string[k]:
                    #             # print("moves[j] = ", moves[j])
                    #             # print("tracking_string[i] = ", tracking_string[i])
                    #             pos = j
                    #             j += 1
                    #         self.controller.angles[k] = float(moving[start + 1:pos + 1])
                    #         # print("moves[start:pos] = ", moves[start+1:pos+1])
                    #         start = pos + 1


                    InverseKinematics.Kinematics(2, self.controller.angles)
                    self.controller.reverse_start = InverseKinematics.Kinematics.position
                    self.controller.all_pos = InverseKinematics.Kinematics.all_positions
                    sending = self.make_run_string(next_x_pos, next_y_pos, next_z_pos, position_list)
                    # x_vector = self.controller.reverse_start[0] - next_x_pos
                    # y_vector = self.controller.reverse_start[1] - next_y_pos
                    # z_vector = self.controller.reverse_start[2] - next_z_pos
                    # distance = math.sqrt(math.pow(x_vector, 2) + math.pow(y_vector, 2) + math.pow(z_vector, 2))
                    # # move_time = self.velocity / distance
                    # move_time = distance / self.velocity
                    # x_velocity = round((x_vector / move_time), 4)
                    # y_velocity = round((y_vector / move_time), 4)
                    # z_velocity = round((z_vector / move_time), 4)
                    # sending = "RunX" + str(x_velocity) + "Y" + str(y_velocity) + "Z" + str(z_velocity) + "V" + str(self.velocity) + \
                    #           "EndX" + str(next_x_pos) + "Y" + str(next_y_pos) + "Z" + str(next_z_pos)

                    print('sending = ', sending)
                    receiving = "Reached"
                    #self.communicate(sending, receiving, 0.3, 3)

                    y_index = sending.find("Y")
                    z_index = sending.find("Z")
                    v_index = sending.find("V")
                    velocities = np.array([0,0,0])
                    velocities[0] = sending[4:y_index]
                    velocities[1] = sending[y_index+1:z_index]
                    velocities[2] = sending[z_index+1:v_index]

                    if np.all((velocities != 0)):
                        moving = self.communicate(sending, "Running", 0.5, 10)

                        # self.run_monitor = threading.Thread(target=self.monitor, args=(self.controller.con_decide,))
                        if 'Running' in moving and 'stopping' in moving:
                            self.monitor(moving)
                        else:
                            self.monitor(self.controller.con_decide)

                    print('did i exit?')
                    if i > 0:
                        self.program_label.selection_clear(i-1)
                        print('clear ', i)
                    elif i == 0:
                        self.program_label.selection_clear(self.program_label.size()-1)
                    self.program_label.selection_set(i)
                    #time.sleep(2)
                print("i = ", i)
                print("self.playback_loop = ", self.playback_loop.get())
                #incriment to next position in playback
                if incriment is True:
                    i += 1
                elif incriment is False:
                    in_capture_pos = True
                    print("take picture!")
                # Reset when at the end of the program playback if looping is selected
                if i == self.program_label.size() and self.playback_loop.get() == 1:
                    #self.program_label.selection_clear(i-1)

                    position_list = str(self.program_label.get(i-1))
                    print('current program label during reset = ', position_list)
                    i = 0
                    print('RESET!!!!!!!!!!!!!!!!!!!!!!!')

            self.kill = 0
            #Do I need this "done" communication?
            #self.communicate("Done", "Done", 0.3, 3)
        else:
            title = "No Connection"
            message = "No USB connection found, connect to arduino."
            popupmsg(title, message)

    def change_variable(self):
        print("KILL BUTTON PRESSED!!!!")
        self.kill = 1

    def make_run_string_xyz(self, next_x, next_y, next_z, list_position):
        #list_position = str(self.program_label.get(i))
        print(list_position)

        x_vector = self.controller.reverse_start[0] - next_x
        y_vector = self.controller.reverse_start[1] - next_y
        z_vector = self.controller.reverse_start[2] - next_z
        distance = math.sqrt(math.pow(x_vector, 2) + math.pow(y_vector, 2) + math.pow(z_vector, 2))
        if distance != 0:
            # move_time = self.velocity / distance
            move_time = distance / self.velocity
            x_velocity = round((x_vector / move_time), 4)
            y_velocity = round((y_vector / move_time), 4)
            z_velocity = round((z_vector / move_time), 4)
        else:
            x_velocity = 0
            y_velocity = 0
            z_velocity = 0

        # sending_message = "RunX" + str(x_velocity) + "Y" + str(y_velocity) + "Z" + str(z_velocity) + "V" + str(self.velocity) + \
        #           "EndX" + str(next_x) + "Y" + str(next_y) + "Z" + str(next_z)
        sending_message = "RunX" + str(x_velocity) + "Y" + str(y_velocity) + "Z" + str(z_velocity) + "V" + str(
            self.velocity)

        sending_message = "RunX" + str(x_velocity) + "Y" + str(y_velocity) + "Z" + str(z_velocity) + \
                          "W" + str(w_velocity) + "P" + str(p_velocity) + "R" + str(r_velocity) + \
                          "V" + str(self.velocity)


        return sending_message

    def make_run_string(self, endXYZWPR, list_position):
        # list_position = str(self.program_label.get(i))
        print(list_position)
        vector = np.zeros(6)
        velocity = np.zeros(6)
        print("current pos = ", self.controller.reverse_start)
        print("end pos = ", endXYZWPR)
        for i in range(0,6):
            vector[i] = self.controller.reverse_start[i] - endXYZWPR[i]
            # if i < 3:
            #     vector[i] = self.controller.reverse_start[i] - endXYZWPR[i]
            # elif i >= 3:
            #     vector[i] = self.controller.reverse_start[i] - math.radians(endXYZWPR[i])
        print("vector = ", vector)
        distance = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2) + math.pow(vector[2], 2))
        print("distance = ", distance)
        if distance != 0:
            # move_time = self.velocity / distance
            move_time = distance / self.velocity
            print("move time = ", move_time)
            for i in range(0, 6):
                if i >= 3:
                    velocity[i] = -1 * round((vector[i] / move_time), 4)
                else:
                    velocity[i] = round((vector[i] / move_time), 4)
        elif distance == 0 and (vector[3] != 0 or vector[4] != 0 or vector[5] != 0):
            maxVloc = np.where(vector == np.amax(abs(vector)))
            maxV = np.max(abs(vector))
            print("maxV = ", maxV)
            # move_time = vector[maxVloc] / self.velocity
            move_time = maxV / self.velocity
            print("move time = ", move_time)
            for i in range(0, 6):
                if i <= 3:
                    velocity[i] = 0
                elif i > 3:
                    velocity[i] = -1 * round((vector[i] / move_time), 4)

        # sending_message = "RunX" + str(x_velocity) + "Y" + str(y_velocity) + "Z" + str(z_velocity) + "V" + str(self.velocity) + \
        #           "EndX" + str(next_x) + "Y" + str(next_y) + "Z" + str(next_z)
        # sending_message = "RunX" + str(x_velocity) + "Y" + str(y_velocity) + "Z" + str(z_velocity) + "V" + str(
        #     self.velocity)

        sending_message = "RunX" + str(velocity[0]) + "Y" + str(velocity[1]) + "Z" + str(velocity[2]) + \
                          "W" + str(velocity[3]) + "P" + str(velocity[4]) + "R" + str(velocity[5]) + \
                          "EndX" + str(endXYZWPR[0]) + "Y" + str(endXYZWPR[1]) + "Z" + str(endXYZWPR[2]) + \
                          "W" + str(endXYZWPR[3]) + "P" + str(endXYZWPR[4]) + "R" + str(endXYZWPR[5])
        print("sending message = ", sending_message)
        return sending_message

    def continue_popup(self, message, title):
        var = IntVar()
        cnt_pop = Toplevel()
        cnt_pop.geometry('200x100')
        cnt_pop.wm_title(title)
        label = Label(cnt_pop, text=message)
        label.grid(row=1, column=1, columnspan=2)
        cnt_button = Button(cnt_pop, text="Continue", command=lambda: var.set(1))
        cnt_button.grid(row=2, column=2, columnspan=2)
        cnt_button.wait_variable(var)
        cnt_pop.destroy()
        cnt_pop.update()

    def move_type(self, move_x):
        if move_x == 1:
            #self.program_label.configure(fg="green")
            #self.program_label.itemconfig(self.save_loc, bg="green")
            self.moveJ_button.configure(bg="green")
            self.moveL_button.configure(bg="grey")
            self.moveP_button.configure(bg="grey")
            self.text_color = "green"
        elif move_x == 2:
            #self.program_label.configure(fg="blue")
            #self.program_label.itemconfig(self.save_loc, bg="blue")
            self.moveL_button.configure(bg="blue")
            self.moveJ_button.configure(bg="grey")
            self.moveP_button.configure(bg="grey")
            self.text_color = "blue"
        elif move_x == 3:
            #self.program_label.configure(fg="red")
            #self.program_label.itemconfig(self.save_loc, bg="red")
            self.moveP_button.configure(bg="red")
            self.moveJ_button.configure(bg="grey")
            self.moveL_button.configure(bg="grey")
            self.text_color = "red"

    def communicate(self, send, back, sleep, timeout): #send=message to send; back=what I am looking to receive back; sleep=delay that I should standardize use 0.2 or 0.3; timeout=time to kill program if I don't hear back from arduino
        counter = 0
        response = ""
        print("communicate send = ", send)
        self.controller.arduino_data.write(send.encode()) #send message to arduino over USB
        time.sleep(sleep) #delay to let transfer happen and arduino decide how to respond
        print('just before response updated')
        response = self.controller.arduino_data.readline().decode() #look for message back from arduino over USB
        print('after response in communicate')
        kill = 0
        print("communicate respond = ", response) #what I actually get back from arduino
        #print("back = ", back) #what I am looking to recieve from arduino
        #time.sleep(0.05)
        # while back not in response:
        # #while respond not in back: #redudent check to find message from arduino if not found the first time
        #     time.sleep(sleep)
        #     print("before")
        #     response = self.controller.arduino_data.readline().decode() #getting message from arduino
        #     print("after")
        #     kill += 1
        #     print('kill = ', kill)
        #     if 'cnt' in response:
        #         kill = 0
        #     print("loop respond = ", response)
        #     if timeout == 1:
        #         break
        #     elif kill == timeout: #break from loop and send error message if cannot find message from arduino
        #         title = "timeout"
        #         message = "No Response, try to reconnect."
        #         popupmsg(title, message)
        print('breaking!!')
        print("respond message to return from communicate = ", response)
        return response


class VisionPage(Frame):

    def __init__(self, parent, controller, attr=None):
        Frame.__init__(self, parent)
        self.controller = controller
        self.runningCalibration = False
        home_button = Button(self, text="Home Screen", bg="grey", border=0, command=lambda: self.close_save())
                             #command=lambda: controller.show_frame_1("HomePage"))

        self.keep_going = 0
        #home_button.grid(row=10, column=40, padx=10, pady=2, rowspan=30, columnspan=20)
        home_button.grid(row=95, column=40, padx=10, pady=1, rowspan=1, columnspan=1)
        self.configure(bg="white")
        self.capture_button = Button(self, text='Capture', bg='grey', border=0, command=lambda: self.capture_press(1, self.frame))
        #self.capture_button.grid(row=5, column=40, padx=10, pady=2, rowspan=30, columnspan=20)
        self.capture_button.grid(row=90, column=40, padx=10, pady=1, rowspan=1, columnspan=1)
        #self.capture_button.place(x=550, y=250)
        self.cap1 = cv2.VideoCapture(0)
        self.cap = cv2.VideoCapture(0)
        self.lmain = Label()

        self.vision_page_local = StringVar(value=self.controller.vision_list_save_var)
        self.vision_model_list = Listbox(self, bg='white', fg='black', listvariable=self.vision_page_local, bd=2, width=50, height=10, relief='solid')
        self.vision_model_list.grid(row=1, column=40, padx=10, pady=0, rowspan=30, columnspan=5)


        #self.controller.vision_model_list.grid(row=0, column=40, padx=10, pady=0, rowspan=2, columnspan=5)
        # self.add_vision_model_button = Button(self, text="Add Vision Model", bg="grey", border=0, command=lambda: self.add_vision_model_test()) #
        self.add_vision_model_button = Button(self, text="Add Vision Model", bg="grey", border=0,
                                              command=lambda: self.add_vision_model_json())  #
        self.add_vision_model_button.grid(row=30, column=40, padx=10, pady=0, rowspan=20, columnspan=2)

        self.edit_vision_model_button = Button(self, text="Edit Vision Model", bg="grey", border=0,command=lambda: self.vision_model_popup())
        self.edit_vision_model_button.grid(row=30, column=43, padx=10, pady=0, rowspan=20, columnspan=2)

        self.calibration_button = Button(self, text="Calibrate", bg="grey", border=0, command=lambda: self.run_calibration_test()) #
        self.calibration_button.grid(row=90, column=43, padx=10, pady=0, rowspan=2, columnspan=2)

        self.test_button = Button(self, text="Live Test", bg="grey", border=0, command=lambda: self.live_test_mode())
        self.test_button.grid(row=95, column=43, padx=10, pady=0, rowspan=2, columnspan=2)

        imageFrame = Frame(self, width=600, height=500)
        imageFrame.grid(row=0, column=0, rowspan=100)#, padx=10, pady=0, rowspan=30
        self.lmain = Label(imageFrame)
        self.lmain.grid(row=0, column=0)
        self.vision_size = IntVar()
        self.vision_size = self.vision_model_list.size()
        print("self.vision_size", self.vision_size)

        self.controller.vision_save += 1
        print("vision save = ", self.controller.vision_save)

        connected_label = Label(self, text="Calibration Status: ", bg="white")
        connected_label.grid(row=0, column=40, padx=10, pady=1, rowspan=1, columnspan=5)
        self.cal_canvas = Canvas(self, width=12, height=12, borderwidth=0)
        self.cal_canvas.grid(row=0, column=43, rowspan=1)
        self.calibration_status = self.cal_canvas.create_oval(2, 2, 13, 13, fill="white", outline="black")
        # self.show_image_iterative()
        # mainloop()
        # thread = threading.Thread(target=self.show_image, args=())
        thread = threading.Thread(target=self.show_image_iterative, args=())
        thread.start()

    def run_calibration_test(self):
        self.runningCalibration = True
        message = "click button to save calibration"
        title = "Save Calibration"
        finishCal = 1
        print("save calibration")
        finishCal = self.overWritePopUp(message, title)
        self.run_calibration()

    def run_calibration(self):
        # cal_filename = "calibration_grid2.jpg"
        # # cv2.imwrite(cal_filename, self.frame)
        # img = cv2.imread(cal_filename)
        overwrite = 1
        self.runningCalibration = False
        print("self.controller.cameraCalibrated = ", self.controller.cameraCalibrated)
        if self.controller.cameraCalibrated is True:
            #confirm overwrite of calibration before calibrating
            message = "Are you sure you want to overwrite the existing calibration?"
            title = "Overwrite Calibration"
            overwrite = self.overWritePopUp(message, title)
        #     if overwrite == 1:
        #         Calibrate.Calibration(img, 0)
        # elif self.controller.cameraCalibrated is False:
        #     Calibrate.Calibration(img, 0)
        #Calibrate.Calibration(self.frame)
        if overwrite == 1:
            successful_calibration = Calibrate.Calibration.calibration_success
            print("calibration successful? - ", successful_calibration)
            if successful_calibration == True:
                self.controller.programVariables['calibration']['calibrationPixelPerMm'] = Calibrate.Calibration.pixel2Mm
                self.controller.programVariables['calibration']['calibrationStatus'] = True
                self.controller.calibrationPixelPerMm = Calibrate.Calibration.pixel2Mm
                self.cal_canvas.itemconfig(self.calibration_status, fill="green")
                self.controller.good_calibration = 1
                self.controller.cameraCalibrated = True
                print("self.controller.cameraCalibrated = ", self.controller.cameraCalibrated)
                with open(self.controller.jsonFileName, "w") as file:
                    json.dump(self.controller.programVariables, file, indent=4)
                popupmsg("Calibration", "Calibration Successful!")
            elif successful_calibration == False:
                self.controller.programVariables['calibration']['calibrationStatus'] = False
                self.cal_canvas.itemconfig(self.calibration_status, fill="red")
                self.controller.good_calibration = 0
                self.controller.cameraCalibrated = False
                popupmsg("Calibration", "Calibration Failed!")
            else:
                self.cal_canvas.itemconfig(self.calibration_status, fill="red")
                self.controller.good_calibration = 0
                self.controller.cameraCalibrated = False
                popupmsg("Calibration", "Calibration Failed to Run!")

    def overWritePopUp(self, message, title):
        var = IntVar()
        cnt_pop = Toplevel()
        cnt_pop.geometry('400x100')
        cnt_pop.wm_title(title)
        label = Label(cnt_pop, text=message)
        label.grid(row=1, column=1, columnspan=2)
        confirmButton = Button(cnt_pop, text="Yes", command=lambda: var.set(1))
        confirmButton.grid(row=2, column=1, columnspan=2)
        returnButton = Button(cnt_pop, text="No", command=lambda: var.set(2))
        returnButton.grid(row=2, column=3, columnspan=2)
        confirmButton.wait_variable(var)
        cnt_pop.destroy()
        cnt_pop.update()
        return var.get()

    def show_image_iterative(self):
        while True:
            _, self.frame = self.cap1.read()
            self.frame = cv2.flip(self.frame, 1)
            if self.runningCalibration is True:
                Calibrate.Calibration(self.frame, 0)
                cv2img = cv2.cvtColor(Calibrate.Calibration.raw_calibration_img_resize, cv2.COLOR_BGR2RGBA)
                self.img = Image.fromarray(cv2img)
            elif self.runningCalibration is False:
                cv2img = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGBA)
                self.img = Image.fromarray(cv2img)
            if self.controller.liveVisionTestMode is True:
                currentSelection = self.vision_model_list.curselection()
                area, blackWhite, upper, lower = self.read_json_vision_file(currentSelection)
                print("live view")
                # self.testFunct(self.frame)
                cv2img = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                centerX, centerY = self.search_for_items(upper, lower, area, blackWhite, False, self.frame)
                for i in centerX:
                    cv2.circle(cv2img, (centerX[i], centerY[i]), 2, (255,0,2550), 3)
                self.img = Image.fromarray(cv2img)
            imgtk = ImageTk.PhotoImage(image=self.img)
            # self.lmain.imgtk = imgtk
            self.lmain.configure(image=imgtk)
            time.sleep(0.05)

            # self.lmain.after(self.show_image())

    def show_image(self):
        _, self.frame = self.cap1.read()
        self.frame = cv2.flip(self.frame, 1)
        cv2img = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGBA)
        self.img = Image.fromarray(cv2img)
        imgtk = ImageTk.PhotoImage(image=self.img)
        #self.lmain.imgtk = imgtk
        self.lmain.configure(image=imgtk)
        self.lmain.after(self.show_image())

    def capture_press(self, action, img_frame):
        self.filename = "savedimage.jpg"
        #cv2.imwrite(self.filename, self.frame)
        cv2.imwrite(self.filename, img_frame)
        self.load = Image.open(self.filename)
        if action == 1:
            self.imgtk2 = ImageTk.PhotoImage(image=self.load)
            popupimg(self.imgtk2, "Snapshot!")
        if action == 2:
            width = 400
            height = 288
            img_resize = self.load.resize((width, height))
            self.imgtk2 = ImageTk.PhotoImage(image=img_resize)
            #imgtk2 = ImageTk.PhotoImage(image=load)
            #new_img = PhotoImage.zoom(scale_w, scale_h)
            #new_img = imgtk2.resize(320, 240)
            vision_image_frame = Frame(self.vision_pop, width=width, height=height, borderwidth=2, relief='solid')
            vision_image_frame.grid(row=1, column=6, rowspan=10, padx=10)  # , padx=10, pady=0, rowspan=30
            #self.lmain1 = Label(vision_image_frame)
            #self.lmain1.grid(row=0, column=0)

            self.image_label = Label(vision_image_frame)
            self.image_label.configure(image=self.imgtk2)
            self.image_label.pack(side="top", fill="x", pady=10)
            self.vision_pop.mainloop()

    def calibration_popup(self):
        calibrate_pop = Toplevel()
        calibrate_pop.geometry('600x500')
        calibrate_pop.wm_title("Calibration Window")
        cal_img_frame = Frame(calibrate_pop, width=600, height=500)
        cal_img_frame.grid(row=0, column=0, rowspan=100)  # , padx=10, pady=0, rowspan=30
        calibrate_pop.cal_label = Label(cal_img_frame)
        calibrate_pop.cal_label.grid(row=0, column=0)
        print("outside of while loop")
        #calibrate_pop.show_image()
        _, cal_frame = self.cap1.read()
        cal_frame = cv2.flip(cal_frame, 1)
        cal_cv2img = cv2.cvtColor(cal_frame, cv2.COLOR_BGR2RGBA)
        cal_img = Image.fromarray(cal_cv2img)
        cal_imgtk = ImageTk.PhotoImage(image=cal_img)
        # self.lmain.imgtk = imgtk
        calibrate_pop.cal_label.configure(image=cal_imgtk)
        calibrate_pop.cal_label.after(self.calibration_popup())

    def vision_model_popup(self):
        self.vision_pop = Toplevel()
        self.vision_pop.geometry('880x380')
        self.vision_pop.wm_title("Define Vision Parameters")
        self.vision_model_name = StringVar()
        vision_model_selection = self.vision_model_list.curselection()
        self.vision_model_name = self.vision_model_list.get(vision_model_selection)
        self.local_vision_max_area = self.controller.vision_model_master[vision_model_selection][0]
        self.local_vision_min_area = self.controller.vision_model_master[vision_model_selection][1]
        print(self.vision_model_name)
        #edit_vision_model_label = Label(self.vision_pop, text="Edit Vision Model Parameters")
        #edit_vision_model_label.grid(row=0, column=0)
        vision_model_name_label = Label(self.vision_pop, text="Vision Model Name:")
        vision_model_name_label.grid(row=1, column=0)
        self.vision_model_name_entry = Entry(self.vision_pop, textvariable=self.vision_model_name, width=20)
        self.vision_model_name_entry.grid(row=1, column=1)
        self.vision_model_name_entry.delete(0, 'end')
        self.vision_model_name_entry.insert(0, self.vision_model_name)
        vision_edit_save_button = Button(self.vision_pop, text="Save", command=lambda: self.save_vision_model_parameters(vision_model_selection))
        vision_edit_save_button.grid(row=11, column=1)
        #self.local_vision_min_area = self.controller.vision_area_min
        #self.local_vision_max_area = self.controller.vision_area_max

        self.vision_max_label = Label(self.vision_pop, text="Min Area")
        self.vision_max_label.grid(row=2, column=0)
        self.area_min_label = Label(self.vision_pop, text=str(self.controller.vision_model_master[vision_model_selection][1]))
        self.area_min_label.grid(row=2, column=2)

        self.area_min_minus_button = Button(self.vision_pop, text="+") #, command=lambda: self.area_adjust(1, 1)
        self.area_min_minus_button.grid(row=2, column=3)
        self.area_min_minus_button.bind('<ButtonPress-1>', lambda event, args=(1, 1, vision_model_selection): self.area_start(args))
        self.area_min_minus_button.bind('<ButtonRelease-1>', lambda event: self.area_stop())

        self.area_min_plus_button = Button(self.vision_pop, text="-") #, command=lambda: self.area_adjust(-1, 1)
        self.area_min_plus_button.grid(row=2, column=1)
        self.area_min_plus_button.bind('<ButtonPress-1>', lambda event, args=(-1, 1, vision_model_selection): self.area_start(args))
        self.area_min_plus_button.bind('<ButtonRelease-1>', lambda event: self.area_stop())

        self.vision_max_label = Label(self.vision_pop, text="Max Area")
        self.vision_max_label.grid(row=3, column=0)
        self.area_max_label = Label(self.vision_pop, text=str(self.controller.vision_model_master[vision_model_selection][0]))
        self.area_max_label.grid(row=3, column=2)

        self.area_max_minus_button = Button(self.vision_pop, text="+") #, command=lambda: self.area_adjust(1, 2)
        self.area_max_minus_button.grid(row=3, column=3)
        self.area_max_minus_button.bind('<ButtonPress-1>', lambda event, args=(1, 2, vision_model_selection): self.area_start(args))
        self.area_max_minus_button.bind('<ButtonRelease-1>', lambda event: self.area_stop())

        self.area_max_plus_button = Button(self.vision_pop, text="-")  #, command=lambda: self.area_adjust(-1, 2)
        self.area_max_plus_button.grid(row=3, column=1)
        #self.area_max_plus_button.bind('<ButtonPress-1>', lambda event, args=(-1, 2): self.area_start(args))
        #self.area_max_plus_button.bind('<ButtonRelease-1>', lambda event: self.area_stop())

        self.area_max_plus_button.bind('<ButtonPress-1>', lambda event, args=(-1, 2, vision_model_selection): self.area_start(args))
        self.area_max_plus_button.bind('<ButtonRelease-1>', lambda event: self.area_stop()) #self.area_adjust(-1, 2)

        self.vision_capture_button = Button(self.vision_pop, text='Capture', bg='grey', border=0, command=lambda: self.capture_press(2, self.frame))
        # self.capture_button.grid(row=5, column=40, padx=10, pady=2, rowspan=30, columnspan=20)
        self.vision_capture_button.grid(row=10, column=0)

        #upper_rgb = np.array([self.controller.vision_rbg_values[0], self.controller.vision_rbg_values[2], self.controller.vision_rbg_values[4]])
        #lower_rbg = np.array([self.controller.vision_rbg_values[1], self.controller.vision_rbg_values[3], self.controller.vision_rbg_values[5]])
        #area_thresholds = [self.controller.vision_area_min, self.controller.vision_area_max]
        self.vision_search_button = Button(self.vision_pop, text='Search', command=lambda: self.search_button_press(vision_model_selection)) #
        self.vision_search_button.grid(row=10, column=2)

        self.color_scale = [0]*6
        self.color_label = [0]*6
        self.color_slider(4, 1, "Upper Red", 255)
        self.color_slider(5, 1, "Lower Red", 0)
        self.color_slider(6, 1, "Upper Blue", 255)
        self.color_slider(7, 1, "Upper Blue", 0)
        self.color_slider(8, 1, "Upper Green", 255)
        self.color_slider(9, 1, "Lower Green", 0)

        self.color_help_button = Button(self.vision_pop, text="Color Helper", bg='grey', border=0,
                                        command=lambda: VisionSlidersHelper.VisionSliderHelp(self.filename))
        self.color_help_button.grid(row=10, column=1)

        self.auto_define_param_buttom = Button(self.vision_pop, text="Define Parameters", bg='grey', border=0,
                                               command=lambda: TeachObject.ObjectAssist(self.filename))
                                               #command=lambda: self.object_assist_update_sliders(vision_model_selection))

        self.auto_define_param_buttom.grid(row=11, column=0)
        self.grab_vals_button = Button(self.vision_pop, text="Grab Values", bg='grey',
                                       command=lambda: self.object_assist_update_sliders(vision_model_selection))
        self.grab_vals_button.grid(row=11, column=3)

        self.black_on_white = True
        self.black_or_white_button = Button(self.vision_pop, text="Black on White",
                                            command=lambda: self.switch())
        self.black_or_white_button.grid(row=11, column=5)
        self.black_white = True
        try:
            self.image_label.bind("<Button-1>", self.start_box)
        except:
            pass

        self.update_sliders(vision_model_selection)
        self.capture_press(2, self.frame)

    def search_button_press(self, selected):
        upper_rgb = np.array([self.controller.vision_rbg_values[0], self.controller.vision_rbg_values[2],
                              self.controller.vision_rbg_values[4]])
        lower_rbg = np.array([self.controller.vision_rbg_values[1], self.controller.vision_rbg_values[3],
                              self.controller.vision_rbg_values[5]])
        area_thresholds = [self.controller.vision_area_min, self.controller.vision_area_max]
        self.read_json_vision_file(selected)
        self.search_for_items(upper_rgb, lower_rbg, area_thresholds, self.black_white, True, self.filename)

    def switch(self):
        if self.black_on_white is True:
            self.black_or_white_button.configure(text="White on Black")
            self.black_on_white = False
            #max_color = max(self.grey_center)
            #self.color_loc = self.grey_center.index(max_center)
        elif self.black_on_white is False:
            self.black_or_white_button.configure(text="Black on White")
            self.black_on_white = True
            #min_color = min(self.grey_center)
            #self.color_loc = self.grey_center.index(min_center)

    def object_assist_update_sliders(self, model_selection):
        print("I am here before going in object assist")
        #TeachObject.ObjectAssist(self.filename, model_selection)
        print("hopefully I come here after the object assist gets closed")
        #rgb = TeachObject.ObjectAssist.red_blue_green_values
        #area = TeachObject.ObjectAssist.min_max_area
        maxArea = TeachObject.ObjectAssist.maxArea
        colors = TeachObject.ObjectAssist.center
        self.black_white = TeachObject.ObjectAssist.black_on_white
        self.grey_center = [0]*len(colors)
        rgb = [0, 0, 0, 0, 0, 0]
        areaMargin = 20 #percent

        area = [100, 10000]
        area[0] = maxArea * (1 + (areaMargin / 100))
        area[1] = maxArea * (1 - (areaMargin / 100))
        k = len(colors)
        for i in range(0,k):
            self.grey_center[i] = sum(colors[i])/3
        #if self.black_on_white is True:
        if self.black_white is True:
            max_color = max(self.grey_center)
            self.color_loc = self.grey_center.index(max_color)
        #elif self.black_on_white is False:
        elif self.black_white is False:
            min_color = min(self.grey_center)
            self.color_loc = self.grey_center.index(min_color)
        j = 0
        # red-blue-green, upper-lower
        for i in range(6):
            if i % 2 == 0:
                #print(self.center[self.center_loc][j])
                rgb[i] = colors[self.color_loc][j] + int(colors[self.color_loc][j] * 0.2)
            elif i % 2 == 1:
                rgb[i] = colors[self.color_loc][j] - int(colors[self.color_loc][j] * 0.2)
                j += 1
        self.set_master_vision_model(model_selection, area, rgb, self.black_white)
        self.update_sliders(model_selection)

    def update_sliders(self, model_selection):
        for i in range(6):
            #self.color_scale[i].set(self.controller.vision_rbg_values[i])
            self.color_scale[i].set(self.controller.vision_model_master[model_selection][i+3])
        self.controller.vision_area_min = self.controller.vision_model_master[model_selection][0]
        self.controller.vision_area_max = self.controller.vision_model_master[model_selection][1]
        self.area_max_label.configure(text=str(self.controller.vision_model_master[model_selection][1]))
        self.area_min_label.configure(text=str(self.controller.vision_model_master[model_selection][0]))

    def start_box(self):
        print("the frame was clicked!")

    def color_slider(self, r, c, color_text, start):
        self.color_label_text = color_text
        self.r1 = r
        self.color_scale[self.r1-4] = Scale(self.vision_pop, from_=0, to_=255, orient=HORIZONTAL, length=200, showvalue=1)
        #self.color_scale[self.r1-4].configure(command=self.update_label)
        self.color_scale[self.r1-4].set(start)
        self.color_scale[self.r1-4].grid(row=r, column=c, columnspan=4)
        self.local_color_value = self.color_scale[self.r1-4].get()
        #self.color_text_and_value = self.color_label_text + "  " + str(self.color_scale[self.r1-4].get())
        self.color_label[self.r1-4] = Label(self.vision_pop, text=self.color_label_text)
        self.color_label[self.r1-4].grid(row=self.r1, column=0)
        #self.color_value = self.color_scale.get()

    def update_label(self, scale_value):  #, color_label, r1
        #print("self.scale_val = ", self.scale_val)
        print("r value = ", self.r1)
        print("self.color_label = ", self.color_label[self.r1-4])
        self.color_text_and_value = self.color_label_text + "  " + str(scale_value)
        self.color_label[self.r1-4].configure(text=self.color_text_and_value)
        self.color_value = scale_value

    def area_start(self, arguement):
        (adjust, min_or_max, selection) = arguement
        print("test output")
        self.area_start_thread = threading.Thread(target=self.area_adjust, args=(adjust, min_or_max, selection))
        self.area_start_thread.start()
        #self.area_adjust(adjust, min_or_max, 1)
        #if min_or_max == 1:
        #    self.local_vision_min_area += adjust
        #    self.area_min_label.config(text=str(self.local_vision_min_area))
        #elif min_or_max == 2:
        #    self.local_vision_max_area += adjust
        #    self.area_max_label.config(text=str(self.local_vision_max_area))

    def area_stop(self): #, adjustment, min_max
        #stop thread
        self.keep_going = 1
        self.controller.button_held_down = 1
        print("Should be stopping thread!!!!")
        print("Keep going - ", self.keep_going)
        self.area_start_thread.join()

    def area_adjust(self, adjustment, min_max, select): #, input
        counter = 0
        sleep_time = 0.1
        print("input = ", input)
        #while self.keep_going == 0:
        #self.local_vision_min_area = self.controller.vision_model_master[select][1]
        #self.local_vision_max_area = self.controller.vision_model_master[select][0]
        while self.controller.button_held_down == 0:
        #while input == 1:
            print("keep going? = ", self.controller.button_held_down)
            if min_max == 1:
                self.local_vision_min_area = self.local_vision_min_area + adjustment
                self.area_min_label.config(text=str(self.local_vision_min_area))
                print("area = ", self.local_vision_min_area)
            elif min_max == 2:
                self.local_vision_max_area = self.local_vision_max_area + adjustment
                self.area_max_label.config(text=str(self.local_vision_max_area))
                print("area = ", self.local_vision_max_area)
            counter += 1

            time.sleep(sleep_time)
            if counter > 50:
                sleep_time = 0.01
        self.keep_going = 0
        self.controller.button_held_down = 0
        #self.area_start_thread.join()

    def save_vision_model_parameters(self, vision_selection):
        self.vision_model_name = self.vision_model_name_entry.get()
        print("vision model name = ", self.vision_model_name)
        self.vision_model_list.delete(vision_selection)
        self.vision_model_list.insert(vision_selection, self.vision_model_name)
        min_max_area = [self.local_vision_min_area, self.local_vision_max_area]

        #self.controller.vision_model_master[vision_selection][0] = self.local_vision_max_area
        #self.controller.vision_model_master[vision_selection][1] = self.local_vision_min_area
        #for i in range(6):
        #    self.controller.vision_model_master[vision_selection][i+2] = self.color_scale[i].get()
        #print(self.controller.vision_model_master[vision_selection])
        #self.controller.vision_area_max = self.local_vision_max_area
        #self.controller.vision_area_min = self.local_vision_min_area

        for i in range(6):
            self.controller.vision_rbg_values[i] = self.color_scale[i].get()
        print("rbg values = ", self.controller.vision_rbg_values)
        self.write_json_vision_file(vision_selection, min_max_area, self.controller.vision_rbg_values,
                                     self.black_white, self.vision_model_name)
        self.set_master_vision_model(vision_selection, min_max_area, self.controller.vision_rbg_values, self.black_white)

    def set_master_vision_model(self, selected_model, area_min_max, rgb, black_white_select):
        #set max area
        self.controller.vision_model_master[selected_model][0] = area_min_max[1]
        #set min area
        self.controller.vision_model_master[selected_model][1] = area_min_max[0]
        #set black on white or white on black
        if black_white_select is True:
            self.controller.vision_model_master[selected_model][2] = 1
        if black_white_select is False:
            self.controller.vision_model_master[selected_model][2] = 2
        for i in range(6):
            self.controller.vision_model_master[selected_model][i+3] = rgb[i]

    def read_json_vision_file(self, selected_model):
        print("selected model = ", selected_model)
        filename = "visionModel" + str(selected_model[0] + 1) + ".json"
        name = 'Vision Model ' + str(selected_model[0] + 1)
        load = open(filename)
        data = json.load(load)
        count = 0
        print("data[name] = ", data)
        length = len(data[name])
        print("length = ", length)
        jsonNames = np.empty(length)
        print("jsonNames = ", jsonNames)
        count = 0
        for i in data[name]:
            print("data[", i, "] = ", data[name][i])
            jsonNames[count] = np.float32(data[name][i])
            count += 1
        print("jsonNames = ", jsonNames)
        # areas = [data[name][jsonNames[0]], data[name][jsonNames[1]]]
        # blackWhite = data[name][jsonNames[2]]
        # upperRGB = [data[name][jsonNames[3]], data[name][jsonNames[4]], data[name][jsonNames[5]]]
        # lowerRGB = [data[name][jsonNames[6]], data[name][jsonNames[7]], data[name][jsonNames[8]]]

        areas = [jsonNames[0], jsonNames[1]]
        if jsonNames[2] == 1:
            blackWhite = True
        else:
            blackWhite = False
        upperRGB = (jsonNames[3], jsonNames[5], jsonNames[7])
        lowerRGB = (jsonNames[4], jsonNames[6], jsonNames[8])

        return areas, blackWhite, upperRGB, lowerRGB

    def write_json_vision_file(self, selected_model, area_min_max, rgb, black_white_select, name):
        print("selected model = ", selected_model[0])
        # name = 'VisionModel' + str(selected_model[0]+1)
        updateVisionModel = {}
        updateVisionModel[name] = {
            'minArea': int(area_min_max[0]),
            'maxArea': int(area_min_max[1]),
            'blackOrWhite': black_white_select,
            'upperR': int(self.controller.vision_model_master[selected_model][3]),
            'lowerR': int(self.controller.vision_model_master[selected_model][4]),
            'upperG': int(self.controller.vision_model_master[selected_model][5]),
            'lowerG': int(self.controller.vision_model_master[selected_model][6]),
            'upperB': int(self.controller.vision_model_master[selected_model][7]),
            'lowerB': int(self.controller.vision_model_master[selected_model][8])
        }
        filename = "visionModel" + str(selected_model[0]+1) + ".json"
        print("in function")
        print(updateVisionModel)
        with open(filename, "w") as file:
            print(filename)
            json.dump(updateVisionModel, file, indent=4)
            print("dumped json")

    def add_vision_model_json(self):
        print("Vision model json test")
        # self.controller.vision_model_index += 1
        self.add_vision_model_test()
        name = 'VisionModel' + str(self.controller.vision_model_index)
        newVisionModel = {}
        newVisionModel[name] = {
            'minArea': -1,
            'maxArea': -1,
            'blackOrWhite': True,
            'upperR': 255,
            'lowerR': 0,
            'upperG': 255,
            'lowerG': 0,
            'upperB': 255,
            'lowerB': 0
        }
        #
        # self.controller.visionModel.resize((self.vision_model_index, 9), refcheck=False)
        # if self.controller.vision_model_index > 1:
        #     self.controller.visionModel.insert(END, newVisionModel)
        # self.add_vision_model_test()
        filename = "visionModel" + str(self.controller.vision_model_index) + ".json"
        # with open(self.controller.jsonVisionFileName, "w") as file:
        with open(filename, "w") as file:
            # json.dump(self.controller.visionModel, file, indent=4)
            json.dump(newVisionModel, file, indent=4)

    def add_vision_model_test(self):
        print("vision model test")
        self.controller.vision_model_index += 1
        self.vision_size = self.vision_model_list.size()
        vision_model_name = 'Vision Model ' + str(self.controller.vision_model_index)
        # vision data tuple = name, max_area, min_area, upper red, lower red, upper blue, lower blue, upper green, lower green
        #vision_data = (vision_model_name, 5000, 100, 255, 0, 255, 0, 255, 0)
        # vision data tuple = max_area, min_area, upper red, lower red, upper blue, lower blue, upper green, lower green
        vision_data = [5000, 100, 1, 255, 0, 255, 0, 255, 0]
        # , capture x, capture y, capture z, place x, place y, place z
        #, 0, 0, 0, 0, 0, 0
        print("vision_model_index = ", self.controller.vision_model_index)
        self.vision_model_list.insert(END, vision_model_name)
        self.vision_size = self.vision_model_list.size()

        self.controller.vision_model_master.resize((self.vision_size, 9), refcheck=False)
        self.controller.vision_model_master[self.vision_size-1] = vision_data
        print(self.controller.vision_model_master)

    def add_vision_model(self):
        #self.vision_model_index = self.vision_model_list.size() + 1
        self.controller.vision_model_index += 1
        self.vision_size = self.vision_model_list.size()
        vision_model_name = 'Vision Model ' + str(self.controller.vision_model_index)
        print("vision_model_index = ", self.controller.vision_model_index)
        self.vision_model_list.insert(END, vision_model_name)
        self.vision_size = self.vision_model_list.size()
        #print("vision model size = ", self.vision_model_list.size())

    def live_test_mode(self):
        print("hello????")
        self.controller.liveVisionTestMode = not self.controller.liveVisionTestMode
        if self.controller.liveVisionTestMode is True:
            self.test_button.configure(bg="green")
        elif self.controller.liveVisionTestMode is False:
            self.test_button.configure(bg="grey")

    def testFunct(self, testParam):
        cv2.imshow("test output function", testParam)

    def search_for_items(self, upper, lower, area_thresh, black_or_white, show_imgs, file_name):
        #search for items based on current parameters, not saved
        print("searching for items")
        #load_img = Image.open(self.filename)
        print("filename = ", file_name)
        if "savedimage" in file_name:
            load_img = cv2.imread(file_name)
            # cv2.imshow("test image", load_img)
            #load_img = cv2.imread(self.filename)
            #load_img = cv2.imread("test_img.jpg")
            # HSV_img = cv2.cvtColor(load_img, cv2.COLOR_BGR2HSV)
            HSV_img = cv2.cvtColor(load_img, cv2.COLOR_RGB2HSV)
        else:
            HSV_img = cv2.cvtColor(file_name, cv2.COLOR_BGR2HSV)
            print("this is happening right?")
            # cv2.imshow("test output", file_name)
        #upper = np.array([self.controller.vision_rbg_values[0], self.controller.vision_rbg_values[2], self.controller.vision_rbg_values[4]])
        #lower = np.array([self.controller.vision_rbg_values[1], self.controller.vision_rbg_values[3], self.controller.vision_rbg_values[5]])
        count = 0
        count2 = 0
        print("upper = ", upper)
        print("lower = ", lower)
        #threshold_img = cv2.inRange(HSV_img, (self.controller.vision_rbg_values[1], self.controller.vision_rbg_values[3], self.controller.vision_rbg_values[5]),
        #            (self.controller.vision_rbg_values[0], self.controller.vision_rbg_values[2], self.controller.vision_rbg_values[4]))
        # if self.black_on_white == False:
        #     threshold_img = cv2.inRange(load_img, lower, upper)
        # elif self.black_on_white == True:
        #     intermediate = cv2.inRange(load_img, lower, upper)
        #     ret, threshold_img = cv2.threshold(intermediate, 127, 255, cv2.THRESH_BINARY_INV)
        # for i in range(3):
        #     upper[i] = int(upper[i])
        #     lower[i] = int(lower[i])
        if black_or_white == False:
            # threshold_img = cv2.inRange(load_img, lower, upper)
            # threshold_img = cv2.inRange(HSV_img, upper, lower)
            threshold_img = cv2.inRange(HSV_img, lower, upper)
        elif black_or_white == True:
            # intermediate = cv2.inRange(load_img, lower, upper)
            # intermediate = cv2.inRange(HSV_img, upper, lower)
            print("lower = ", lower)
            print("upper = ", upper)
            intermediate = cv2.inRange(HSV_img, lower, upper)
            ret, threshold_img = cv2.threshold(intermediate, 127, 255, cv2.THRESH_BINARY_INV)

        contours, hierarchy = cv2.findContours(threshold_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        # print("hierarchy = ", hierarchy)
        print("len(contours) = ", len(contours))
        #print("contours = ", contours)
        area_min = area_thresh[0]
        area_max = area_thresh[1]
        num_contours = 0
        cX = np.array([0,0])
        cY = np.array([0,0])
        area = 0
        count = 0
        # for (c, h) in zip(contours, hierarchy[0]):
        for c in contours:

            # print("h = ", h)
            # print("c = ", c)
            # print("hierarchy = ", hierarchy[c])

            M = cv2.moments(c)
            area = cv2.contourArea(c)
            minCircle = cv2.minEnclosingCircle(c)
            radius = int(minCircle[1])
            minCircleArea = 3.14 * (radius * radius)
            if minCircleArea > 0:
                currentCircularity = area / minCircleArea
                areaBool = (area > area_min and area < area_max)
                circularityHighBool = (currentCircularity > (TeachObject.ObjectAssist.Circularity * 1.2))
                circularityLowBool = (currentCircularity < (TeachObject.ObjectAssist.Circularity * 0.8))
                # print(area > area_min and area < area_max)
                # print(currentCircularity > (TeachObject.ObjectAssist.Circularity * 1.2))
                # print(currentCircularity < (TeachObject.ObjectAssist.Circularity * 0.8))
                #if area > self.controller.vision_area_min and area < self.controller.vision_area_max:
                # if (area > area_min and area < area_max) and \
                #         (currentCircularity > (TeachObject.ObjectAssist.Circularity * 1.2) and
                #          currentCircularity < (TeachObject.ObjectAssist.Circularity * 0.8)):
                if areaBool and circularityHighBool and circularityLowBool:
                    print("count = ", count)
                    count = count + 1
                    # cv2.drawContours(load_img, c, -1, (255, 255, 0), 3)
                    print("area = ", area)
                    if M["m00"] != 0:
                        cX[num_contours] = int(M["m10"] / M["m00"])
                        cY[num_contours] = int(M["m01"] / M["m00"])
                    else:
                        cX[num_contours] = 0
                        cY[num_contours] = 0
                    #print("cx = ", cX)
                    #print("cy = ", cY)
                    if show_imgs == True:
                        cv2.drawContours(load_img, c, -1, (255, 255, 0), 3)
                        cv2.circle(load_img, (cX[num_contours], cY[num_contours]), 5, (0, 0, 255), -1)
                    num_contours += 1
                    cX.resize(num_contours + 1)
                    cY.resize(num_contours + 1)
        if show_imgs == True:
            cv2.drawContours(load_img, contours, -1, (0, 255, 0), 3)
            cv2.imshow("threshold_img", threshold_img)
            cv2.imshow("original", load_img)
            print("cX = ", cX)
            print("cY = ", cY)
            #cv2.imshow("contours", contours)
        return cX, cY

    def close_save(self):
        self.controller.vision_list_save_var = [0] * self.vision_model_list.size()
        print("Existing vision models at close: ")
        for i in range(0, self.vision_model_list.size()):
            self.controller.vision_list_save_var[i] = self.vision_model_list.get(i)
            print(self.vision_model_list.get(i))
        self.cap1.release()
        self.controller.show_frame_1("HomePage")


class SetIO(Frame):

    def __init__(self, parent, controller, attr=None):
        Frame.__init__(self, parent)
        self.controller = controller
        home_button = Button(self, text="Home Screen", bg="grey", border=0,
                             command=lambda: controller.show_frame_1("HomePage"))
        home_button.grid(row=1, column=1)

        io_ports = 24
        io_numbers = [0]*io_ports
        for i in range(0, len(io_numbers)):
            io_numbers[i] = i+1
        self.io_select = [""] * 5
        self.io_direction = {}
        self.io_steps = {}
        io_select2 = 0
        print(self.io_select)
        #for i in range(0, 5):
        #    motor_text = "Motor " + str(i + 1) + " Output"
        #    step_label = Label(self, text=motor_text)
        #    step_label.grid(row=3 + (i * 2), column=1, rowspan=2)
        j = 0
        for i in range(0, 5):
            motor_text = "Motor " + str(i + 1) + " Output"
            step_label = Label(self, text=motor_text)
            step_label.grid(row=3 + (i * 2), column=1, rowspan=2)
            steps = ttk.Combobox(self, values=io_numbers)  #
            self.io_steps[i] = steps
            steps.grid(row=3+j, column=3)
            steps.current(j+5)
            direction = ttk.Combobox(self, values=io_numbers)  #
            self.io_direction[i] = direction
            direction.grid(row=4+j, column=3)
            direction.current(j+6)
            direct_label = Label(self, text="Direction Pin")
            direct_label.grid(row=4+j, column=2)
            step_label = Label(self, text="Steps Pin")
            step_label.grid(row=3+j, column=2)
            j = j + 2
        set_button = Button(self, text="Set IO", command=self.set_io)
        set_button.grid(row=14, column=1)
        self.speed_scale = Scale(self, from_=1, to=100, sliderlength=20)
        self.speed_scale.grid(row=2, column=20, rowspan=7, columnspan=1, padx=100)
        speed_label = Label(self, text="Velocity")
        speed_label.grid(row=1, column=20, columnspan=1, padx=100)

    def set_io(self):

        combined = [0] * (len(self.io_steps) + len(self.io_direction))
        for i in range(0, len(combined)):
            if i < len(self.io_steps):
                combined[i] = self.io_steps[i].current()+1
            else:
                combined[i] = self.io_direction[i-len(self.io_steps)].current()+1
        print(combined)
        i = 0
        j = 0
        while i < len(combined):
            #print("first loop")
            while j < len(combined):
                #print("second loop")
                if combined[i] == combined[j] and i != j:
                    title = "Invalid IO"
                    message = "Cannot repeat pins for IO"
                    popupmsg(title, message)
                #print("i = ", i)
                #print("j = ", j)
                j += 1
            i += 1
            j = 0

        print("IO Set on Computer!")
        self.communicate('Set IO', 'IO Ready', 10)
        for k in range(0, int(len(combined)/2)):
            send = "Joint" + str(k+1) + "Step" + str(combined[k]) + "Direction" + str(combined[k+5])
            self.communicate(send, 'Next', 10)
            time.sleep(0.1)
        #send = "Complete"
        #self.communicate(send, "All IO Set", 10)
        #send = "Speed" + str(self.speed_scale.get())
        #self.communicate(send, "Speed Set", 10)

    def communicate(self, send, back, timeout):
        print(send)
        self.controller.arduino_data.write(send.encode())
        time.sleep(0.5)
        respond = self.controller.arduino_data.readline().decode()
        kill = 0
        #print(respond)
        while respond not in back:
            time.sleep(2)
            print("before readline")
            respond = self.controller.arduino_data.readline().decode()
            print("after readline")
            print("respond = ", respond)
            kill += 1
            print("kill = ", kill)
            if kill == timeout:
                title = "timeout"
                message = "No Response, try to reconnect."
                popupmsg(title, message)
        print("respond2 = ", respond)


#root = Tk()
sys.setrecursionlimit(10**9)
app = MaterRobotProgram()
#app = VisionPage(master=root)
app.mainloop()