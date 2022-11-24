import numpy as np
import threading
import time
from tkinter import *
import tkinter.ttk as ttk


class Pattern:
    def __init__(self, orgin):
        self.pattern_builder_window = Toplevel()
        self.pattern_builder_window.geometry('450x150')
        self.pattern_builder_window.title("Pattern Building Tool")
        column_base = 8
        column_base2 = 12
        self.padding = 10
        self.double_arrow_label(2, column_base, 0)
        self.double_arrow_label(4, column_base, 1)
        self.pattern_size = [1, 1]
        self.generate_button = Button(self.pattern_builder_window, text="Generate",
                                      command=lambda: self.build_pattern_array(self.pattern_size[0],
                                                                               self.pattern_size[1], orgin,
                                                                               float(self.span_distance_input.get()),
                                                                               float(self.height_distance_input.get())))
        self.generate_button.grid(row=10, column=column_base-1, columnspan=4, padx=self.padding)
        self.insertButton = Button(self.pattern_builder_window, text="Record", command=lambda: self.addPattern())
        self.insertButton.grid(row=20, column=(column_base+column_base2)/2, columnspan=4, padx=self.padding)
        self.span_distance_input = Entry(self.pattern_builder_window, width=10)
        self.span_distance_input.grid(row=6, column=column_base-1, columnspan=4, padx=self.padding)
        self.height_distance_input = Entry(self.pattern_builder_window, width=10)
        self.height_distance_input.grid(row=8, column=column_base-1, columnspan=4, padx=self.padding)
        self.entry_description_labels(6, column_base, "Span Distance")
        self.entry_description_labels(8, column_base, "Height Distance")
        orgin_label = Label(self.pattern_builder_window, text="Pattern Orgin")
        orgin_label.grid(row=2, column=column_base2, columnspan=6, padx=self.padding)
        xyz = ["X", "Y", "Z"]
        for i in range(3):
            coord_text = xyz[i] + " Coordinate"
            coor_label = Label(self.pattern_builder_window, text=coord_text)
            coor_label.grid(row=(4+(i*2)), column=column_base2, columnspan=4, padx=self.padding)
        load_loc_button = Button(self.pattern_builder_window, text="Use Current Coordinates")
        load_loc_button.grid(row=10, column=column_base2, columnspan=8, padx=self.padding)
        x_coord_str = StringVar()
        x_coord_str = str(orgin[0])
        y_coord_str = StringVar()
        y_coord_str = str(orgin[1])
        z_coord_str = StringVar()
        z_coord_str = str(orgin[2])
        Pattern.output_pattern_array = 0
        self.x_coord_entry = Entry(self.pattern_builder_window, width=10, textvariable=x_coord_str)
        self.x_coord_entry.insert(0, x_coord_str)
        self.x_coord_entry.grid(row=4, column=column_base2+4, columnspan=4, padx=self.padding)
        self.y_coord_entry = Entry(self.pattern_builder_window, width=10, textvariable=y_coord_str)
        self.y_coord_entry.insert(0, y_coord_str)
        self.y_coord_entry.grid(row=6, column=column_base2+4, columnspan=4, padx=self.padding)
        self.z_coord_entry = Entry(self.pattern_builder_window, width=10, textvariable=z_coord_str)
        self.z_coord_entry.grid(row=8, column=column_base2 + 4, columnspan=4, padx=self.padding)
        self.z_coord_entry.insert(0, z_coord_str)
        #mainloop()

    def addPattern(self):
        Pattern.completePattern = True

    def entry_description_labels(self, r, c, text_input):
        entry_label = Label(self.pattern_builder_window, text=text_input)
        entry_label.grid(row=r, column=(c-8), columnspan=6, padx=self.padding)

    def double_arrow_label(self, r, c, pattern_direction):
        descriptions = ["Number of Rows", "Number of Columns"]
        arrow_column_span = 2
        description_label = Label(self.pattern_builder_window, text=descriptions[pattern_direction])
        description_label.grid(row=r, column=(c-8), columnspan=6, padx=self.padding)
        up_button = Button(self.pattern_builder_window, text=">", command=lambda: self.change_value(1,pattern_direction, value_label))
        up_button.grid(row=r, column=(c+2), columnspan=arrow_column_span)
        down_button = Button(self.pattern_builder_window, text="<", command=lambda: self.change_value(-1,pattern_direction, value_label))
        down_button.grid(row=r, column=(c-2), columnspan=arrow_column_span)
        value_label = Label(self.pattern_builder_window, text=" 1 ")
        value_label.grid(row=r, column=c, columnspan=arrow_column_span)

    def change_value(self, change, direction, label_of_value):
        self.pattern_size[direction] += change
        if self.pattern_size[direction] < 1:
            self.pattern_size[direction] = 1
        if self.pattern_size[direction] > 20:
           self.pattern_size[direction] = 20
        label_of_value.configure(text=self.pattern_size[direction])
        #print("pattern_size = ", self.pattern_size)

    def build_pattern_array(self, length_num, width_num, orgin_xyz, length_mm, width_mm):
        Pattern.output_pattern_array = np.zeros([length_num, width_num, 3], dtype='float')
        # print(length_num)
        # print(width_num)
        # print(orgin_xyz)
        # print(length_mm)
        # print(width_mm)
        # print("math = ", (orgin_xyz[0] + length_mm))
        length_incriment = length_mm / length_num
        width_incriment = width_mm / width_num
        k = 0
        Pattern.output_pattern_array[0][0][0] = orgin_xyz[0]
        Pattern.output_pattern_array[0][0][1] = orgin_xyz[1]
        Pattern.output_pattern_array[0][0][2] = orgin_xyz[2]
        for i in range(0, length_num):
            Pattern.output_pattern_array[i][0][0] = orgin_xyz[0]
            Pattern.output_pattern_array[i][0][2] = orgin_xyz[2]
            if i > 0:
                Pattern.output_pattern_array[i][0][1] = Pattern.output_pattern_array[i-1][0][1] + length_incriment
            for j in range(1, width_num):
                Pattern.output_pattern_array[i][j][0] = Pattern.output_pattern_array[i][j-1][0] + width_incriment
                Pattern.output_pattern_array[i][j][1] = Pattern.output_pattern_array[i][j-1][1]
                Pattern.output_pattern_array[i][j][2] = Pattern.output_pattern_array[i][j-1][2]
                print(Pattern.output_pattern_array[i][j][1], end="   ")
            print("")
        #print(output_pattern_array[0:length_num][0:width_num][0])


#Pattern([10,10,10])
