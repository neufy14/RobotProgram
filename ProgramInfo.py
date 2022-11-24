import numpy as np
import threading
import time
from tkinter import *
import tkinter.ttk as ttk

class PositionInfo:
    def __init__(self, index, pos):
        print("in position info")
        # Grid layout variables
        self.labelSpan = 2
        self.entrySpan = 3
        self.numColums = 2
        self.positionEntryWidth = 10
        self.positionLabelWidth = 10
        self.entryValPad = 5
        self.pageWidth = (self.labelSpan + self.entrySpan) * self.numColums

        PositionInfo.valueSaved = False
        PositionInfo.valueOutDate = False

        self.programInfoPage = Toplevel()
        PositionInfo.isOpen = self.programInfoPage.winfo_exists()
        self.programInfoPage.geometry("300x200")
        print("index = ", index)
        name = "Position " + str(index+1)
        PositionInfo.nameAndCoords = [name, pos]
        self.programInfoPage.title(name)
        self.coordinateLocationLabels([(int(self.labelSpan/2)), 2], pos)
        self.positionName([1, 2], name)
        self.SaveButton = Button(self.programInfoPage, text='Save', command=lambda: self.savePosition())
        self.SaveButton.grid(row=5, column=int(self.pageWidth/2), columnspan=self.entrySpan)
        # PositionInfo.isOpen = self.programInfoPage.winfo_exists()
        print("window is open = ", PositionInfo.isOpen)
        # mainloop()

    def coordinateLocationLabels(self, orgin, coordinates):
        posText = ['X', 'Y', 'Z', 'W', 'P', 'R']
        self.posValue = []
        for i in range(6):
            labelText = str(posText[i]) + ' Position = '
            posLabel = Label(self.programInfoPage, text=labelText, width=self.positionLabelWidth)
            self.posValue.append(Entry(self.programInfoPage, width=self.positionEntryWidth))
            # self.posValue = Entry(self.programInfoPage, width=self.positionEntryWidth)
            if i < int(len(posText)/self.numColums):
                colLoc = orgin[0]
                rowLoc = i+orgin[1]
            else:
                colLoc = self.entrySpan+orgin[0]+self.labelSpan
                rowLoc = (i-int(len(posText)/self.numColums))+orgin[1]
                # print(rowLoc)
                # print(colLoc)
                # print(self.labelSpan)
            posLabel.grid(row=rowLoc, column=colLoc, columnspan=self.labelSpan)
            self.posValue[i].grid(row=rowLoc, column=colLoc+self.labelSpan, columnspan=self.entrySpan, padx=self.entryValPad)
            self.posValue[i].insert(0, coordinates[i])

    def positionName(self, orgin, name):
        self.positionNameLabel = Label(self.programInfoPage, text='Position Name')
        self.positionNameLabel.grid(row=orgin[0], column=orgin[1], columnspan=self.labelSpan*2)
        self.positionNameEntry = Entry(self.programInfoPage)
        self.positionNameEntry.grid(row=orgin[0], column=orgin[1]+self.entrySpan, columnspan=self.entrySpan)
        self.positionNameEntry.insert(0, name)

    def savePosition(self):
        print('save')
        PositionInfo.valueSaved = True
        newName = self.positionNameEntry.get()
        newPos = [0, 0, 0, 0, 0, 0]
        # for i in range(6):
        #     newPos[i] = i.self.posValue.get()
        newPos = [i.get() for i in self.posValue]
        PositionInfo.nameAndCoords = [newName, newPos]
        print(PositionInfo.nameAndCoords)

# XYZWPR = [1,2,3,4,5,6]
# PositionInfo(0, XYZWPR)