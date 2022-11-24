import InverseKinematics
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from tkinter import *
import tkinter.ttk as ttk
import threading
import math

def simulate(spots, num):
    x_spot = np.zeros([7])
    y_spot = np.zeros([7])
    z_spot = np.zeros([7])
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
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
    # self.fig = plt.figure()
    # self.ax = self.fig.add_subplot(111, projection='3d')
    sc = ax.scatter(x_spot, y_spot, z_spot, c='r', marker='o')
    for i in range(1, len(x_spot)):
        pl = ax.plot([x_spot[i - 1], x_spot[i]], [y_spot[i - 1], y_spot[i]], [z_spot[i - 1], z_spot[i]], c='r')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500, 500)
    ax.set_zlim(-50, 400)

    # anim = FuncAnimation(fig, self.update, repeat=False, fargs=(ax, spots, sc, pl), frames=np.arange(0, len(spots)),
    #                     interval=50, blit=False)

    plt.show()

    # sim_window = Toplevel()
    # title = 'Simulated View ' + str(num)
    # sim_window.wm_title(title)
    # canvas = FigureCanvasTkAgg(fig, sim_window)
    # # canvas.show()
    # canvas.get_tk_widget().pack(side=BOTTOM, fill=BOTH, expand=True)

    # if self.update_ready == 1:
    #    print("test, am I here?")
    #    FuncAnimation(fig, self.update, repeat=False, fargs=(ax, spots, sc, pl),
    #                  frames=np.arange(0, len(spots)),interval=50, blit=False)
    #    self.update_ready = 0

# angles = [0, 40, 50, 0, -90, 0]
# angles = [14.4, 59.07, 2.72, 0, -60, 7.2]
angles = [90.0, 56.87, 6.9, 0.0, -63.71, 0.0]
x = 10
z = 5
theta = math.atan(z/x)
theta2 = math.atan2(z, x)
print("theta = ", theta)
print("theta2 = ", theta2)
InverseKinematics.Kinematics(2, angles)
cartesianCoordinates = InverseKinematics.Kinematics.position
allJointCoordiantes = InverseKinematics.Kinematics.all_positions
t1 = threading.Thread(target=simulate, args=(allJointCoordiantes, 1))
t1.start()

# simulate(allJointCoordiantes, 1)
InverseKinematics.Kinematics(1, cartesianCoordinates)
newAngles = InverseKinematics.Kinematics.joints
# angleDifference = np.array(newAngles) - np.array(angles)
# InverseKinematics.Kinematics(2, newAngles)
# newCartesianCoordinates = InverseKinematics.Kinematics.position
# allNewJointCoordiantes = InverseKinematics.Kinematics.all_positions
# # t2 = threading.Thread(target=simulate, args=(allNewJointCoordiantes, 2))
# # t2.start()
# # simulate(allNewJointCoordiantes, 2)
# InverseKinematics.Kinematics(1, newCartesianCoordinates)
# newNewAngles = InverseKinematics.Kinematics.joints
# newAngleDifference = np.array(newNewAngles) - np.array(newAngles)


print("cartesianCoordinates = ", cartesianCoordinates)
print("newAngles = ", newAngles)
# print("angleDifference = ", angleDifference)
# print("newCartesianCoordinates = ", newCartesianCoordinates)
# print("newNewAngles = ", newNewAngles)
# print("newAngleDifference = ", newAngleDifference)

# print(allJointCoordiantes)
# print(allNewJointCoordiantes)

mainloop()