import math
import numpy as np
from numpy.linalg import inv

class KinematicTest:
    def __init__(self, reverse_input):
        a1 = 10
        a2 = 10
        a3 = 10
        a4 = 10
        a5 = 2
        a6 = 2
        input_matrix = [[0, math.radians(90), 0, a1],
                        [0, 0, a2, 0],
                        [0, math.radians(+90), 0, 0],
                        [0, math.radians(-90), 0, (a3 + a4)],
                        [0, math.radians(+90), 0, 0],
                        [0, 0, 0, (a5 + a6)]]

        pX = [0] * 2
        pY = [0] * 2
        pX_a1 = [0] * 2
        pa2H = [0] * 2
        pa3H = [0] * 2
        Theta_A = [0] * 2
        Theta_B = [0] * 2
        Theta_C = [0] * 2
        Theta_D = [0] * 2
        Theta_E = [0] * 2
        J2_Angle = [0] * 2
        J3_Angle = [0] * 2
        J4_Angle = [0] * 2
        J5_Angle = [0] * 2
        temp1 = np.zeros([4, 4])
        J_Matrix = np.zeros([4, 4, 6])
        allArms = [a1, a2, a3, a4, a5, a6]
        cartesianXYZ = [reverse_input[0], reverse_input[1], reverse_input[2]]
        # self.geometricInverse(cartesianXYZ, allArms)
        R0_T = np.zeros([4, 4])
        # R0_T[0][0] = ((math.cos(reverse_input[3])) * (math.cos(reverse_input[5]))) - (
        #         math.cos(reverse_input[4]) * math.sin(reverse_input[3]) * math.sin(reverse_input[5]))
        # R0_T[0][1] = ((math.sin(reverse_input[3])) * (math.cos(reverse_input[5]))) + (
        #         math.cos(reverse_input[4]) * math.cos(reverse_input[3]) * math.sin(reverse_input[5]))
        # R0_T[0][2] = ((math.sin(reverse_input[3])) * (math.sin(reverse_input[4])))
        # R0_T[0][3] = reverse_input[0]
        # R0_T[1][0] = ((math.cos(reverse_input[4])) * (math.cos(reverse_input[5])) * math.sin(reverse_input[3])) + (
        #         math.cos(reverse_input[3]) * math.sin(reverse_input[5]))
        # R0_T[1][1] = ((math.cos(reverse_input[4])) * (math.cos(reverse_input[5])) * math.cos(reverse_input[3])) - (
        #         math.sin(reverse_input[3]) * math.sin(reverse_input[5]))
        # R0_T[1][2] = -((math.cos(reverse_input[3])) * (math.sin(reverse_input[4])))
        # R0_T[1][3] = reverse_input[1]
        # R0_T[2][0] = ((math.sin(reverse_input[4])) * (math.sin(reverse_input[5])))
        # R0_T[2][1] = ((math.cos(reverse_input[3])) * (math.sin(reverse_input[4])))
        # R0_T[2][2] = -1 * ((math.cos(reverse_input[4])))
        # R0_T[2][3] = reverse_input[2]
        # R0_T[3][0] = 0
        # R0_T[3][1] = 0
        # R0_T[3][2] = 0
        # R0_T[3][3] = 1

        # R0_T[0][0] = ((math.cos(reverse_input[4])) * (math.cos(reverse_input[3]))) - (
        #         math.cos(reverse_input[5]) * math.sin(reverse_input[4]) * math.sin(reverse_input[3]))
        # R0_T[0][1] = ((math.sin(reverse_input[4])) * (math.cos(reverse_input[3]))) + (
        #         math.cos(reverse_input[5]) * math.cos(reverse_input[4]) * math.sin(reverse_input[3]))
        # R0_T[0][2] = ((math.sin(reverse_input[4])) * (math.sin(reverse_input[5])))
        # R0_T[0][3] = reverse_input[0]
        # R0_T[1][0] = ((math.cos(reverse_input[5])) * (math.cos(reverse_input[3])) * math.sin(reverse_input[4])) + (
        #         math.cos(reverse_input[4]) * math.sin(reverse_input[3]))
        # R0_T[1][1] = ((math.cos(reverse_input[5])) * (math.cos(reverse_input[3])) * math.cos(reverse_input[4])) - (
        #         math.sin(reverse_input[4]) * math.sin(reverse_input[3]))
        # R0_T[1][2] = -((math.cos(reverse_input[4])) * (math.sin(reverse_input[5])))
        # R0_T[1][3] = reverse_input[1]
        # R0_T[2][0] = ((math.sin(reverse_input[5])) * (math.sin(reverse_input[3])))
        # R0_T[2][1] = ((math.cos(reverse_input[4])) * (math.sin(reverse_input[5])))
        # R0_T[2][2] = -1 * (math.cos(reverse_input[5]))
        # R0_T[2][3] = reverse_input[2]
        # R0_T[3][0] = 0
        # R0_T[3][1] = 0
        # R0_T[3][2] = 0
        # R0_T[3][3] = 1

        R0_T[0][0] = (math.cos(reverse_input[5]) * math.cos(reverse_input[4]))
        R0_T[0][1] = (math.cos(reverse_input[5]) * math.sin(reverse_input[4]) * math.sin(reverse_input[3])) - \
                     ((math.sin(reverse_input[5])) * (math.cos(reverse_input[3])))
        R0_T[0][2] = ((math.cos(reverse_input[5])) * (math.sin(reverse_input[4])) * math.cos(reverse_input[3])) + \
                     ((math.sin(reverse_input[5])) * (math.sin(reverse_input[3])))
        R0_T[0][3] = reverse_input[0]
        R0_T[1][0] = (math.sin(reverse_input[5]) * math.cos(reverse_input[4]))
        R0_T[1][1] = (math.sin(reverse_input[5]) * math.sin(reverse_input[4]) * math.sin(reverse_input[3])) + \
                     ((math.cos(reverse_input[5])) * (math.cos(reverse_input[3])))
        R0_T[1][2] = ((math.sin(reverse_input[5])) * (math.sin(reverse_input[4])) * math.cos(reverse_input[3])) - \
                     ((math.cos(reverse_input[5])) * (math.sin(reverse_input[3])))
        R0_T[1][3] = reverse_input[1]
        R0_T[2][0] = -1 * math.sin(reverse_input[4])
        R0_T[2][1] = math.cos(reverse_input[4]) * math.sin(reverse_input[3])
        R0_T[2][2] = math.cos(reverse_input[4]) * math.cos(reverse_input[3])
        R0_T[2][3] = reverse_input[2]
        R0_T[3][0] = 0
        R0_T[3][1] = 0
        R0_T[3][2] = 0
        R0_T[3][3] = 1

        print("R0_T")
        print(R0_T)
        R0_3_Trans_Small = np.zeros([3, 3])
        temp_small = np.zeros([3, 3])
        work_frame = np.zeros([4, 4])
        tool_frame = np.zeros([4, 4])
        for i in range(len(work_frame)):
            work_frame[i,i] = 1
            tool_frame[i, i] = 1

        R0_T_workFrame = self.matrix_multiply(R0_T, work_frame)
        work_frame_inverse = np.linalg.inv(work_frame)
        R0_6 = self.matrix_multiply(R0_T_workFrame, work_frame_inverse)
        input_matrix[5][0] = math.radians(-180)
        # input_matrix[5][1] = math.radians(-90)
        # input_matrix[5][0] = reverse_input[4]
        # input_matrix[5][1] = reverse_input[5]
        # input_matrix[5][0] = math.radians(reverse_input[4])
        i = 5
        # fill J6 matrix
        # There is a problem with how I fill J6, need to check indexes
        for j in range(0, 4):
            for k in range(0, 4):
                if j == 0:
                    if k == 0:
                        J_Matrix[j][k][i] = math.cos(input_matrix[i][0])
                    elif k == 1:
                        J_Matrix[j][k][i] = -1 * math.cos(input_matrix[i][1]) * math.sin(input_matrix[i][0])
                    elif k == 2:
                        J_Matrix[j][k][i] = math.sin(input_matrix[i][1]) * math.sin(input_matrix[i][0])
                    elif k == 3:
                        # J_Matrix[j][k][i] = (input_matrix[i][3]) * math.cos(input_matrix[i][0])
                        J_Matrix[j][k][i] = (input_matrix[i][2]) * math.cos(input_matrix[i][0])
                if j == 1:
                    if k == 0:
                        J_Matrix[j][k][i] = math.sin(input_matrix[i][0])
                    elif k == 1:
                        J_Matrix[j][k][i] = math.cos(input_matrix[i][1]) * math.cos(input_matrix[i][0])
                    elif k == 2:
                        J_Matrix[j][k][i] = -1 * math.sin(input_matrix[i][1]) * math.cos(input_matrix[i][0])
                    elif k == 3:
                        # J_Matrix[j][k][i] = (input_matrix[i][3]) * math.sin(input_matrix[i][0])
                        J_Matrix[j][k][i] = (input_matrix[i][2]) * math.sin(input_matrix[i][0])
                if j == 2:
                    if k == 0:
                        J_Matrix[j][k][i] = 0
                    if k == 1:
                        J_Matrix[j][k][i] = math.sin(input_matrix[i][1])
                    if k == 2:
                        J_Matrix[j][k][i] = math.cos(input_matrix[i][1])
                    if k == 3:
                        # J_Matrix[j][k][i] = input_matrix[i][2]
                        J_Matrix[j][k][i] = input_matrix[i][3]
                if j == 3:
                    if k == 0:
                        J_Matrix[j][k][i] = 0
                    if k == 1:
                        J_Matrix[j][k][i] = 0
                    if k == 2:
                        J_Matrix[j][k][i] = 0
                    if k == 3:
                        J_Matrix[j][k][i] = 1
        J6_matrix = J_Matrix[:, :, 5]
        print("J6_matrix")
        print(J6_matrix)
        J6_inv = np.linalg.inv(J6_matrix)
        # J6_inv[2][3] = J6_inv[2][3] * -1
        print("J6 inverse")
        print(J6_inv)
        print("R0_6")
        print(R0_6)
        R0_5 = self.matrix_multiply(R0_6, J6_inv)
        # R0_5 = self.matrix_multiply(J6_inv, R0_6)
        print("R0_5")
        print(R0_5)

        input_matrix[0][0] = math.atan(R0_5[0][3]/R0_5[1][3])
        J5_xyz = [R0_5[0][3], R0_5[1][3], R0_5[2][3]]
        J1rads, J2rads, J3radsPlus90 = self.geometricInverse(allArms, J5_xyz)
        # input_matrix[0][0] = math.degrees(J1rads)
        # input_matrix[1][0] = math.degrees(J2rads)
        # input_matrix[2][0] = math.degrees(J3rads)
        input_matrix[0][0] = J1rads
        # input_matrix[1][0] = math.degrees(J2rads)
        input_matrix[1][0] = J2rads
        # input_matrix[2][0] = math.degrees(J3radsPlus90) - 90
        input_matrix[2][0] = J3radsPlus90 - math.radians(90)
        J1 = math.degrees(J1rads)
        J2 = math.degrees(J2rads)
        J3 = math.degrees(J3radsPlus90) - 90
        print("J1 = ", J1)
        print("J2 = ", J2)
        print("J3 = ", J3)

        # Fill in J1 through 3 for reverse kinematics
        for i in range(0, 3):
            for j in range(0, 4):
                for k in range(0, 4):
                    if j == 0:
                        if k == 0:
                            J_Matrix[j][k][i] = math.cos(input_matrix[i][0])
                        elif k == 1:
                            J_Matrix[j][k][i] = -math.cos(input_matrix[i][1]) * math.sin(input_matrix[i][0])
                        elif k == 2:
                            J_Matrix[j][k][i] = math.sin(input_matrix[i][1]) * math.sin(input_matrix[i][0])
                        elif k == 3:
                            # J_Matrix[j][k][i] = (input_matrix[i][3]) * math.cos(input_matrix[i][0])
                            J_Matrix[j][k][i] = (input_matrix[i][2]) * math.cos(input_matrix[i][0])
                    if j == 1:
                        if k == 0:
                            J_Matrix[j][k][i] = math.sin(input_matrix[i][0])
                        elif k == 1:
                            J_Matrix[j][k][i] = math.cos(input_matrix[i][1]) * math.cos(input_matrix[i][0])
                        elif k == 2:
                            J_Matrix[j][k][i] = -math.sin(input_matrix[i][1]) * math.cos(input_matrix[i][0])
                        elif k == 3:
                            # J_Matrix[j][k][i] = (input_matrix[i][3]) * math.sin(input_matrix[i][0])
                            J_Matrix[j][k][i] = (input_matrix[i][2]) * math.sin(input_matrix[i][0])
                    if j == 2:
                        if k == 0:
                            J_Matrix[j][k][i] = 0
                        if k == 1:
                            J_Matrix[j][k][i] = math.sin(input_matrix[i][1])
                        if k == 2:
                            J_Matrix[j][k][i] = math.cos(input_matrix[i][1])
                        if k == 3:
                            # J_Matrix[j][k][i] = input_matrix[i][2]
                            J_Matrix[j][k][i] = input_matrix[i][3]
                    if j == 3:
                        if k == 0:
                            J_Matrix[j][k][i] = 0
                        if k == 1:
                            J_Matrix[j][k][i] = 0
                        if k == 2:
                            J_Matrix[j][k][i] = 0
                        if k == 3:
                            J_Matrix[j][k][i] = 1

        for k in range(0, 4):
            if k == 0:
                J1_Matrix = J_Matrix[:, :, k]
            if k == 1:
                # R0_1 = self.matrix_multiply(work_frame, temp1)
                R0_1 = self.matrix_multiply(J1_Matrix, work_frame)
                # temp2 = self.flip_matrix(R0_1)
                J2_Matrix = J_Matrix[:, :, k]
            if k == 2:
                # R0_2 = self.matrix_multiply(temp2, J2_Matrix)
                # R0_2 = self.matrix_multiply(J2_Matrix, R0_1)
                R0_2 = self.matrix_multiply(R0_1, J2_Matrix)
                # R0_2 = self.matrix_multiply(J2_Matrix, temp2)
                # for i in range(0, 4):
                #     for j in range(0, 4):
                #         temp1[i][j] = J_Matrix[i][j][k]
                #         temp2[j][i] = R0_2[j][i]
                J3_Matrix = J_Matrix[:, :, k]
                # R0_3 = self.matrix_multiply(temp2, temp1)
                # R0_3 = self.matrix_multiply(J3_Matrix, R0_2)
                R0_3 = self.matrix_multiply(R0_2, J3_Matrix)
        print("R0_1")
        print(R0_1)
        print("R0_2")
        print(R0_2)
        print("R0_3")
        print(R0_3)
        print("J1 Matrix")
        print(J1_Matrix)
        print("J2 Matrix")
        print(J2_Matrix)
        R0_3_Inv = np.linalg.inv(R0_3)
        # R0_3_clippedInv = R0_3_Inv[:3, :3]
        R0_3_clipped = R0_3[:3, :3]
        R0_3_clippedInv = np.linalg.inv(R0_3_clipped)
        R0_5_clipped = R0_5[:3, :3]
        # R3_6 = self.matrix_multiply(R0_3_clippedInv, R0_5_clipped)
        # R3_6 = self.matrix_multiply(R0_5, R0_3_Inv)
        R3_6 = self.matrix_multiply(R0_3_Inv, R0_6)
        print("R3_6")
        print(R3_6)
        # R3_6 = self.matrix_multiply(R0_5_clipped, R0_3_clipped)
        # J5 = math.degrees(math.atan(R3_6[2][2] / (math.sqrt(1 - math.pow(R3_6[2][2], 2)))))
        # J4 = math.degrees(math.atan(R3_6[1][3] / R3_6[2][3]))
        # J6 = math.degrees(math.atan(-R3_6[3][1] / R3_6[3][2]))
        # J5 = math.degrees(math.atan2(R3_6[2][2], (math.sqrt(1 - math.pow(R3_6[2][2], 2)))))
        J5 = -1 * math.degrees(math.acos(R3_6[2][2]))
        # J6 = math.degrees(math.asin(R3_6[2][1] / math.sin(math.radians(J5))))
        # J4 = math.degrees(math.acos(R3_6[1][2] / math.sin(math.radians(J5))))
        J4 = math.degrees(math.atan2((R3_6[1][2] / math.sin(math.radians(J5))), (R3_6[0][2] / math.sin(math.radians(J5)))))
        J6 = math.degrees(math.atan2((R3_6[2][1] / math.sin(math.radians(J5))), (-1 * (R3_6[2][0] / math.sin(math.radians(J5))))))

        # J4 = math.degrees(math.atan2(R3_6[0][2], R3_6[1][2]))
        # J6 = math.degrees(math.atan2(-R3_6[2][0], R3_6[2][1]))
        J1 = math.degrees(J1rads)
        J2 = math.degrees(J2rads)
        # J3 = math.degrees(J3rads)
        joints = [J1, J2, J3, J4, J5, J6]
        print(joints)

    def geometricInverse(self, arms, pos):
        x = pos[0]
        y = pos[1]
        z = pos[2]
        print("pos = ", pos)
        a1 = arms[0]
        a2 = arms[1]
        # a3 = arms[3] + arms[4]
        a3 = arms[2] + arms[3]
        xy = math.pow((math.pow(x, 2) + math.pow(y, 2)), 0.5)
        H = math.pow((math.pow(xy, 2) + math.pow((z - a1), 2)), 0.5)
        # H = math.pow((math.pow(a3, 2) + math.pow(a2, 2)), 0.5)
        print("H = ", H)
        theta1, _, J3radPlus90 = self.lawOfCos(a3, a2, H)

        # J3 = math.acos((math.pow(a3, 2) + math.pow(a2, 2) + math.pow((math.pow(a3, 2) + math.pow(a2, 2)), 0.5))/(2 * a3 * a2))
        theta2 = math.atan((z - a1)/x)
        J2rad = theta2 + theta1
        J1rad = math.atan(y / x)

        return J1rad, J2rad, J3radPlus90

    def lawOfCos(self, a, b, c):
        print("a = ", a)
        print("b = ", b)
        print("c = ", c)
        val = ((math.pow(b, 2) + math.pow(c, 2) - math.pow(a, 2)) / (2 * b * c))
        val2 = math.acos(val)
        print((math.pow(b, 2) + math.pow(c, 2) - math.pow(a, 2)) / (2 * b * c))
        alpha = math.acos((math.pow(b, 2) + math.pow(c, 2) - math.pow(a, 2)) / (2 * b * c))
        beta = math.acos((math.pow(a, 2) + math.pow(c, 2) - math.pow(b, 2)) / (2 * a * c))
        gamma = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
        return alpha, beta, gamma

    def matrix_multiply(self, m1, m2):
        length1 = 0
        width1 = 0
        length2 = 0
        width2 = 0
        type = 0
        length1 = len(m1)
        width1 = len(m1[0])
        length2 = len(m2)
        try:
            width2 = len(m2[0])
            type = 1
        except:
            width2 = 1
            type = 2
            pass
        # print(solution)
        if type == 1:
            solution = np.zeros([length1, width1])
            for i in range(0, len(solution)):
                for j in range(0, len(solution)):
                    # for k in range(0, len(solution)):
                    for k in range(0, width2):
                        # solution[i][j] = (m1[i][j] * m2[k][j]) + solution[i][j]
                        # print(m1[i][k], ' + ', m2[k][j])
                        solution[i][j] = (m1[i][k] * m2[k][j]) + solution[i][j]
                        # solution[j][i] = (m1[k][i] * m2[j][k]) + solution[j][i]
                # print("")
        elif type == 2:
            solution = np.zeros([width2, length2])
            for i in range(0, len(solution)):
                for j in range(0, len(solution)):
                    # solution[i][j] = (m1[i][j] * m2[k][j]) + solution[i][j]
                    # print(m1[i][k], ' + ', m2[k][j])
                    solution[i][j] = (m1[i][j] * m2[j]) + solution[i][j]
                    # solution[j][i] = (m1[k][i] * m2[j][k]) + solution[j][i]
                # print("")
        return solution

    def flip_matrix(self, matrix):
        flipped = np.zeros([len(matrix[0]), len(matrix)])
        for i in range(len(flipped)):
            for j in range(len(flipped[0])):
                flipped[i][j] = matrix[j][i]
        return flipped


angles = [0, 60, 0, 0, -60, 0]
# cartesianCoordinates = [22.32, -0.0, 4.66, 90.0, 90.0, -90.0]
# cartesianCoordinates = [19.0, 0.0, 35.98, 0.0, -0.0, -0.0]
# cartesianCoordinates = [25.0, -0.0, 14.66, -180.0, -0.0, -0.0]
# cartesianCoordinates = [25.39, -0.0, 18.19, 180.0, -10.0, 0.0]
cartesianCoordinates = [22.32, 0.31, 4.66, 180.0, -0.0, 0.0]
for i in range(3, 5):
    cartesianCoordinates[i] = math.radians(cartesianCoordinates[i])
KinematicTest(cartesianCoordinates)