import math
import numpy as np
from numpy.linalg import inv



class Kinematics:
    def __init__(self, type, loc_input):
        self.aMatrix = [100, 200, 50, 50, 20, 20]
        if type == 1:
            #takes cartisian coordinates and oriantation and calculates joint angles
            print("Running inverse kinematics")
            # print("cartesian input = ", loc_input)
            Kinematics.joints = self.inverse_kinematics(loc_input)
            # print("joints = ", Kinematics.joints)
            # print("cartesian input again = ", loc_input)
        if type == 2:
            # takes joint angles and calculates cartisian coordinates and oriantation
            #print("About to Perform Forward Kinematics")
            #print("Kinematic Input = ", loc_input)
            print("Running forward kinematics")
            # print("joint input = ", loc_input)
            Kinematics.position, Kinematics.all_positions = self.forward_kinematics(loc_input, 3)
            for i in range(0, len(Kinematics.position)):
                Kinematics.position[i] = round(Kinematics.position[i],2)
                for j in range(0,2):
                    Kinematics.all_positions[i][j] = round(Kinematics.all_positions[i][j], 2)
            # print("Cartesian Coordinates = ", Kinematics.position)
        if type == 3:
            # takes movement velocities and calculates required joint angle velocities
            print("Number 3 here")

    def inverse_kinematics_old(self, reverse_input):
        input_matrix = [[0, math.radians(-90), 41.694, 0],
                       [0, 0, 0, 150],
                       [0, math.radians(90), 0, 0],
                       [0, math.radians(-90), -159.5, 0],
                       [0, math.radians(90), 0, 0],
                       [0, 0, -65, 0]]
        # a1 = 10
        # a2 = 10
        # a3 = 10
        # a4 = 10
        # a5 = 2
        # a6 = 2
        # input_matrix = [[0, math.radians(90), 0, a1],
        #                 [0, 0, a2, 0],
        #                 [0, math.radians(+90), 0, 0],
        #                 [0, math.radians(-90), 0, (a3 + a4)],
        #                 [0, math.radians(+90), 0, 0],
        #                 [0, 0, 0, (a5 + a6)]]
        #input_matrix = self.flip_matrix(input_matrix)
        pX = [0]*2
        pY = [0]*2
        pX_a1 = [0]*2
        pa2H = [0]*2
        pa3H = [0]*2
        Theta_A = [0]*2
        Theta_B = [0]*2
        Theta_C = [0]*2
        Theta_D = [0]*2
        Theta_E = [0]*2
        J2_Angle = [0]*2
        J3_Angle = [0]*2
        J4_Angle = [0]*2
        J5_Angle = [0]*2
        temp1 = np.zeros([4, 4])
        J_Matrix = np.zeros([4,4,6])

        R0_3_Trans_Small = np.zeros([3,3])
        temp_small = np.zeros([3,3])

        R0_T = np.zeros([4,4])
        R0_T[0][0] = ((math.cos(reverse_input[3])) * (math.cos(reverse_input[5]))) - (
                    math.cos(reverse_input[4]) * math.sin(reverse_input[3]) * math.sin(reverse_input[5]))
        R0_T[0][1] = ((math.sin(reverse_input[3])) * (math.cos(reverse_input[5]))) + (
                    math.cos(reverse_input[4]) * math.cos(reverse_input[3]) * math.sin(reverse_input[5]))
        R0_T[0][2] = ((math.sin(reverse_input[3])) * (math.sin(reverse_input[4])))
        R0_T[0][3] = reverse_input[0]
        R0_T[1][0] = ((math.cos(reverse_input[4])) * (math.cos(reverse_input[5])) * math.sin(reverse_input[3])) + (
                    math.cos(reverse_input[3]) * math.sin(reverse_input[5]))
        R0_T[1][1] = ((math.cos(reverse_input[4])) * (math.cos(reverse_input[5])) * math.cos(reverse_input[3])) - (
                    math.sin(reverse_input[3]) * math.sin(reverse_input[5]))
        R0_T[1][2] = -((math.cos(reverse_input[3])) * (math.sin(reverse_input[4])))
        R0_T[1][3] = reverse_input[1]
        R0_T[2][0] = ((math.sin(reverse_input[4])) * (math.sin(reverse_input[5])))
        R0_T[2][1] = ((math.cos(reverse_input[3])) * (math.sin(reverse_input[4])))
        R0_T[2][2] = -((math.cos(reverse_input[4])))
        R0_T[2][3] = reverse_input[2]
        R0_T[3][0] = 0
        R0_T[3][1] = 0
        R0_T[3][2] = 0
        R0_T[3][3] = 1
        #print("R0_T")
        #print(R0_T)

        work_frame = np.zeros([4,4])
        for i in range(len(work_frame)):
            work_frame[i,i] = 1

        R0_T_Work = self.matrix_multiply(R0_T, work_frame)
        R0_6 = self.matrix_multiply(R0_T_Work, work_frame)
        R0_6[0][0] = R0_6[0][0] * -1
        #print("R0_6")
        #print(R0_6)

        R0_6_Remove = np.zeros([4,4])
        R0_3_Trans = np.zeros([4,4])
        #R0_3_Trans_Small = np.zeros([4,4])

        R0_6_Remove[0][0] = math.cos(math.radians(180))
        R0_6_Remove[0][1] = math.sin(math.radians(180))
        R0_6_Remove[1][0] = -math.sin(math.radians(180)) * math.cos(input_matrix[5][1])
        R0_6_Remove[1][1] = math.cos(math.radians(180)) * math.cos(input_matrix[5][1])
        R0_6_Remove[2][2] = math.cos(input_matrix[5][1])
        R0_6_Remove[2][3] = -input_matrix[5][2]
        R0_6_Remove[3][3] = 1
        #print("R0_6_Remove")
        #print(R0_6_Remove)
        #R0_6 = self.flip_matrix(R0_6)
        R0_6_Remove = self.flip_matrix(R0_6_Remove)
        R0_5 = self.matrix_multiply(R0_6, R0_6_Remove)
        #R0_5 = self.flip_matrix(R0_5)
        #print("R0_5")
        #print(R0_5)
        #R0_5 = self.flip_matrix(R0_5)

        J1_Angle = math.degrees(math.atan(R0_5[1][3] / R0_5[0][3]))
        pX[0] = math.sqrt(pow(abs(R0_5[0][3]), 2) + pow(abs(R0_5[1][3]), 2))
        pY[0] = R0_5[2][3] - input_matrix[0][2]
        pX_a1[0] = pX[0] - input_matrix[0][3]
        pX_a1[1] = -1 * pX_a1[0]
        pa2H[1] = math.sqrt(pow(pX_a1[1], 2) + pow(pY[0], 2))
        Theta_B[1] = math.degrees(math.atan(pX_a1[1] / pY[0]))
        #print(((pow(input_matrix[1][3], 2) + pow(pa2H[1], 2) -
        #                                     pow(abs(input_matrix[3][2]), 2)) / (2 * input_matrix[1][3] * pa2H[1])))
        #Theta_A[1] = math.degrees(math.acos((pow(input_matrix[1][3], 2) + pow(pa2H[1], 2) -
        #                                     pow(abs(input_matrix[3][2]), 2)) / (2 * input_matrix[1][3] * pa2H[1])))
        inbetween = (((pow(input_matrix[1][3], 2) + pow(pa2H[1], 2) -
                                             pow(abs(input_matrix[3][2]), 2)) / (2 * input_matrix[1][3] * pa2H[1])))

        #print(math.acos(54))
        Theta_A[1] = math.degrees(math.acos(inbetween))
        #print(Theta_A[1])
        Theta_D[1] = 90 - (Theta_A[1] + Theta_B[1])
        pa2H[0] = math.sqrt(pow(pY[0], 2) + pow(pX_a1[0], 2))
        pa3H[0] = math.sqrt(pow(input_matrix[3][2], 2) + pow(input_matrix[2][3], 2))
        Theta_A[0] = math.degrees(math.atan(pX[0] / pY[0]))
        Theta_B[0] = math.degrees(math.acos((pow(input_matrix[1][3], 2) + pow(pa2H[0], 2) -
                                             pow(abs(pa3H[0]), 2)) / (2 * input_matrix[1][3] * pa2H[0])))

        if input_matrix[2][3] == 0:
            Theta_E[0] = 90
        else:
            Theta_E[0] = math.degrees(math.atan(abs(input_matrix[3][2]) / input_matrix[2][3]))
        Theta_C[0] = 180 - math.degrees(math.acos((pow(pa3H[0], 2) + pow(input_matrix[1][3], 2) -
                                       pow(abs(pa2H[0]), 2)) / (2 * input_matrix[1][3] * pa3H[0]))) + (90 - Theta_E[0])
        Theta_C[1] = Theta_C[0]

        if pX_a1[0] < 0:
            J2_Angle[0] = (Theta_A[0] + Theta_B[0]) * -1
        else:
            J2_Angle[0] = -180 + Theta_D[1]

        J3_Angle[0] = Theta_C[0]

        input_matrix[0][0] = math.radians(J1_Angle)
        input_matrix[1][0] = math.radians(J2_Angle[0])
        input_matrix[2][0] = math.radians(J3_Angle[0] - 90)

        # Fill in J1 through 3 for reverse kinematics
        for i in range(0,2):
            for j in range(0,3):
                for k in range(0,3):
                    if j == 0:
                        if k == 0:
                            J_Matrix[j][k][i] = math.cos(input_matrix[i][0])
                        elif k == 1:
                            J_Matrix[j][k][i] = -math.cos(input_matrix[i][1]) * math.sin(input_matrix[i][0])
                        elif k == 2:
                            J_Matrix[j][k][i] = math.sin(input_matrix[i][1]) * math.sin(input_matrix[i][0])
                        elif k == 3:
                            J_Matrix[j][k][i] = (input_matrix[i][3]) * math.cos(input_matrix[i][0])
                    if j == 1:
                        if k == 0:
                            J_Matrix[j][k][i] = math.sin(input_matrix[i][0])
                        elif k == 1:
                            J_Matrix[j][k][i] = math.cos(input_matrix[i][1]) * math.cos(input_matrix[i][0])
                        elif k == 2:
                            J_Matrix[j][k][i] = -math.sin(input_matrix[i][1]) * math.cos(input_matrix[i][0])
                        elif k == 3:
                            J_Matrix[j][k][i] = (input_matrix[i][3]) * math.sin(input_matrix[i][0])
                    if j == 2:
                        if k == 0:
                            J_Matrix[j][k][i] = 0
                        if k == 1:
                            J_Matrix[j][k][i] = math.sin(input_matrix[i][1])
                        if k == 2:
                            J_Matrix[j][k][i] = math.cos(input_matrix[i][1])
                        if k == 3:
                            J_Matrix[j][k][i] = input_matrix[i][2]
                    if j == 3:
                        if k == 0:
                            J_Matrix[j][k][i] = 0
                        if k == 1:
                            J_Matrix[j][k][i] = 0
                        if k == 2:
                            J_Matrix[j][k][i] = 0
                        if k == 3:
                            J_Matrix[j][k][i] = 1

        for k in range(0,3):
            if k == 0:
                for i in range(0,3):
                    for j in range(0,3):
                        temp1[j][i] = J_Matrix[i][j][k]
            if k == 1:
                R0_1 = self.matrix_multiply(work_frame, temp1)
                temp2 = self.flip_matrix(R0_1)
                for i in range(0,3):
                    for j in range(0,3):
                        temp1[i][j] = J_Matrix[i][j][k]
            if k == 2:
                R0_2 = self.matrix_multiply(temp2, temp1)
                for i in range(0,3):
                    for j in range(0,3):
                        temp1[i][j] = J_Matrix[i][j][k]
                        temp2[j][i] = R0_2[j][i]
                R0_3 = self.matrix_multiply(temp2, temp1)

        R0_3_Trans = self.flip_matrix(R0_3)

        for i in range(0,2):
            for j in range(0,2):
                R0_3_Trans_Small[i][j] = R0_3_Trans[i][j]
                temp_small[i][j] = R0_5[i][j]

        J4_Angle[0] = math.degrees(math.atan2(R0_3_Trans[1][2], R0_3_Trans[0][2]))
        #print(len(R0_3_Trans_Small))
        #print(len(R0_3_Trans_Small[0]))
        #print(R0_3_Trans_Small)
        R3_6 = self.matrix_multiply(R0_3_Trans_Small, temp_small)
        J5_Angle[0] = math.degrees(math.atan2((math.sqrt(1 - pow(R3_6[2][2], 2))), R3_6[2][2]))

        # print("J1 = ", J1_Angle)
        # print("J2 = ", J2_Angle)
        # print("J3 = ", J3_Angle)
        # print("J4 = ", J4_Angle)
        # print("J5 = ", J5_Angle)

        # BOTH OF THESE IF STATEMENTS WILL CHANGE BASED OFF MOTOR MOUNTING
        # if reverse_input[2] < -24:
        #     J1_Angle = J1_Angle + 50;
        #     J2_Angle[0] = -1 * (180 + J2_Angle[0]);
        #     J3_Angle[0] = J3_Angle[0] + 90;
        #     J4_Angle[0] = 105;
        #     J5_Angle[0] = (90 - (J5_Angle[0] - 90));
        # if reverse_input[2] > -24:
        #     J1_Angle = J1_Angle + 50;
        #     J2_Angle[0] = -1 * (J2_Angle[0]);
        #     J3_Angle[0] = J3_Angle[0] + 90;
        #     J4_Angle[0] = 105;
        #     J5_Angle[0] = (90 - J5_Angle[0]);

        j_angles = [J1_Angle, J2_Angle[0], J3_Angle[0], J4_Angle[0], J5_Angle[0], 0]
        return j_angles

    def inverse_kinematics(self, reverse_input):
        # a1 = 10
        # a2 = 10
        # a3 = 10
        # a4 = 10
        # a5 = 2
        # a6 = 2

        a1 = self.aMatrix[0]
        a2 = self.aMatrix[1]
        a3 = self.aMatrix[2]
        a4 = self.aMatrix[3]
        a5 = self.aMatrix[4]
        a6 = self.aMatrix[5]

        input_matrix = [[0, math.radians(90), 0, a1],
                        [0, 0, a2, 0],
                        [0, math.radians(+90), 0, 0],
                        [0, math.radians(-90), 0, (a3 + a4)],
                        [0, math.radians(+90), 0, 0],
                        [0, 0, 0, (a5 + a6)]]


        J_Matrix = np.zeros([4, 4, 6])
        allArms = [a1, a2, a3, a4, a5, a6]
        cartesianXYZ = [reverse_input[0], reverse_input[1], reverse_input[2]]
        R0_T = np.zeros([4, 4])
        for i in range(3, 5):
            reverse_input[i] = math.radians(reverse_input[i])
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

        # print("R0_T")
        # print(R0_T)
        R0_3_Trans_Small = np.zeros([3, 3])
        temp_small = np.zeros([3, 3])
        work_frame = np.zeros([4, 4])
        tool_frame = np.zeros([4, 4])
        for i in range(len(work_frame)):
            work_frame[i, i] = 1
            tool_frame[i, i] = 1

        R0_T_workFrame = self.matrix_multiply(R0_T, work_frame)
        work_frame_inverse = np.linalg.inv(work_frame)
        R0_6 = self.matrix_multiply(R0_T_workFrame, work_frame_inverse)
        input_matrix[5][0] = math.radians(-180)
        i = 5
        # fill J6 matrix
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
        # print("J6_matrix")
        # print(J6_matrix)
        J6_inv = np.linalg.inv(J6_matrix)
        # J6_inv[2][3] = J6_inv[2][3] * -1
        # print("J6 inverse")
        # print(J6_inv)
        # print("R0_6")
        # print(R0_6)
        R0_5 = self.matrix_multiply(R0_6, J6_inv)
        # R0_5 = self.matrix_multiply(J6_inv, R0_6)
        # print("R0_5")
        # print(R0_5)

        input_matrix[0][0] = math.atan(R0_5[0][3] / R0_5[1][3])
        J5_xyz = [R0_5[0][3], R0_5[1][3], R0_5[2][3]]
        J1rads, J2rads, J3radsPlus90 = self.geometricInverse(allArms, J5_xyz)
        input_matrix[0][0] = J1rads
        input_matrix[1][0] = J2rads
        input_matrix[2][0] = J3radsPlus90 - math.radians(90)
        J1 = math.degrees(J1rads)
        J2 = math.degrees(J2rads)
        J3 = math.degrees(J3radsPlus90) - 90
        # print("J1 = ", J1)
        # print("J2 = ", J2)
        # print("J3 = ", J3)

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
                R0_1 = self.matrix_multiply(J1_Matrix, work_frame)
                J2_Matrix = J_Matrix[:, :, k]
            if k == 2:
                R0_2 = self.matrix_multiply(R0_1, J2_Matrix)
                J3_Matrix = J_Matrix[:, :, k]
                R0_3 = self.matrix_multiply(R0_2, J3_Matrix)
        R0_3_Inv = np.linalg.inv(R0_3)
        R3_6 = self.matrix_multiply(R0_3_Inv, R0_6)
        J5 = -1 * math.degrees(math.acos(R3_6[2][2]))
        J4 = math.degrees(
            math.atan2((R3_6[1][2] / math.sin(math.radians(J5))), (R3_6[0][2] / math.sin(math.radians(J5)))))
        J6 = math.degrees(
            math.atan2((R3_6[2][1] / math.sin(math.radians(J5))), (-1 * (R3_6[2][0] / math.sin(math.radians(J5))))))
        J1 = math.degrees(J1rads)
        J2 = math.degrees(J2rads)
        joints = [J1, J2, J3, J4, J5, J6]
        for i in range(3, 5):
            reverse_input[i] = math.degrees(reverse_input[i])
        # print("joints = ", joints)
        return joints

    def geometricInverse(self, arms, pos):
        x = pos[0]
        y = pos[1]
        z = pos[2]
        # print("pos = ", pos)
        a1 = arms[0]
        a2 = arms[1]
        # a3 = arms[3] + arms[4]
        a3 = arms[2] + arms[3]
        xy = math.pow((math.pow(x, 2) + math.pow(y, 2)), 0.5)
        H = math.pow((math.pow(xy, 2) + math.pow((z - a1), 2)), 0.5)
        # H = math.pow((math.pow(a3, 2) + math.pow(a2, 2)), 0.5)
        theta1, _, J3radPlus90 = self.lawOfCos(a3, a2, H)

        # J3 = math.acos((math.pow(a3, 2) + math.pow(a2, 2) + math.pow((math.pow(a3, 2) + math.pow(a2, 2)), 0.5))/(2 * a3 * a2))
        theta2 = math.atan((z - a1) / x)
        J2rad = theta2 + theta1
        J1rad = math.atan(y / x)

        return J1rad, J2rad, J3radPlus90

    def lawOfCos(self, a, b, c):
        # print("a = ", a)
        # print("b = ", b)
        # print("c = ", c)
        val = ((math.pow(b, 2) + math.pow(c, 2) - math.pow(a, 2)) / (2 * b * c))
        val2 = math.acos(val)
        # print((math.pow(b, 2) + math.pow(c, 2) - math.pow(a, 2)) / (2 * b * c))
        alpha = math.acos((math.pow(b, 2) + math.pow(c, 2) - math.pow(a, 2)) / (2 * b * c))
        beta = math.acos((math.pow(a, 2) + math.pow(c, 2) - math.pow(b, 2)) / (2 * a * c))
        gamma = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
        return alpha, beta, gamma

    def forward_kinematics(self, forward_input, back_type):
        # a1 = 10
        # a2 = 10
        # a3 = 10
        # a4 = 10
        # a5 = 2
        # a6 = 2

        a1 = self.aMatrix[0]
        a2 = self.aMatrix[1]
        a3 = self.aMatrix[2]
        a4 = self.aMatrix[3]
        a5 = self.aMatrix[4]
        a6 = self.aMatrix[5]

        # input_matrix = [[0, math.radians(90), 0, (a1)],
        #                        [0, 0, (a2), 0],
        #                        [0, math.radians(-90), 0, 0],
        #                        [0, math.radians(90), (a3+a4), 0],
        #                        [0, math.radians(-90), 0, 0],
        #                        [0, 0, (a5+a6), 0]]
        input_matrix = [[0, math.radians(90), 0, a1],
                        [0, 0, a2, 0],
                        [0, math.radians(+90), 0, 0],
                        [0, math.radians(-90), 0, (a3 + a4)],
                        [0, math.radians(+90), 0, 0],
                        [0, 0, 0, (a5 + a6)]]


        final_angle = np.zeros([6])
        work_frame = np.zeros([4, 4])
        tool_frame = np.zeros([4, 4])

        for i in range(len(work_frame)):
            work_frame[i, i] = 1
            tool_frame[i, i] = 1
        for i in range(len(forward_input)):
            input_matrix[i][0] = math.radians(forward_input[i])

        J1_matrix = np.zeros([4, 4])
        J2_matrix = np.zeros([4, 4])
        J3_matrix = np.zeros([4, 4])
        J4_matrix = np.zeros([4, 4])
        J5_matrix = np.zeros([4, 4])
        J6_matrix = np.zeros([4, 4])
        for i in range(4):
            for j in range(4):
                if i == 0:
                    if j == 0:
                        J1_matrix[i][j] = math.cos(input_matrix[0][0])
                        J2_matrix[i][j] = math.cos(input_matrix[1][0])
                        J3_matrix[i][j] = math.cos(input_matrix[2][0])
                        J4_matrix[i][j] = math.cos(input_matrix[3][0])
                        J5_matrix[i][j] = math.cos(input_matrix[4][0])
                        J6_matrix[i][j] = math.cos(input_matrix[5][0])
                    if j == 1:
                        J1_matrix[i][j] = -1 * math.sin(input_matrix[0][0]) * math.cos(input_matrix[0][1])
                        J2_matrix[i][j] = -1 * math.sin(input_matrix[1][0]) * math.cos(input_matrix[1][1])
                        J3_matrix[i][j] = -1 * math.sin(input_matrix[2][0]) * math.cos(input_matrix[2][1])
                        J4_matrix[i][j] = -1 * math.sin(input_matrix[3][0]) * math.cos(input_matrix[3][1])
                        J5_matrix[i][j] = -1 * math.sin(input_matrix[4][0]) * math.cos(input_matrix[4][1])
                        J6_matrix[i][j] = -1 * math.sin(input_matrix[5][0]) * math.cos(input_matrix[5][1])
                    if j == 2:
                        J1_matrix[i][j] = math.sin(input_matrix[0][0]) * math.sin(input_matrix[0][1])
                        J2_matrix[i][j] = math.sin(input_matrix[1][0]) * math.sin(input_matrix[1][1])
                        J3_matrix[i][j] = math.sin(input_matrix[2][0]) * math.sin(input_matrix[2][1])
                        J4_matrix[i][j] = math.sin(input_matrix[3][0]) * math.sin(input_matrix[3][1])
                        J5_matrix[i][j] = math.sin(input_matrix[4][0]) * math.sin(input_matrix[4][1])
                        J6_matrix[i][j] = math.sin(input_matrix[5][0]) * math.sin(input_matrix[5][1])
                    if j == 3:
                        J1_matrix[i][j] = input_matrix[0][2] * math.cos(input_matrix[0][0])
                        J2_matrix[i][j] = input_matrix[1][2] * math.cos(input_matrix[1][0])
                        J3_matrix[i][j] = input_matrix[2][2] * math.cos(input_matrix[2][0])
                        J4_matrix[i][j] = input_matrix[3][2] * math.cos(input_matrix[3][0])
                        J5_matrix[i][j] = input_matrix[4][2] * math.cos(input_matrix[4][0])
                        J6_matrix[i][j] = input_matrix[5][2] * math.cos(input_matrix[5][0])
                if i == 1:
                    if j == 0:
                        J1_matrix[i][j] = math.sin(input_matrix[0][0])
                        J2_matrix[i][j] = math.sin(input_matrix[1][0])
                        J3_matrix[i][j] = math.sin(input_matrix[2][0])
                        J4_matrix[i][j] = math.sin(input_matrix[3][0])
                        J5_matrix[i][j] = math.sin(input_matrix[4][0])
                        J6_matrix[i][j] = math.sin(input_matrix[5][0])
                    if j == 1:
                        J1_matrix[i][j] = math.cos(input_matrix[0][0]) * math.cos(input_matrix[0][1])
                        J2_matrix[i][j] = math.cos(input_matrix[1][0]) * math.cos(input_matrix[1][1])
                        J3_matrix[i][j] = math.cos(input_matrix[2][0]) * math.cos(input_matrix[2][1])
                        J4_matrix[i][j] = math.cos(input_matrix[3][0]) * math.cos(input_matrix[3][1])
                        J5_matrix[i][j] = math.cos(input_matrix[4][0]) * math.cos(input_matrix[4][1])
                        J6_matrix[i][j] = math.cos(input_matrix[5][0]) * math.cos(input_matrix[5][1])
                    if j == 2:
                        J1_matrix[i][j] = -1 * math.cos(input_matrix[0][0]) * math.sin(input_matrix[0][1])
                        J2_matrix[i][j] = -1 * math.cos(input_matrix[1][0]) * math.sin(input_matrix[1][1])
                        J3_matrix[i][j] = -1 * math.cos(input_matrix[2][0]) * math.sin(input_matrix[2][1])
                        J4_matrix[i][j] = -1 * math.cos(input_matrix[3][0]) * math.sin(input_matrix[3][1])
                        J5_matrix[i][j] = -1 * math.cos(input_matrix[4][0]) * math.sin(input_matrix[4][1])
                        J6_matrix[i][j] = -1 * math.cos(input_matrix[5][0]) * math.sin(input_matrix[5][1])
                    if j == 3:
                        J1_matrix[i][j] = input_matrix[0][2] * math.sin(input_matrix[0][0])
                        J2_matrix[i][j] = input_matrix[1][2] * math.sin(input_matrix[1][0])
                        J3_matrix[i][j] = input_matrix[2][2] * math.sin(input_matrix[2][0])
                        J4_matrix[i][j] = input_matrix[3][2] * math.sin(input_matrix[3][0])
                        J5_matrix[i][j] = input_matrix[4][2] * math.sin(input_matrix[4][0])
                        J6_matrix[i][j] = input_matrix[5][2] * math.sin(input_matrix[5][0])
                if i == 2:
                    if j == 0:
                        J1_matrix[i][j] = 0
                        J2_matrix[i][j] = 0
                        J3_matrix[i][j] = 0
                        J4_matrix[i][j] = 0
                        J5_matrix[i][j] = 0
                        J6_matrix[i][j] = 0
                    if j == 1:
                        J1_matrix[i][j] = math.sin(input_matrix[0][1])
                        J2_matrix[i][j] = math.sin(input_matrix[1][1])
                        J3_matrix[i][j] = math.sin(input_matrix[2][1])
                        J4_matrix[i][j] = math.sin(input_matrix[3][1])
                        J5_matrix[i][j] = math.sin(input_matrix[4][1])
                        J6_matrix[i][j] = math.sin(input_matrix[5][1])
                    if j == 2:
                        J1_matrix[i][j] = math.cos(input_matrix[0][1])
                        J2_matrix[i][j] = math.cos(input_matrix[1][1])
                        J3_matrix[i][j] = math.cos(input_matrix[2][1])
                        J4_matrix[i][j] = math.cos(input_matrix[3][1])
                        J5_matrix[i][j] = math.cos(input_matrix[4][1])
                        J6_matrix[i][j] = math.cos(input_matrix[5][1])
                    if j == 3:
                        J1_matrix[i][j] = (input_matrix[0][3])
                        J2_matrix[i][j] = (input_matrix[1][3])
                        J3_matrix[i][j] = (input_matrix[2][3])
                        J4_matrix[i][j] = (input_matrix[3][3])
                        J5_matrix[i][j] = (input_matrix[4][3])
                        J6_matrix[i][j] = (input_matrix[5][3])
                if i == 3:
                    if j == 0:
                        J1_matrix[i][j] = 0
                        J2_matrix[i][j] = 0
                        J3_matrix[i][j] = 0
                        J4_matrix[i][j] = 0
                        J5_matrix[i][j] = 0
                        J6_matrix[i][j] = 0
                    if j == 1:
                        J1_matrix[i][j] = 0
                        J2_matrix[i][j] = 0
                        J3_matrix[i][j] = 0
                        J4_matrix[i][j] = 0
                        J5_matrix[i][j] = 0
                        J6_matrix[i][j] = 0
                    if j == 2:
                        J1_matrix[i][j] = 0
                        J2_matrix[i][j] = 0
                        J3_matrix[i][j] = 0
                        J4_matrix[i][j] = 0
                        J5_matrix[i][j] = 0
                        J6_matrix[i][j] = 0
                    if j == 3:
                        J1_matrix[i][j] = 1
                        J2_matrix[i][j] = 1
                        J3_matrix[i][j] = 1
                        J4_matrix[i][j] = 1
                        J5_matrix[i][j] = 1
                        J6_matrix[i][j] = 1

        H0_2 = self.matrix_multiply(J1_matrix, J2_matrix)
        # H0_2 = self.matrix_multiply(J2_matrix, J1_matrix)
        H0_3 = self.matrix_multiply(H0_2, J3_matrix)
        H0_4 = self.matrix_multiply(H0_3, J4_matrix)
        H0_5 = self.matrix_multiply(H0_4, J5_matrix)
        H0_6 = self.matrix_multiply(H0_5, J6_matrix)
        H0_3_Inv = np.linalg.inv(H0_3)
        H0_3_flip = np.zeros([4, 4])
        for i in range(0, 4):
            for j in range(0, 4):
                H0_3_flip[i][j] = H0_3[j][i]
        # H3_6 = self.matrix_multiply(H0_3_Inv, H0_6)
        # H3_6 = self.matrix_multiply(H0_6, H0_3_Inv)
        H3_6 = self.matrix_multiply(H0_3_flip, H0_6)
        # H3_6 = self.matrix_multiply(H0_6, H0_3_flip)

        # print("H0_5")
        # print(H0_5)
        #
        # print("H0_6")
        # print(H0_6)
        #
        # print("J6")
        # print(J6_matrix)
        #
        # print("H3_6")
        # print(H3_6)
        # JJ5 = math.degrees(math.atan2(H3_6[2][2], -1 * math.pow((1 - H3_6[2][2]), 0.5)))
        JJ5 = math.degrees(math.acos(H3_6[2][2]))
        JJ4 = math.degrees(math.atan2((H3_6[1][2] / math.sin(math.radians(JJ5))), (H3_6[0][2] / math.sin(math.radians(JJ5)))))
        JJ6 = math.degrees(math.atan2((H3_6[2][1] / math.sin(math.radians(JJ5))), (-1 * (H3_6[2][0] / math.sin(math.radians(JJ5))))))
        # JJ6 = math.degrees(math.asin(H3_6[2][1] / math.sin(math.radians(JJ5))))
        # JJ4 = math.degrees(math.acos(H3_6[1][2] / math.sin(math.radians(JJ5))))
        # print("JJ4 = ", JJ4)
        # print("JJ5 = ", JJ5)
        # print("JJ6 = ", JJ6)
        #
        # print("J1")
        # print(J1_matrix)
        #
        # print("J2")
        # print(J2_matrix)
        #
        # print("H0_2")
        # print(H0_2)
        #
        # print("H0_3")
        # print(H0_3)

        x = H0_6[0][3]
        y = H0_6[1][3]
        z = H0_6[2][3]

        x_array = [0, J1_matrix[0][3], H0_2[0][3], H0_3[0][3], H0_4[0][3], H0_5[0][3], H0_6[0][3]]  # , J2_matrix[0][3]
        y_array = [0, J1_matrix[1][3], H0_2[1][3], H0_3[1][3], H0_4[1][3], H0_5[1][3], H0_6[1][3]]  # , J2_matrix[1][3]
        z_array = [0, J1_matrix[2][3], H0_2[2][3], H0_3[2][3], H0_4[2][3], H0_5[2][3], H0_6[2][3]]  # , J2_matrix[2][3]

        #x_array = [0, J1_matrix[3][0], H0_2[3][0], H0_3[3][0], H0_4[3][0], H0_5[3][0], H0_6[3][0]]  # , J2_matrix[0][3]
        #y_array = [0, J1_matrix[3][1], H0_2[3][1], H0_3[3][1], H0_4[3][1], H0_5[3][1], H0_6[3][1]]  # , J2_matrix[1][3]
        #z_array = [0, J1_matrix[3][2], H0_2[3][2], H0_3[3][2], H0_4[3][2], H0_5[3][2], H0_6[3][2]]  # , J2_matrix[2][3]

        total_position_matrix = np.zeros([7, 3])
        for i in range(0, 7):
            total_position_matrix[i][0] = x_array[i]
            total_position_matrix[i][1] = y_array[i]
            total_position_matrix[i][2] = z_array[i]

        # # rot_y = math.degrees(math.atan2((-1 * H0_6[2][2]), math.sqrt(math.pow(H0_6[1][2], 2)
        # #                                                              + math.pow(H0_6[0][2], 2))))
        # rot_y = math.degrees(math.atan2((-1 * H0_6[2][1]), math.sqrt(math.pow(H0_6[1][0], 2)
        #                                                              + math.pow(H0_6[0][0], 2))))
        # # rot_x = math.degrees(math.atan2((H0_6[2][1] / math.cos(math.radians(rot_y))),
        # #                                 (H0_6[2][0] / math.cos(math.radians(rot_y)))))
        # rot_x = math.degrees(math.atan2((H0_6[2][1] / math.cos(math.radians(rot_y))),
        #                                 (H0_6[2][2] / math.cos(math.radians(rot_y)))))
        # # rot_z = math.degrees(math.atan2((H0_6[1][2] / math.cos(math.radians(rot_y))),
        # #                                 (H0_6[0][2] / math.cos(math.radians(rot_y)))))
        # rot_z = math.degrees(math.atan2((H0_6[1][0] / math.cos(math.radians(rot_y))),
        #                                 (H0_6[0][0] / math.cos(math.radians(rot_y)))))

        rot_y = math.degrees(math.atan2((-1 * H0_6[2][0]), math.sqrt(math.pow(H0_6[2][1], 2)
                                                                     + math.pow(H0_6[2][2], 2))))
        rot_x = math.degrees(math.atan2((H0_6[2][1]), (H0_6[2][2])))
        rot_z = math.degrees(math.atan2((H0_6[1][0]), (H0_6[0][0])))


        final_positions = [x, y, z, rot_x, rot_y, rot_z]
        # print('x = ', x)
        # print('y = ', y)
        # print('z = ', z)
        # print('w_x = ', rot_x)
        # print('w_y = ', rot_y)
        # print('w_z = ', rot_z)

        if back_type == 1:
            return H0_6, x_array, y_array, z_array
        if back_type == 2:
            return H0_6
        if back_type == 3:
            return final_positions, total_position_matrix

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

    def jacobian(self, joint_input, velocity_matrix):
        # units of
        # joint input units = degrees
        # joint_input = [90,45,-45,0,-90,0]
        # velocity matrix units = mm/s
        # velocity_matrix = [0,0,-10,0,0,0]
        # print('home matrix')
        home_matrix, x_positions, y_positions, z_positions = self.forward_kinematics(joint_input, 1)
        # print("Home Matrix")
        # print(home_matrix)
        delta = 0.5  # degrees
        jacobian_matrix = np.zeros([6, 6])
        for i in range(0, 6):
            partial_derivative_matrix = np.zeros([4, 4])
            # print(partial_derivative_matrix)
            joint_input[i] = joint_input[i] + delta
            # print('joint = ', (i+1))
            # print('len(home_matrix) = ', len(home_matrix))
            delta_matrix = self.forward_kinematics(joint_input, 2)
            # print('delta_matrix = ')
            # print(delta_matrix)
            for j in range(0, len(home_matrix)):
                for k in range(0, len(home_matrix[0])):
                    # temp = (home_matrix[j][k] - delta_matrix[j][k])
                    # partial_derivative_matrix[j][k] = (temp / delta)
                    partial_derivative_matrix[j][k] = (
                                (home_matrix[j][k] - delta_matrix[j][k]) / delta)  # mm/rad or rad/rad
            rotation_matrix_transpose = np.zeros([3, 3])
            rotation_matrix_derivative = np.zeros([3, 3])
            rotation_matrix_home = np.zeros([3, 3])
            for j in range(0, 3):
                for k in range(0, 3):
                    rotation_matrix_transpose[j][k] = home_matrix[k][j]  # rad
                    rotation_matrix_home[j][k] = home_matrix[j][k]  # rad
                    rotation_matrix_derivative[j][k] = partial_derivative_matrix[j][k]  # rad/rad
                    # rotation_matrix_transpose[j][k] = home_matrix[j][k]
                    # rotation_matrix_derivative[j][k] = partial_derivative_matrix[k][j]
            # skew_symetric_matrix = matrix_multiply(rotation_matrix_derivative, rotation_matrix_transpose)
            test = self.matrix_multiply(rotation_matrix_transpose, rotation_matrix_home)
            # print('test')
            # print(test)
            skew_symetric_matrix = self.matrix_multiply(rotation_matrix_derivative, rotation_matrix_transpose)  # rad
            # skew_symetric_matrix = np.dot(rotation_matrix_derivative, rotation_matrix_transpose)
            # if i == 5:
            # print('partial_derivative_matrix = ')
            # print(partial_derivative_matrix)
            # print('skew_symetric_matrix = ')
            # print(skew_symetric_matrix)

            # This was the original, I am going to try flipping it
            # jacobian_matrix[0][i] = partial_derivative_matrix[0][3]
            # jacobian_matrix[1][i] = partial_derivative_matrix[1][3]
            # jacobian_matrix[2][i] = partial_derivative_matrix[2][3]
            # jacobian_matrix[3][i] = skew_symetric_matrix[1][2]
            # jacobian_matrix[4][i] = -1 * skew_symetric_matrix[0][2]
            # jacobian_matrix[5][i] = skew_symetric_matrix[0][1]

            # This is the flipped version
            jacobian_matrix[i][0] = partial_derivative_matrix[0][3]  # mm/rad
            jacobian_matrix[i][1] = partial_derivative_matrix[1][3]  # mm/rad
            jacobian_matrix[i][2] = partial_derivative_matrix[2][3]  # mm/rad
            # skew symetric could be wrong, below is the original
            jacobian_matrix[i][3] = skew_symetric_matrix[1][2]  # rad
            jacobian_matrix[i][4] = -1 * skew_symetric_matrix[0][2]  # rad
            jacobian_matrix[i][5] = skew_symetric_matrix[0][1]  # rad

            # jacobian_matrix[i][5] = skew_symetric_matrix[1][2]
            # jacobian_matrix[i][3] = -1 * skew_symetric_matrix[0][2]
            # jacobian_matrix[i][4] = skew_symetric_matrix[0][1]
            # print('joint = ', i+1)
            # print('Vx = ', partial_derivative_matrix[0][3])
            # print('Vy = ', partial_derivative_matrix[1][3])
            # print('Vz = ', partial_derivative_matrix[2][3])
            # print('Wx = ', skew_symetric_matrix[1][2])
            # print('Vy = ', -1*skew_symetric_matrix[0][2])
            # print('Vz = ', skew_symetric_matrix[0][1])
            # print('skew_symetric_matrix')
            # print(skew_symetric_matrix)
            joint_input[i] = joint_input[i] - delta

        # print('jacobian = ')
        # print(jacobian_matrix)
        # print('velocity_matrix')
        # print(velocity_matrix)
        # jacobian in mm/rad (translation) and rad (orientation)
        # velocity in mm/sec (translation) and rad/sec (orientation)
        joint_velocity_matrix = self.matrix_multiply(jacobian_matrix, velocity_matrix)
        jacobian_inv = inv(jacobian_matrix)
        test1 = self.matrix_multiply(jacobian_inv, jacobian_matrix)
        # print("Inverse Jacobian")
        # print(jacobian_inv)
        # joint velocities should be in rad/s
        # joint_velocity_matrix1 = np.dot(jacobian_matrix, velocity_matrix)
        jacobian_inv_flip = np.zeros([6, 6])
        for i in range(0, 6):
            for j in range(0, 6):
                jacobian_inv_flip[i][j] = jacobian_inv[j][i]
        joint_velocity_matrix1 = np.dot(jacobian_inv_flip, velocity_matrix)
        # joint_velocity_matrix1 = np.dot(jacobian_inv, velocity_matrix)
        # print('joint_velocity_matrix')
        # print(joint_velocity_matrix)
        # print('joint_velocity_matrix1')
        # print(joint_velocity_matrix1)
        position_matrix = np.zeros([7, 3])
        for i in range(0, 7):
            position_matrix[i][0] = x_positions[i]
            position_matrix[i][1] = y_positions[i]
            position_matrix[i][2] = z_positions[i]
        # simulate(x_positions, y_positions, z_positions)
        return joint_velocity_matrix1, position_matrix

#Kinematics()
#app.mainloop()