import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy.linalg import inv


def forward_kinematics(forward_input, back_type):
    a1 = 10
    a2 = 10
    a3 = 10
    a4 = 10
    a5 = 2
    a6 = 2

    #input_matrix = [[0, math.radians(90), 0, (a1)],
    #                        [0, 0, (a2), 0],
    #                        [0, math.radians(-90), 0, 0],
    #                        [0, math.radians(90), (a3+a4), 0],
    #                        [0, math.radians(-90), 0, 0],
    #                        [0, 0, (a5+a6), 0]]
    input_matrix = [[0, math.radians(90), 0, a1],
                   [0, 0, a2, 0],
                   [0, math.radians(+90), 0, 0],
                   [0, math.radians(-90), 0, (a3+a4)],
                   [0, math.radians(+90), 0, 0],
                   [0, 0, 0, (a5+a6)]]

    final_angle = np.zeros([6])
    work_frame = np.zeros([4, 4])
    tool_frame = np.zeros([4, 4])

    for i in range(len(work_frame)):
        work_frame[i, i] = 1
        tool_frame[i, i] = 1
    #print("joint input")
    for i in range(len(forward_input)):
        #print(round(math.radians(forward_input[i]),2))
        #print(round((forward_input[i]), 2))
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
                    J1_matrix[i][j] = input_matrix[0][2]*math.cos(input_matrix[0][0])
                    J2_matrix[i][j] = input_matrix[1][2]*math.cos(input_matrix[1][0])
                    J3_matrix[i][j] = input_matrix[2][2]*math.cos(input_matrix[2][0])
                    J4_matrix[i][j] = input_matrix[3][2]*math.cos(input_matrix[3][0])
                    J5_matrix[i][j] = input_matrix[4][2]*math.cos(input_matrix[4][0])
                    J6_matrix[i][j] = input_matrix[5][2]*math.cos(input_matrix[5][0])
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

    #print("J1 Matrix")
    #print(J1_matrix)

    H0_2 = matrix_multiply(J1_matrix, J2_matrix)
    H0_3 = matrix_multiply(H0_2, J3_matrix)
    H0_4 = matrix_multiply(H0_3, J4_matrix)
    H0_5 = matrix_multiply(H0_4, J5_matrix)
    H0_6 = matrix_multiply(H0_5, J6_matrix)

    #print("H0_2")
    #print(H0_2)

    x = H0_6[0][3]
    y = H0_6[1][3]
    z = H0_6[2][3]

    x_array = [0, J1_matrix[0][3], H0_2[0][3], H0_3[0][3], H0_4[0][3], H0_5[0][3], H0_6[0][3]] #, J2_matrix[0][3]
    y_array = [0, J1_matrix[1][3], H0_2[1][3], H0_3[1][3], H0_4[1][3], H0_5[1][3], H0_6[1][3]] #, J2_matrix[1][3]
    z_array = [0, J1_matrix[2][3], H0_2[2][3], H0_3[2][3], H0_4[2][3], H0_5[2][3], H0_6[2][3]] #, J2_matrix[2][3]

    rot_y = math.degrees(math.atan2((-1 * H0_6[2][2]), math.sqrt(math.pow(H0_6[1][2], 2)
                                                                          + math.pow(H0_6[0][2], 2))))
    rot_x = math.degrees(math.atan2((H0_6[2][1] / math.cos(math.radians(rot_y))),
                                             (H0_6[2][0] / math.cos(math.radians(rot_y)))))
    rot_z = math.degrees(math.atan2((H0_6[1][2] / math.cos(math.radians(rot_y))),
                                             (H0_6[0][2] / math.cos(math.radians(rot_y)))))

    if back_type == 1:
        return H0_6, x_array, y_array, z_array
    if back_type == 2:
        return H0_6


def inverse_kinematics():
    a1 = 10
    a2 = 10
    a3 = 10
    a4 = 10
    a5 = 10
    a6 = 10
    reverse_input = [0,
                     0,
                     0,
                     0,
                     0,
                     0]
    input_matrix = [[0, math.radians(90), 0, (a1)],
                    [0, 0, (a2), 0],
                    [0, math.radians(-90), 0, 0],
                    [0, math.radians(90), (a3 + a4), 0],
                    [0, math.radians(-90), 0, 0],
                    [0, 0, (a5 + a6), 0]]


def matrix_multiply(m1, m2):
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
    #print(solution)
    if type == 1:
        solution = np.zeros([length1, width1])
        for i in range(0, len(solution)):
            for j in range(0, len(solution)):
                #for k in range(0, len(solution)):
                for k in range(0, width2):
                    #solution[i][j] = (m1[i][j] * m2[k][j]) + solution[i][j]
                    #print(m1[i][k], ' + ', m2[k][j])
                    solution[i][j] = (m1[i][k] * m2[k][j]) + solution[i][j]
                    #solution[j][i] = (m1[k][i] * m2[j][k]) + solution[j][i]
            #print("")
    elif type == 2:
        solution = np.zeros([width2, length2])
        for i in range(0, len(solution)):
            for j in range(0, len(solution)):
                #solution[i][j] = (m1[i][j] * m2[k][j]) + solution[i][j]
                #print(m1[i][k], ' + ', m2[k][j])
                solution[i][j] = (m1[i][j] * m2[j]) + solution[i][j]
                #solution[j][i] = (m1[k][i] * m2[j][k]) + solution[j][i]
            #print("")
    return solution


def jacobian(joint_input, velocity_matrix):
    #units of
    #joint input units = degrees
    #joint_input = [90,45,-45,0,-90,0]
    #velocity matrix units = mm/s
    #velocity_matrix = [0,0,-10,0,0,0]
    print('home matrix')
    home_matrix, x_positions, y_positions, z_positions = forward_kinematics(joint_input, 1)
    #print("Home Matrix")
    #print(home_matrix)
    delta = 0.5 #degrees
    jacobian_matrix = np.zeros([6, 6])
    for i in range(0,6):
        partial_derivative_matrix = np.zeros([4, 4])
        #print(partial_derivative_matrix)
        joint_input[i] = joint_input[i] + delta
        #print('joint = ', (i+1))
        #print('len(home_matrix) = ', len(home_matrix))
        print("Delta Matrix")
        delta_matrix = forward_kinematics(joint_input, 2)
        #print('delta_matrix = ')
        #print(delta_matrix)
        for j in range(0,len(home_matrix)):
            for k in range(0,len(home_matrix[0])):
                #temp = (home_matrix[j][k] - delta_matrix[j][k])
                #partial_derivative_matrix[j][k] = (temp / delta)
                partial_derivative_matrix[j][k] = ((home_matrix[j][k] - delta_matrix[j][k]) / delta) #mm/rad or rad/rad
        rotation_matrix_transpose = np.zeros([3,3])
        rotation_matrix_derivative = np.zeros([3,3])
        rotation_matrix_home = np.zeros([3, 3])
        for j in range(0,3):
            for k in range(0,3):
                rotation_matrix_transpose[j][k] = home_matrix[k][j] # rad
                rotation_matrix_home[j][k] = home_matrix[j][k] # rad
                rotation_matrix_derivative[j][k] = partial_derivative_matrix[j][k] #rad/rad
                #rotation_matrix_transpose[j][k] = home_matrix[j][k]
                #rotation_matrix_derivative[j][k] = partial_derivative_matrix[k][j]
        #skew_symetric_matrix = matrix_multiply(rotation_matrix_derivative, rotation_matrix_transpose)
        test = matrix_multiply(rotation_matrix_transpose, rotation_matrix_home)
        #print('test')
        #print(test)
        skew_symetric_matrix = matrix_multiply(rotation_matrix_derivative, rotation_matrix_transpose) #rad
        #skew_symetric_matrix = np.dot(rotation_matrix_derivative, rotation_matrix_transpose)
        #if i == 5:
            #print('partial_derivative_matrix = ')
            #print(partial_derivative_matrix)
        #print('skew_symetric_matrix = ')
        #print(skew_symetric_matrix)

        #This was the original, I am going to try flipping it
        #jacobian_matrix[0][i] = partial_derivative_matrix[0][3]
        #jacobian_matrix[1][i] = partial_derivative_matrix[1][3]
        #jacobian_matrix[2][i] = partial_derivative_matrix[2][3]
        #jacobian_matrix[3][i] = skew_symetric_matrix[1][2]
        #jacobian_matrix[4][i] = -1 * skew_symetric_matrix[0][2]
        #jacobian_matrix[5][i] = skew_symetric_matrix[0][1]

        #This is the flipped version
        jacobian_matrix[i][0] = partial_derivative_matrix[0][3] #mm/rad
        jacobian_matrix[i][1] = partial_derivative_matrix[1][3] #mm/rad
        jacobian_matrix[i][2] = partial_derivative_matrix[2][3] #mm/rad
        #skew symetric could be wrong, below is the original
        jacobian_matrix[i][3] = skew_symetric_matrix[1][2] #rad
        jacobian_matrix[i][4] = -1 * skew_symetric_matrix[0][2] #rad
        jacobian_matrix[i][5] = skew_symetric_matrix[0][1] #rad

        #jacobian_matrix[i][5] = skew_symetric_matrix[1][2]
        #jacobian_matrix[i][3] = -1 * skew_symetric_matrix[0][2]
        #jacobian_matrix[i][4] = skew_symetric_matrix[0][1]
        #print('joint = ', i+1)
        #print('Vx = ', partial_derivative_matrix[0][3])
        #print('Vy = ', partial_derivative_matrix[1][3])
        #print('Vz = ', partial_derivative_matrix[2][3])
        #print('Wx = ', skew_symetric_matrix[1][2])
        #print('Vy = ', -1*skew_symetric_matrix[0][2])
        #print('Vz = ', skew_symetric_matrix[0][1])
        #print('skew_symetric_matrix')
        #print(skew_symetric_matrix)
        joint_input[i] = joint_input[i] - delta

    #print('jacobian = ')
    #print(jacobian_matrix)
    #print('velocity_matrix')
    #print(velocity_matrix)
    #jacobian in mm/rad (translation) and rad (orientation)
    #velocity in mm/sec (translation) and rad/sec (orientation)
    joint_velocity_matrix = matrix_multiply(jacobian_matrix, velocity_matrix)
    #print(jacobian_matrix)
    jacobian_inv = inv(jacobian_matrix)
    test1 = matrix_multiply(jacobian_inv, jacobian_matrix)
    print("Inverse Jacobian")
    print(jacobian_inv)
    print("velocity matrix")
    print(velocity_matrix)
    #joint velocities should be in rad/s
    #joint_velocity_matrix1 = np.dot(jacobian_matrix, velocity_matrix)
    jacobian_inv_flip = np.zeros([6,6])
    for i in range(0,6):
        for j in range(0,6):
            jacobian_inv_flip[i][j] = jacobian_inv[j][i]
    joint_velocity_matrix1 = np.dot(jacobian_inv_flip, velocity_matrix)
    #print("joint_velocity_matrix1")
    #print(joint_velocity_matrix1)
    #joint_velocity_matrix1 = np.dot(jacobian_inv, velocity_matrix)
    #print('joint_velocity_matrix')
    #print(joint_velocity_matrix)
    #print('joint_velocity_matrix1')
    #print(joint_velocity_matrix1)
    position_matrix = np.zeros([7, 3])
    for i in range(0,7):
        position_matrix[i][0] = x_positions[i]
        position_matrix[i][1] = y_positions[i]
        position_matrix[i][2] = z_positions[i]
    print("X position = ", x_positions[6])
    print("Y position = ", y_positions[6])
    print("Z position = ", z_positions[6])
    #simulate(x_positions, y_positions, z_positions)
    return joint_velocity_matrix1, position_matrix


def simulate(spots):
    x_spot = np.zeros([7])
    y_spot = np.zeros([7])
    z_spot = np.zeros([7])
    holder = spots[0]
    #print('holder = ', holder[6][2])
    #print('len(x_spots[0]) = ', len(x_spot))
    print(len(spots))
    for i in range(len(spots[0])):
        x_spot[i] = holder[i][0]
        y_spot[i] = holder[i][1]
        z_spot[i] = holder[i][2]
        #x_spot[i] = spots[0][i][0]
        #y_spot[i] = spots[0][i][1]
        #z_spot[i] = spots[0][i][2]
    print('x_spot = ', x_spot)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(x_spot, y_spot, z_spot, c='r', marker='o')
    for i in range(1, len(x_spot)):
        pl = ax.plot([x_spot[i - 1], x_spot[i]], [y_spot[i - 1], y_spot[i]], [z_spot[i - 1], z_spot[i]], c='r')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_zlim(-5, 40)
    anim = FuncAnimation(fig, update, repeat=True, fargs=(ax, spots, sc, pl), frames=np.arange(0,len(spots)),
                         interval=50, blit=False)
    plt.show()


def update(inter, ax1, spot1, sc1, pl1):
    ax1.clear()
    #print('inter = ', inter)
    ax1.set_xlabel('X Label')
    ax1.set_ylabel('Y Label')
    ax1.set_zlabel('Z Label')
    ax1.set_xlim(-50, 50)
    ax1.set_ylim(-50, 50)
    ax1.set_zlim(-5, 40)
    x_spot1 = np.zeros([7])
    y_spot1 = np.zeros([7])
    z_spot1 = np.zeros([7])
    temp = spot1[inter]
    # print('holder = ', holder[6][2])
    # print('len(x_spots[0]) = ', len(x_spot))
    for i in range(len(spot1[inter])):
        x_spot1[i] = temp[i][0]
        y_spot1[i] = temp[i][1]
        z_spot1[i] = temp[i][2]
        # x_spot[i] = spots[0][i][0]
        # y_spot[i] = spots[0][i][1]
        # z_spot[i] = spots[0][i][2]
    #fig1 = plt.figure()
    #ax1 = fig1.add_subplot(111, projection='3d')
    ax1.scatter(x_spot1, y_spot1, z_spot1, c='r', marker='o')
    #sc1.set_offsets(np.c_[x_spot1, y_spot1, z_spot1])
    for i in range(1, len(x_spot1)):
        ax1.plot([x_spot1[i - 1], x_spot1[i]], [y_spot1[i - 1], y_spot1[i]], [z_spot1[i - 1], z_spot1[i]], c='r')
    return ax1


def main():
    #joints = [90,
    #          45,
    #          -45,
    #          50,
    #          -90,
    #          0]
    joints = [0,
              60,
              0,
              0,
              -60,
              0]
    # velocity matrix units = mm/s
    velocity = [5,
                0,
                0,
                0,
                0,
                0]

    delta_t = 10  # units will be milliseconds
    movetime = 200  # units of milliseconds
    stop = movetime/delta_t
    count = 0
    total_positions = np.zeros((int(stop), 7, 3))
    #while (delta_t * count) < movetime:
    while count < stop:
        joint_velocity, total_positions[count] = jacobian(joints, velocity)
        print('joint_velocity')
        for i in range(len(joint_velocity)):
            print(round(joint_velocity[i],2))
        #print(total_positions[count])
        for i in range(0, len(joints)):
            joints[i] = joints[i] + math.degrees((delta_t / 1000) * joint_velocity[i])
            #joints[i] = joints[i] + ((delta_t / 1000) * joint_velocity[i])
        #below this is debug output stuff
        if count == 0 or count == (stop-1):
            if count == 0:
                print('')
                print('Starting Position')
            else:
                print('')
                print('Ending Position')
            print('x = ', total_positions[count][6][0])
            print('y = ', total_positions[count][6][1])
            print('z = ', total_positions[count][6][2])
            print('')


        count += 1
        print(count)
    holder = total_positions[1]
    #print(holder[0:6][1])
    #print(total_positions[:][1][:])
    simulate(total_positions)


main()
#jacobian()
#forward_kinematics()