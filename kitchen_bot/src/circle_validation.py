from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# DH parameters of UR5 Robots
# a = [0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000]
# d = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823]
# alpha = [ 1.570796327, 0, 0, 1.570796327, -1.570796327, 0 ]
# theta_home = [0, -1.570796327, 0, -1.570796327, 0, 0]

# Declaring the variable for joint angles
theta1, theta2, theta4, theta5, theta6, theta7 = symbols("theta1 theta2 theta4 theta5 theta6 theta7")
d1, d3, d5, d7 = symbols("d1 d3 d5 d7")
d1 = 89.2
d3 = 425
d5 = 392.5
d7 = 205.5



T_01 = Matrix([[cos(theta1), 0, -sin(theta1), 0], [sin(theta1), 0, cos(theta1), 0],
                                 [0, -1, 0, d1], [0, 0, 0, 1]])
T_12 = Matrix([[cos(theta2),   0, sin(theta2), 0], [sin(theta2), 0, -cos(theta2), 0],
                                 [0, 1, 0, 0], [0, 0, 0, 1]])
T_23 = Matrix([[cos(0), 0, sin(0), 0], [sin(0), 0, -cos(0), 0],
                                 [0, 1, 0, d3], [0, 0, 0, 1]])
T_34 = Matrix([[cos(theta4), 0, -sin(theta4), 0], [sin(theta4), 0, cos(theta4), 0],
                                 [0, -1, 0, 0], [0, 0, 0, 1]])
T_45 = Matrix([[cos(theta5), 0, -sin(theta5), 0], [sin(theta5), 0, cos(theta5), 0],
                                 [0, -1, 0, d5], [0, 0, 0, 1]])
T_56 = Matrix([[cos(theta6), 0, sin(theta6), 0], [sin(theta6), 0, -cos(theta6), 0],
                                 [0, 1, 0, 0], [0, 0, 0, 1]])
T_67 = Matrix([[cos(theta7), -sin(theta7), 0, 0], [sin(theta7), cos(theta7), 0, 0],
                                 [0, 0, 1, d7], [0, 0, 0, 1]])

# Calculating transformation matrix from the base frame to end effector frame
T_02 = T_01 * T_12
T_04 = T_01 * T_12 * T_23 * \
                         T_34
T_05 = T_01 * T_12 * T_23 * \
                         T_34 * T_45
T_06 = T_01 * T_12 * T_23 * \
                         T_34 * T_45 * T_56
T_07 = T_01 * T_12 * T_23 * \
                         T_34 * T_45 * T_56 * \
                         T_67


theta_mat = Matrix([theta1, theta2, theta4, theta5, theta6, theta7])

z_1 = T_01[:3, 2]
z_2 = T_02[:3, 2]
z_3 = T_04[:3, 2]
z_4 = T_05[:3, 2]
z_5 = T_06[:3, 2]
z_6 = T_07[:3, 2]

xp = T_07[:3, 3]

dh_q = xp.jacobian(theta_mat)

z_mat = z_1.row_join(z_2).row_join(z_3).row_join(z_4).row_join(z_5).row_join(z_6)

J_0 = dh_q.col_join(z_mat)

q_new = Matrix([[0], [-pi/2], [-pi/4], [0], [112*pi/180], [0]])
subs_m = N(J_0.subs([(theta1, q_new[0]), (theta2, q_new[1]), (theta4, q_new[2]), (theta5, q_new[3]),
                     (theta6, q_new[4]), (theta7, q_new[5])]), 5)
J_0_inv = N(subs_m.inv(), 5)


x_dot = []
z_dot = []
q_dot = []
th = pi/2
xes = []
yes = []
zes = []

fig = plt.figure()
ax = Axes3D(fig)
ax.set_xlim3d(-550, -450)
ax.set_ylim3d(-100, 100)
ax.set_zlim3d(400, 600)

for i in range(40):
    x_dot.append(N(((-1)*1.25*100*sin(th)), 5))
    z_dot.append(N((100*1.25*cos(th)), 5))
    th += pi/20

for i in range(40):
    j_vel_mat = Matrix([[0], [x_dot[i]], [z_dot[i]], [0], [0], [0]])
    q_dot.append(J_0_inv*j_vel_mat)
    q_new = (q_new + q_dot[i]*0.05).evalf()
    transform = N(T_07.subs([(theta1, q_new[0]), (theta2, q_new[1]), (theta4, q_new[2]),
                                               (theta5, q_new[3]), (theta6, q_new[4]), (theta7, q_new[5])]), 5)

    xes.append(transform[0, 3])
    yes.append(transform[1, 3])
    zes.append(transform[2, 3])

    subs_m = N(J_0.subs([(theta1, q_new[0]), (theta2, q_new[1]), (theta4, q_new[2]), (theta5, q_new[3]),
                         (theta6, q_new[4]), (theta7, q_new[5])]), 5)
    J_0_inv = N(subs_m.inv(), 5)
    ax.scatter3D(transform[0, 3], transform[1, 3], transform[2, 3])
    plt.pause(0.1)
    print(i, subs_m.det())


plt.show()