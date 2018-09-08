"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 9/3/18
"""

import triangle
import triangle.plot as plot
import matplotlib.pyplot as plt

#
box = triangle.get_data('box')
#
ax1 = plt.subplot(121, aspect='equal')
triangle.plot.plot(ax1, **box)


t = triangle.triangulate(box, 'pc')

ax2 = plt.subplot(122, sharex=ax1, sharey=ax1)
plot.plot(ax2, **t)


plt.show()

#
# # adj_region = { 0: [0],
# #                1: [1, 2, 5],
# #                2: [2, 1, 3, 7],
# #                3: [3, 2, 4, 7],
# #                4: [4, 3, 8],
# #                5: [5, 1, 6],
# #                6: [6, 5, 7, 10],
# #                7: [7, 2, 3, 6, 9],    # l1
# #                8: [8, 4, 9, 14],
# #                9: [9, 7, 8, 10],
# #                10:[10, 6, 9, 11],
# #                11:[11, 10, 12, 27],
# #                12:[12, 13, 21, 24, 11],  # l4
# #                13:[13, 14, 15, 12],
# #                14:[14, 8, 20, 13],
# #                15:[15, 13, 16, 21],
# #                16:[16, 15, 17, 20],
# #                17:[17, 16, 18],
# #                18:[18, 17, 19],
# #                19:[19, 18, 20],
# #                20:[20, 14, 16, 19],   # l5
# #                21:[21, 12, 15, 22],
# #                22:[22, 21, 23],
# #                23:[23, 22, 24, 41],
# #                24:[24, 12, 23, 25],
# #                25:[25, 24, 26, 38],
# #                26:[26, 25, 27, 28],
# #                27:[27, 11, 26],
# #                28:[28, 26, 29, 37],
# #                29:[29, 28, 30, 33],
# #                30:[30, 29, 31],
# #                31:[31, 30, 32, 33],
# #                32:[32, 31, 34],
# #                33:[33, 29, 31, 34, 36],  # l2
# #                34:[34, 32, 33, 35],
# #                35:[35, 34, 36, 39],
# #                36:[36, 33, 35, 37],
# #                37:[37, 28, 36, 38],
# #                38:[38, 25, 37, 40, 41],  # l3
# #                39:[39, 35, 40],
# #                40:[40, 38, 39, 42],
# #                41:[41, 23, 38, 42],
# #                42:[42, 40, 41, 43],
# #                43:[43, 42]}
#
# # region_dict = {"l1": 7,
# #                "l2": 33,
# #                "l3": 38,
# #                "l4": 12,
# #                "l5": 20}
#
#
# # adj_region = { 0: [],
# #                1: [1, 2, 4, 5, 6],
# #                2: [2, 1, 3, 6, 7],        # l5
# #                3: [3, 2, 7],
# #                4: [4, 1, 5, 8],
# #                5: [5, 1, 4, 6, 7, 8],      # l1
# #                6: [6, 1, 2, 5, 7],
# #                7: [7, 2, 3, 5, 6, 8, 10, 11],
# #                8: [8, 4, 5, 7, 9, 10],
# #                9: [9, 8, 10, 12],
# #                10:[10, 7, 8, 9, 11, 12],   # l4
# #                11:[11, 7, 10, 12],
# #                12:[12, 9, 10, 11, 13],
# #                13:[13, 12, 14, 15, 16, 17, 18],
# #                14:[14, 13, 15, 19],
# #                15:[15, 13, 14, 16, 19],    # l2
# #                16:[16, 13, 15, 17, 19],
# #                17:[17, 13, 16, 18, 19],    # l3
# #                18:[18, 13, 17, 19],
# #                19:[19, 14, 15, 16, 17, 18]
# # }
#
# import cplex
#
# # ============================================================
# # This file gives us a sample to use Cplex Python API to
# # establish a Linear Programming model and then solve it.
# # The Linear Programming problem displayed bellow is as:
# #                  min z = cx
# #    subject to:      Ax = b
# # ============================================================
#
# # ============================================================
# # Input all the data and parameters here
# import numpy as np
#
# num_decision_var = 3
# num_constraints = 3
#
# A = np.array([
#     [1, -2, 1],
#     [-4, 1, 2],
#     [-2, 0, 1.0],
# ])
# b = np.array([11, 3, 1.0])
# c = np.array([-3.0, 1, 1])
#
# constraint_type = ["L", "G", "E"]  # Less, Greater, Equal
# # ============================================================
#
# # Establish the Linear Programming Model
# myProblem = cplex.Cplex()
#
# # Add the decision variables and set their lower bound and upper bound (if necessary)
# myProblem.variables.add(names=["x" + str(i) for i in range(num_decision_var)])
# for i in range(num_decision_var):
#     myProblem.variables.set_lower_bounds(i, 0.0)
#
# # Add constraints
#
# for i in range(num_constraints):
#     myProblem.linear_constraints.add(
#         lin_expr=[cplex.SparsePair(ind=[j for j in range(num_decision_var)], val=A[i])],
#         rhs=[b[i]],
#         # names = ["c"+str(i)],
#         senses=[constraint_type[i]]
#     )
#
# # Add objective function and set its sense
# for i in range(num_decision_var):
#     myProblem.objective.set_linear([(i, c[i])])
# myProblem.objective.set_sense(myProblem.objective.sense.minimize)
#
# # Solve the model and print the answer
# myProblem.solve()
# print(myProblem.solution.get_values())
#
# def contain(v_a, v_b, v_c):
#     lhs = np.array([[(v_b[0] - v_a[0])*(-1), (v_b[1] - v_a[1])*(-1), -1, 0, 0, 0, 0, 0, 0, 0],
#                     [v_b[0] - v_a[0], v_b[1] - v_a[1], 0, -1, 0, 0, 0, 0, 0, 0],
#                     [(v_c[0] - v_a[0])*(-1), (v_c[1] - v_a[1])*(-1), 0, 0, -1, 0, 0, 0, 0, 0],
#                     [v_c[0] - v_a[0], v_c[1] - v_a[1], 0, 0, 0, -1, 0, 0, 0, 0],
#                     [0, 0, 1, 0, 0, 0, -1, 0, 0, 0],
#                     [0, 0, -1, 0, 0, 0, -1, 0, 0, 0],
#                     [0, 0, 0, 1, 0, 0, 0, -1, 0, 0],
#                     [0, 0, 0, -1, 0, 0, 0, -1, 0, 0],
#                     [0, 0, 0, 0, 1, 0, 0, 0, -1, 0],
#                     [0, 0, 0, 0, -1, 0, 0, 0, -1, 0],
#                     [0, 0, 0, 0, 0, 1, 0, 0, 0, -1],
#                     [0, 0, 0, 0, 0, -1, 0, 0, 0, -1],
#                     ]
#                     )
#     rhs = np.array([(v_a[0] * (v_b[0] - v_a[0]) + v_a[1] * (v_b[1] - v_a[1])) * (-1),
#                     v_a[0] * (v_b[0] - v_a[0]) + v_a[1] * (v_b[1] - v_a[1]) + np.power(v_b[0] - v_a[0], 2) + np.power(
#                         v_b[1] - v_a[1], 2),
#                     (v_a[0] * (v_c[0] - v_a[0]) + v_a[1] * (v_c[1] - v_a[1])) * (-1),
#                     v_a[0] * (v_c[0] - v_a[0]) + v_a[1] * (v_c[1] - v_a[1]) + np.power(v_c[0] - v_a[0], 2) + np.power(
#                         v_c[1] - v_a[1], 2),0, 0, 0, 0, 0, 0, 0, 0])
#     sense = ["L", "L", "L", "L", "L", "L", "L", "L", "L", "L"]
#
#     return lhs, rhs, sense
#
#
# vertices = {1: (0, 0),
#             2: (0, 1),
#             3: (1, 1),
#             4: (1, 0),
#             # Inner square has these vertices:
#             # obstacle 1
#             5: (0.4, 1),
#             6: (0.6, 1),
#             7: (0.4, 0.7),
#             8: (0.6, 0.7),
#             # obstacle 2
#             9: (0.3, 0.2),
#             10: (0.7, 0.2),
#             11: (0.3, 0),
#             12: (0.7, 0),
#             # region 1
#             13: (0.1, 0.9),
#             14: (0.3, 0.9),
#             15: (0.1, 0.7),
#             16: (0.3, 0.7),
#             # region 2
#             17: (0.7, 0.9),
#             18: (0.9, 0.9),
#             19: (0.7, 0.7),
#             20: (0.9, 0.7),
#             # region 5
#             21: (0, 0.3),
#             23: (0, 0.1),
#             24: (0.2, 0.1),
#             # reg(ion 4
#             25: (0.3, 0.5),
#             26: (0.5, 0.5),
#             27: (0.3, 0.3),
#             28: (0.5, 0.3),
#             # region 3
#             29: (0.7, 0.5),
#             30: (0.9, 0.5),
#             31: (0.7, 0.3),
#             32: (0.9, 0.3)}
#
# triangles = {1: [22, 0, 23],
#              2: [0, 10, 23],
#              3: [8, 21, 23],
#              4: [27, 26, 8],
#              5: [21, 26, 24],
#              6: [26, 21, 8],
#              7: [8, 23, 10],
#              8: [20, 21, 24],
#              9: [24, 14, 20],
#              10: [1, 12, 13],
#              11: [12, 1, 14],
#              12: [14, 24, 15],
#              13: [14, 1, 20],
#              14: [15, 6, 13],
#              15: [6, 15, 24],
#              16: [13, 4, 1],
#              17: [4, 13, 6],
#              18: [6, 24, 25],
#              19: [9, 27, 8],
#              20: [9, 11, 3],
#              21: [30, 25, 27],
#              22: [27, 9, 30],
#              23: [9, 31, 30],
#              24: [31, 9, 3],
#              25: [3, 29, 31],
#              26: [30, 28, 25],
#              27: [25, 28, 7],
#              28: [7, 18, 16],
#              29: [7, 28, 18],
#              30: [5, 16, 17],
#              31: [16, 5, 7],
#              32: [29, 19, 18],
#              33: [19, 29, 2],
#              34: [17, 2, 5],
#              35: [2, 17, 19],
#              36: [29, 3, 2],
#              37: [28, 29, 18],
#              38: [7, 6, 25]}
#
# v_a = [0, 0]
# v_b = [1, 1]
# v_c = [2, 2]
# print(contain(v_a, v_b, v_c))
#
# n_Horizon = 10
# n_Robot = 190
# n_Region = 10
# position = []
#
# Robot_Region_Horizon = []
#
# num_decision_var = 10 * n_Robot * n_Horizon
# # Establish the Linear Programming Model
# myProblem = cplex.Cplex()
#
# # Add the decision variables and set their lower bound and upper bound (if necessary)
# myProblem.variables.add(names=["x" + str(i) for i in range(num_decision_var)])
# for i in range(n_Robot*n_Horizon):
#     myProblem.variables.set_lower_bounds(10*i, 0.0)
#     myProblem.variables.set_upper_bounds(10*i, 1.0)
#     myProblem.variables.set_lower_bounds(10*(i+1), 0.0)
#     myProblem.variables.set_upper_bounds(10*(i+1), 1.0)
#
#
# for horizonCounter in range(n_Horizon):
#     for robotCounter in range(n_Robot):
#         shift = horizonCounter * n_Robot * n_Region + robotCounter * n_Region
#         for region in range(n_Region):
#             if Robot_Region_Horizon[shift + region]:
#                 label = triangles[region]
#                 v_a, v_b, v_c = (vertices[label[0]], vertices[label[1]], vertices[label[2]])
#                 lhs, rhs, sense = contain(v_a, v_b, v_c)
#                 # np.concatenate((A, lhs), axis=0)
#                 # np.concatenate((b, rhs), axis=None)
#                 # Re = Re + sense
#
#                 for i in range(len(sense)):
#                     myProblem.linear_constraints.add(
#                         lin_expr=[cplex.SparsePair(ind=[10 * robotCounter * n_Horizon + 10 * horizonCounter,
#                                                         10 * robotCounter * n_Horizon + 10 * horizonCounter + 10],
#                                                    val=lhs[i])],
#                         rhs=[rhs[i]],
#                         # names = ["c"+str(i)],
#                         senses=[sense[i]]
#                     )
#                 break
# # Add objective function and set its sense
# for i in range(n_Robot * n_Horizon):
#     for j in range(6, 10):
#         myProblem.objective.set_linear([(10*i+j, 1)])
# myProblem.objective.set_sense(myProblem.objective.sense.minimize)
# # Solve the model and print the answer
# myProblem.solve()
# print("ok")
# # print(myProblem.solution.get_values())

