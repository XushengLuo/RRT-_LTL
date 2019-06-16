"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date_
"""
# Optimization-based Trajectory Generation with Linear Temporal Logic Specifications


from z3 import *
import cplex
import datetime
import numpy as np
from Problem import problemFormulation
import triangle
import triangle.plot as plot
import matplotlib.pyplot as plt
from WorkspacePlot import region_plot
from itertools import repeat
import sys
from SMT_problem import case_0, case_2_1, case_1, case_2, case_3, case_4, OR, case_5, case_auto
from termcolor import colored


def IMPLIES(b1, b2):
    return z3.Implies(b1, b2)


def BoolVar2Int(b):
    return If(b, 1, 0)


def tri(x, y):
    # plot the plan
    box = triangle.get_data('box')
    #
    # ax1 = plt.subplot(121, aspect='equal')
    # triangle.plot.plot(ax1, **box)

    t = triangle.triangulate(box, 'pc')

    ax2 = plt.subplot(111)  # , sharex=ax1, sharey=ax1)
    plot.plot(ax2, **t)
    plt.plot(x, y, 'yo-')

    plt.show()


def area(x1, y1, x2, y2, x3, y3):
    return np.fabs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)


# A function to check whether point P(x, y)
# lies inside the triangle formed by
# A(x1, y1), B(x2, y2) and C(x3, y3)
def isInside(x1, y1, x2, y2, x3, y3, x, y):
    # Calculate area of triangle ABC
    A = area(x1, y1, x2, y2, x3, y3)

    # Calculate area of triangle PBC
    A1 = area(x, y, x2, y2, x3, y3)

    # Calculate area of triangle PAC
    A2 = area(x1, y1, x, y, x3, y3)

    # Calculate area of triangle PAB
    A3 = area(x1, y1, x2, y2, x, y)

    # Check if sum of A1, A2 and A3
    # is same as A
    if np.fabs(A - A1 - A2 - A3) < 1e-10:
        return True
    else:
        return False


def contain(v_a, v_b, v_c):
    # whether a point is in the triangle
    # https://stackoverflow.com/questions/14757920/count-points-inside-triangle-fast
    x_a = v_a[0]
    y_a = v_a[1]
    x_b = v_b[0]
    y_b = v_b[1]
    x_c = v_c[0]
    y_c = v_c[1]
    v_0 = np.array([x_c - x_a, y_c - y_a])
    v_1 = np.array([x_b - x_a, y_b - y_a])

    dot00 = np.dot(v_0, v_0)
    dot01 = np.dot(v_0, v_1)
    dot11 = np.dot(v_1, v_1)

    invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    x1 = invDenom * dot11 * (x_c - x_a) - invDenom * dot01 * (x_b - x_a)
    y1 = invDenom * dot11 * (y_c - y_a) - invDenom * dot01 * (y_b - y_a)
    c1 = -invDenom * dot11 * x_a * (x_c - x_a) - invDenom * dot11 * y_a * (y_c - y_a) \
         + invDenom * dot01 * x_a * (x_b - x_a) + invDenom * dot01 * y_a * (y_b - y_a)

    x2 = invDenom * dot00 * (x_b - x_a) - invDenom * dot01 * (x_c - x_a)
    y2 = invDenom * dot00 * (y_b - y_a) - invDenom * dot01 * (y_c - y_a)
    c2 = -invDenom * dot00 * x_a * (x_b - x_a) - invDenom * dot00 * y_a * (y_b - y_a) + invDenom * dot01 * x_a * (
        x_c - x_a) + invDenom * dot01 * y_a * (y_c - y_a)

    x3 = x1 + x2
    y3 = y1 + y2
    c3 = c1 + c2

    lhs = np.array([[x1 * (-1), y1 * (-1)],
                    [x2 * (-1), y2 * (-1)],
                    [x3, y3]]
                   )

    rhs = np.array([c1, c2, 1 - c3])
    sense = ["L", "L", "L"]

    return lhs, rhs, sense

init = \
    [(0.526, 0.525), (0.324, 0.805), (0.546, 0.482), (0.091, 0.94), (0.633, 0.496), (0.603, 0.88), (0.971, 0.053),
     (0.781, 0.653), (0.816, 0.671), (0.489, 0.461), (0.002, 0.985), (0.007, 0.94), (0.774, 0.841), (0.992, 0.918),
     (0.147, 0.601), (0.18, 0.054)]

init = list(init)

# number of robots case : # robots
robot = {1: 1,
         2: 2,
         3: 16,
         4: 16,
         5: 16,
         6: 20,
         7: 20,
         21: 4,
         32: 32,
         40: 40,
         100: len(init)}
n_Robot = robot[100]  # robot[int(sys.argv[1])]
# specify the case
case = 6  # case = 3 including 3,4,5,6,7
# iteration starting point case:= : #robots
ite = {3: 27,
       2: 18,
       1: 21,
       21: 23,
       4: 30,
       5: 30,
       6: 20}

# inilize SAT solver
start_z3 = datetime.datetime.now()

L = 100

r = 0.25
# # --------- r = 0.25 ----------------
# adj_region = {0: [0],
#               1: [1, 2, 4],
#               2: [2, 1, 3, 37],
#               3: [3, 2, 6, 37],
#               4: [4, 1, 5, 37],
#               5: [5, 4, 9],
#               6: [6, 3, 7, 42],
#               7: [7, 6, 8, 12],
#               8: [8, 7, 9, 37],
#               9: [9, 5, 8, 10],
#               10: [10, 9, 11, 24],
#               11: [11, 10, 23, 40],
#               12: [12, 7, 14, 40],
#               13: [13, 14, 42],
#               14: [14, 12, 13, 15],
#               15: [15, 14, 16, 20],
#               16: [16, 15, 17, 41],
#               17: [17, 16, 18],
#               18: [18, 17, 19],
#               19: [19, 18, 41],
#               20: [20, 15, 21, 40],
#               21: [21, 20, 22],
#               22: [22, 21, 23, 34],
#               23: [23, 11, 22, 39],
#               24: [24, 10, 25],
#               25: [25, 24, 26, 30],
#               26: [26, 25, 27, 38],
#               27: [27, 26, 28],
#               28: [28, 27, 29],
#               29: [29, 28, 33, 38],
#               30: [30, 25, 31, 38],
#               31: [31, 30, 32, 39],
#               32: [32, 31, 33, 35],
#               33: [33, 29, 32],
#               34: [34, 22, 35, 39],
#               35: [35, 32, 34, 36],
#               36: [36, 35],
#               37: [37, 3, 4, 8],  # l1
#               38: [38, 26, 29, 30],  # l2
#               39: [39, 23, 31, 34],  # l3
#               40: [40, 11, 12, 20],  # l4
#               41: [41, 16, 19],  # l5
#               42: [42, 6, 13],  # l6
#               }
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
#             13: (0.1, 0.95),
#             14: (0.1, 0.7),
#             15: (0.35, 0.7),
#
#             # region 2
#             16: (0.7, 0.95),
#             17: (0.7, 0.7),
#             18: (0.95, 0.7),
#
#             # region 5
#             19: (0, 0.35),
#             20: (0, 0.1),
#             21: (0.25, 0.1),
#
#             # reg(ion 4
#             22: (0.3, 0.55),
#             23: (0.3, 0.3),
#             24: (0.55, 0.3),
#             # region 3
#
#             25: (0.7, 0.55),
#             26: (0.7, 0.3),
#             27: (0.95, 0.3),
#
#             # region 6
#             28: (0, 0.65),
#             29: (0, 0.4),
#             30: (0.25, 0.4)
#             }
#
# triangles = {19: [20, 1, 21],
#              17: [21, 11, 9],
#              18: [11, 21, 1],
#              15: [23, 19, 9],
#              20: [23, 9, 24],
#              16: [9, 19, 21],
#              14: [19, 23, 30],
#              6: [14, 28, 30],
#              2: [2, 28, 13],
#              3: [14, 13, 28],
#              8: [22, 15, 14],
#              12: [22, 30, 23],
#              4: [13, 15, 5],
#              9: [22, 7, 15],
#              7: [14, 30, 22],
#              13: [29, 19, 30],
#              1: [2, 13, 5],
#              21: [9, 10, 24],
#              36: [10, 12, 4],
#              22: [24, 10, 26],
#              34: [10, 27, 26],
#              35: [27, 10, 4],
#              31: [25, 27, 18],
#              32: [4, 18, 27],
#              23: [26, 25, 24],
#              10: [25, 7, 22],
#              24: [8, 7, 25],
#              27: [8, 16, 6],
#              5: [7, 5, 15],
#              30: [18, 17, 25],
#              28: [16, 3, 6],
#              29: [3, 16, 18],
#              33: [18, 4, 3],
#              26: [17, 16, 8],
#              25: [8, 25, 17],
#              11: [25, 22, 24],
#              37: [13, 14, 15],
#              38: [16, 17, 18],
#              39: [25, 26, 27],
#              40: [22, 23, 24],
#              41: [19, 20, 21],
#              42: [28, 29, 30]}
r = 0.2
# # ------------------------ r = 0.2 -------------------------
# # adjacency relation
adj_region = {0: [0],
              1: [1, 2, 4],
              2: [2, 1, 3, 37],
              3: [3, 2, 6],
              4: [4, 1, 5, 37],
              5: [5, 4, 9],
              6: [6, 3, 7, 42],
              7: [7, 6, 8, 12],
              8: [8, 7, 9, 37],
              9: [9, 5, 8, 10],
              10: [10, 9, 11, 24],
              11: [11, 10, 23, 40],
              12: [12, 7, 14, 40],
              13: [13, 14, 42],
              14: [14, 12, 13, 15],
              15: [15, 14, 16, 20],
              16: [16, 15, 17, 41],
              17: [17, 16, 18],
              18: [18, 17, 19],
              19: [19, 18, 41],
              20: [20, 15, 21, 40],
              21: [21, 20, 22],
              22: [22, 21, 23, 34],
              23: [23, 11, 22, 39],
              24: [24, 10, 25],
              25: [25, 24, 26, 30],
              26: [26, 25, 27, 38],
              27: [27, 26, 28],
              28: [28, 27, 29],
              29: [29, 28, 33, 38],
              30: [30, 25, 31, 38],
              31: [31, 30, 32, 39],
              32: [32, 31, 33, 35],
              33: [33, 29, 32],
              34: [34, 22, 35, 39],
              35: [35, 32, 34, 36],
              36: [36, 35],
              37: [37, 2, 4, 8],  # l1
              38: [38, 26, 29, 30],  # l2
              39: [39, 23, 31, 34],  # l3
              40: [40, 11, 12, 20],  # l4
              41: [41, 16, 19],  # l5
              42: [42, 6, 13],  # l6
              }
# coordinates of vertices
vertices = {1: (0, 0),
            2: (0, 1),
            3: (1, 1),
            4: (1, 0),
            # Inner square has these vertices:
            # obstacle 1
            5: (0.4, 1),
            6: (0.6, 1),
            7: (0.4, 0.7),
            8: (0.6, 0.7),
            # obstacle 2
            9: (0.3, 0.2),
            10: (0.7, 0.2),
            11: (0.3, 0),
            12: (0.7, 0),
            # region 1
            13: (0.1, 0.9),
            14: (0.1, 0.7),
            15: (0.3, 0.7),

            # region 2
            16: (0.7, 0.9),
            17: (0.7, 0.7),
            18: (0.9, 0.7),

            # region 5
            19: (0, 0.3),
            20: (0, 0.1),
            21: (0.2, 0.1),

            # reg(ion 4
            22: (0.3, 0.5),
            23: (0.3, 0.3),
            24: (0.5, 0.3),
            # region 3

            25: (0.7, 0.5),
            26: (0.7, 0.3),
            27: (0.9, 0.3),

            # region 6
            28: (0, 0.6),
            29: (0, 0.4),
            30: (0.2, 0.4)
            }
# vertices of each triangle
triangles = {19: [20, 1, 21],
             17: [21, 11, 9],
             18: [11, 21, 1],
             15: [23, 19, 9],
             20: [23, 9, 24],
             16: [9, 19, 21],
             14: [19, 23, 30],
             6: [14, 28, 30],
             2: [2, 14, 13],
             3: [14, 2, 28],
             8: [22, 15, 14],
             12: [22, 30, 23],
             4: [13, 15, 5],
             9: [22, 7, 15],
             7: [14, 30, 22],
             13: [29, 19, 30],
             1: [2, 13, 5],
             21: [9, 10, 24],
             36: [10, 12, 4],
             22: [24, 10, 26],
             34: [10, 27, 26],
             35: [27, 10, 4],
             31: [25, 27, 18],
             32: [4, 18, 27],
             23: [26, 25, 24],
             10: [25, 7, 22],
             24: [8, 7, 25],
             27: [8, 16, 6],
             5: [7, 5, 15],
             30: [18, 17, 25],
             28: [16, 3, 6],
             29: [3, 16, 18],
             33: [18, 4, 3],
             26: [17, 16, 8],
             25: [8, 25, 17],
             11: [25, 22, 24],
             37: [13, 14, 15],
             38: [16, 17, 18],
             39: [25, 26, 27],
             40: [22, 23, 24],
             41: [19, 20, 21],
             42: [28, 29, 30]}
r = 0.15
# ---------------------- r = 0.15 ---------------------
# adj_region = {0: [0],
#               1: [1, 2, 4],
#               2: [2, 1, 3],
#               3: [3, 2, 6, 37],
#               4: [4, 1, 5, 37],
#               5: [5, 4, 9],
#               6: [6, 3, 7, 42],
#               7: [7, 6, 8, 37],
#               8: [8, 7, 9, 12],
#               9: [9, 5, 8, 10],
#               10: [10, 9, 11, 24],
#               11: [11, 10, 23, 40],
#               12: [12, 8, 15, 40],
#               13: [13, 14, 42],
#               14: [14, 41, 13, 15],
#               15: [15, 12, 14, 16],
#               16: [16, 15, 17, 20],
#               17: [17, 16, 18],
#               18: [18, 17, 19],
#               19: [19, 18, 41],
#               20: [20, 16, 21, 40],
#               21: [21, 20, 22],
#               22: [22, 21, 23, 34],
#               23: [23, 11, 22, 39],
#               24: [24, 10, 25],
#               25: [25, 24, 26, 30],
#               26: [26, 25, 27, 38],
#               27: [27, 26, 28],
#               28: [28, 27, 29],
#               29: [29, 28, 33, 38],
#               30: [30, 25, 31, 38],
#               31: [31, 30, 32, 39],
#               32: [32, 31, 33, 35],
#               33: [33, 29, 32],
#               34: [34, 22, 35, 39],
#               35: [35, 32, 34, 36],
#               36: [36, 35],
#               37: [37, 3, 4, 7],  # l1
#               38: [38, 26, 29, 30],  # l2
#               39: [39, 23, 31, 34],  # l3
#               40: [40, 11, 12, 20],  # l4
#               41: [41, 14, 19],  # l5
#               42: [42, 6, 13],  # l6
#               }
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
#             13: (0.1, 0.85),
#             14: (0.1, 0.7),
#             15: (0.25, 0.7),
#
#             # region 2
#             16: (0.7, 0.85),
#             17: (0.7, 0.7),
#             18: (0.85, 0.7),
#
#             # region 5
#             19: (0, 0.25),
#             20: (0, 0.1),
#             21: (0.15, 0.1),
#
#             # reg(ion 4
#             22: (0.3, 0.45),
#             23: (0.3, 0.3),
#             24: (0.45, 0.3),
#             # region 3
#
#             25: (0.7, 0.45),
#             26: (0.7, 0.3),
#             27: (0.85, 0.3),
#
#             # region 6
#             28: (0, 0.55),
#             29: (0, 0.4),
#             30: (0.15, 0.4)
#             }
#
# triangles = {19: [20, 1, 21],
#              17: [21, 11, 9],
#              18: [11, 21, 1],
#              15: [23, 21, 30],
#              20: [23, 9, 24],
#              16: [9, 23, 21],
#              14: [19, 21, 30],
#              6: [14, 28, 30],
#              2: [2, 28, 13],
#              3: [14, 3, 28],
#              8: [22, 15, 30],
#              12: [22, 30, 23],
#              4: [13, 15, 5],
#              9: [22, 7, 15],
#              7: [14, 30, 15],
#              13: [29, 19, 30],
#              1: [2, 13, 5],
#              21: [9, 10, 24],
#              36: [10, 12, 4],
#              22: [24, 10, 26],
#              34: [10, 27, 26],
#              35: [27, 10, 4],
#              31: [25, 27, 18],
#              32: [4, 18, 27],
#              23: [26, 25, 24],
#              10: [25, 7, 22],
#              24: [8, 7, 25],
#              27: [8, 16, 6],
#              5: [7, 5, 15],
#              30: [18, 17, 25],
#              28: [16, 3, 6],
#              29: [3, 16, 18],
#              33: [18, 4, 3],
#              26: [17, 16, 8],
#              25: [8, 25, 17],
#              11: [25, 22, 24],
#              37: [13, 14, 15],
#              38: [16, 17, 18],
#              39: [25, 26, 27],
#              40: [22, 23, 24],
#              41: [19, 20, 21],
#              42: [28, 29, 30]}
r = 0.1
# ---------------------- r = 0.1 ---------------------
# adj_region = {0: [0],
#               1: [1, 2, 4],
#               2: [2, 1, 3, 37],
#               3: [3, 2, 6],
#               4: [4, 1, 5, 37],
#               5: [5, 4, 9],
#               6: [6, 3, 7, 42],
#               7: [7, 6, 8, 37],
#               8: [8, 7, 9, 12],
#               9: [9, 5, 8, 10],
#               10: [10, 9, 11, 40],
#               11: [11, 10, 23, 24],
#               12: [12, 8, 14, 40],
#               13: [13, 14, 42],
#               14: [14, 12, 13, 15],
#               15: [15, 41, 14, 16],
#               16: [16, 15, 17, 20],
#               17: [17, 16, 18],
#               18: [18, 17, 19],
#               19: [19, 18, 41],
#               20: [20, 16, 21, 40],
#               21: [21, 20, 22],
#               22: [22, 21, 23, 34],
#               23: [23, 11, 22, 39],
#               24: [24, 11, 25],
#               25: [25, 24, 26, 30],
#               26: [26, 25, 27, 38],
#               27: [27, 26, 28],
#               28: [28, 27, 29],
#               29: [29, 28, 33, 38],
#               30: [30, 25, 31, 38],
#               31: [31, 30, 32, 39],
#               32: [32, 31, 33, 35],
#               33: [33, 29, 32],
#               34: [34, 22, 35, 39],
#               35: [35, 32, 34, 36],
#               36: [36, 35],
#               37: [37, 2, 4, 7],  # l1
#               38: [38, 26, 29, 30],  # l2
#               39: [39, 23, 31, 34],  # l3
#               40: [40, 10, 12, 20],  # l4
#               41: [41, 15, 19],  # l5
#               42: [42, 6, 13],  # l6
#               }
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
#             13: (0.1, 0.8),
#             14: (0.1, 0.7),
#             15: (0.2, 0.7),
#
#             # region 2
#             16: (0.7, 0.8),
#             17: (0.7, 0.7),
#             18: (0.8, 0.7),
#
#             # region 5
#             19: (0, 0.2),
#             20: (0, 0.1),
#             21: (0.1, 0.1),
#
#             # reg(ion 4
#             22: (0.3, 0.4),
#             23: (0.3, 0.3),
#             24: (0.4, 0.3),
#             # region 3
#
#             25: (0.7, 0.4),
#             26: (0.7, 0.3),
#             27: (0.8, 0.3),
#
#             # region 6
#             28: (0, 0.5),
#             29: (0, 0.4),
#             30: (0.1, 0.4)
#             }
#
# triangles = {19: [20, 1, 21],
#              17: [21, 11, 9],
#              18: [11, 21, 1],
#              15: [23, 21, 19],
#              20: [23, 9, 24],
#              16: [9, 23, 21],
#              14: [19, 23, 30],
#              6: [14, 28, 30],
#              2: [2, 14, 13],
#              3: [14, 2, 28],
#              8: [22, 15, 30],
#              12: [22, 30, 23],
#              4: [13, 15, 5],
#              9: [22, 7, 15],
#              7: [14, 30, 15],
#              13: [29, 19, 30],
#              1: [2, 13, 5],
#              21: [9, 10, 24],
#              36: [10, 12, 4],
#              22: [24, 10, 26],
#              34: [10, 27, 26],
#              35: [27, 10, 4],
#              31: [25, 27, 18],
#              32: [4, 18, 27],
#              23: [26, 25, 24],
#              10: [24, 7, 22],
#              24: [8, 7, 25],
#              27: [8, 16, 6],
#              5: [7, 5, 15],
#              30: [18, 17, 25],
#              28: [16, 3, 6],
#              29: [3, 16, 18],
#              33: [18, 4, 3],
#              26: [17, 16, 8],
#              25: [8, 25, 17],
#              11: [25, 7, 24],
#              37: [13, 14, 15],
#              38: [16, 17, 18],
#              39: [25, 26, 27],
#              40: [22, 23, 24],
#              41: [19, 20, 21],
#              42: [28, 29, 30]}
r = 0.05
# --------------------- r = 0.05 ------------------------ right
# adj_region = {0: [0],
#               1: [1, 2, 4],
#               2: [2, 1, 3],
#               3: [3, 2, 6, 37],
#               4: [4, 1, 5, 37],
#               5: [5, 4, 9],
#               6: [6, 3, 7, 37],
#               7: [7, 6, 8, 42],
#               8: [8, 7, 9, 12],
#               9: [9, 5, 8, 10],
#               10: [10, 9, 24, 40],
#               11: [11, 23, 24, 25],
#               12: [12, 8, 14, 40],
#               13: [13, 14, 42],
#               14: [14, 12, 13, 15],
#               15: [15, 14, 16, 20],
#               16: [16, 15, 17, 41],
#               17: [17, 16, 18],
#               18: [18, 17, 19],
#               19: [19, 18, 41],
#               20: [20, 15, 21, 40],
#               21: [21, 20, 22],
#               22: [22, 21, 23, 34],
#               23: [23, 11, 22, 39],
#               24: [24, 10, 11],
#               25: [25, 11, 26, 30],
#               26: [26, 25, 27, 38],
#               27: [27, 26, 28],
#               28: [28, 27, 29],
#               29: [29, 28, 33, 38],
#               30: [30, 25, 31, 38],
#               31: [31, 30, 32, 39],
#               32: [32, 31, 33, 35],
#               33: [33, 29, 32],
#               34: [34, 22, 35, 39],
#               35: [35, 32, 34, 36],
#               36: [36, 35],
#               37: [37, 3, 4, 6],  # l1
#               38: [38, 26, 29, 30],  # l2
#               39: [39, 23, 31, 34],  # l3
#               40: [40, 10, 12, 20],  # l4
#               41: [41, 16, 19],  # l5
#               42: [42, 7, 13],  # l6
#               }
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
#             13: (0.1, 0.75),
#             14: (0.1, 0.7),
#             15: (0.15, 0.7),
#
#             # region 2
#             16: (0.7, 0.75),
#             17: (0.7, 0.7),
#             18: (0.75, 0.7),
#
#             # region 5
#             19: (0, 0.15),
#             20: (0, 0.1),
#             21: (0.05, 0.1),
#
#             # reg(ion 4
#             22: (0.3, 0.35),
#             23: (0.3, 0.3),
#             24: (0.35, 0.3),
#             # region 3
#
#             25: (0.7, 0.35),
#             26: (0.7, 0.3),
#             27: (0.75, 0.3),
#
#             # region 6
#             28: (0, 0.45),
#             29: (0, 0.4),
#             30: (0.05, 0.4)
#             }
# # vertices of each triangle
# triangles = {19: [20, 1, 21],
#              17: [21, 11, 9],
#              18: [11, 21, 1],
#              15: [23, 19, 9],
#              20: [23, 9, 24],
#              16: [9, 19, 21],
#              14: [19, 23, 30],
#              6: [14, 28, 15],
#              2: [2, 28, 13],
#              3: [14, 13, 28],
#              8: [22, 15, 30],
#              12: [22, 30, 23],
#              4: [13, 15, 5],
#              9: [22, 7, 15],
#              7: [15, 30, 28],
#              13: [29, 19, 30],
#              1: [2, 13, 5],
#              21: [9, 10, 24],
#              36: [10, 12, 4],
#              22: [24, 10, 26],
#              34: [10, 27, 26],
#              35: [27, 10, 4],
#              31: [25, 27, 18],
#              32: [4, 18, 27],
#              23: [26, 25, 24],
#              10: [24, 7, 22],
#              24: [8, 7, 24],
#              27: [8, 16, 6],
#              5: [7, 5, 15],
#              30: [18, 17, 25],
#              28: [16, 3, 6],
#              29: [3, 16, 18],
#              33: [18, 4, 3],
#              26: [17, 16, 8],
#              25: [8, 25, 17],
#              11: [8, 25, 24],
#              37: [13, 14, 15],
#              38: [16, 17, 18],
#              39: [25, 26, 27],
#              40: [22, 23, 24],
#              41: [19, 20, 21],
#              42: [28, 29, 30]}

# labeled region
region_dict = {"l1": 37,
               "l2": 38,
               "l3": 39,
               "l4": 40,
               "l5": 41,
               "l6": 42}
tri_dict = {37: "l1",
            38: "l2",
            39: "l3",
            40: "l4",
            41: "l5",
            42: "l6"}

n_Region = len(adj_region)


init_region = []
for ini in init:
    for index, vert in triangles.items():
        if isInside(vertices[vert[0]][0], vertices[vert[0]][1], vertices[vert[1]][0], vertices[vert[1]][1],
                    vertices[vert[2]][0], vertices[vert[2]][1], ini[0], ini[1]):
            init_region.append(index)
            break
pre_time = (datetime.datetime.now() - start_z3).total_seconds()


for n_Horizon in range(ite[case], L):
    start = datetime.datetime.now()

# for n_Horizon in range(int(sys.argv[1]), L):
# for n_Horizon in [27, 28, 27]:
    # Z3 solver
    s = Solver()
    s.reset()

    # declare Boolean variables denoting robot i is in region j at horizon (time) k
    #            position           |
    # robot      [               ]  | horizon 1
    #            position           |
    # robot      [               ]  | horizon 2

    Robot_Region_Horizon = BoolVector("b", n_Robot * n_Region * n_Horizon)

    loop = BoolVector("l", n_Horizon)
    loop_c = [loop[0] == False] + [sum([If(loop[horizonCounter], 1, 0)
                                        for horizonCounter in
                                        range(1, n_Horizon)]) == 1]  # loop starts from the second step
    # loop constraint
    loop_region_c = [
        IMPLIES(loop[k], Robot_Region_Horizon[(k - 1) * n_Region * n_Robot + robotCounter * n_Region + region]
                == Robot_Region_Horizon[(n_Horizon - 1) * n_Region * n_Robot + robotCounter * n_Region + region])
        for k in range(1, n_Horizon) for robotCounter in range(n_Robot) for region in range(n_Region)]

    s.add(loop_c + loop_region_c)

    # initial state all in tri 36
    init_c = []
    for robotCounter in range(n_Robot):
        init_c = init_c + [Robot_Region_Horizon[robotCounter * n_Region + init_region[robotCounter]]]
    # init_c = [Robot_Region_Horizon[robotCounter * n_Region + 36] for robotCounter in range(n_Robot)]
    s.add(init_c)

    # each robot has to be in one region at any time instance
    one = []
    for horizonCounter in range(0, n_Horizon):
        indexShift = horizonCounter * n_Region * n_Robot
        for robotCounter in range(0, n_Robot):
            one = one + [sum([BoolVar2Int(Robot_Region_Horizon[i + robotCounter * n_Region + indexShift]) for i in
                              range(0, n_Region)]) == 1]
    s.add(one)

    # constraints on adjacent regions
    adj = []
    for horizonCounter in range(0, n_Horizon - 1):
        indexShift = horizonCounter * n_Region * n_Robot
        indexShift_plus = (horizonCounter + 1) * n_Region * n_Robot
        for robotCounter in range(n_Robot):
            for counter in range(0, n_Region):
                adjacent = adj_region[counter]
                child = [Robot_Region_Horizon[i + robotCounter * n_Region + indexShift_plus] for i in adjacent]
                # print 'anticedent of', counter+indexShift, 'is ', anticedent
                adj = adj + [IMPLIES(Robot_Region_Horizon[counter + robotCounter * n_Region + indexShift], OR(*child))]
                # the * used to unpack the python list to arguments                        )
    s.add(adj)

    # case selection
    if case == 1:
        case_1(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 2:
        case_2(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 3:
        case_3(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 21:
        case_2_1(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 4:
        case_4(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 5:
        case_5(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 6:
        case_auto(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    # without considering formulation
    start_z3_less = datetime.datetime.now()
    if s.check() == sat:
        z3_time_less = (datetime.datetime.now() - start).total_seconds() + pre_time
        z3_time = (datetime.datetime.now() - start_z3).total_seconds()
        print(ite[case], n_Horizon, z3_time, z3_time_less)
        # print("Success when # horizon is {0}".format(n_Horizon))
        m = s.model()
        # print the plan
        # print('                ', end="")
        # for horizonCounter in range(n_Horizon):
        #     print('%2s' % (horizonCounter+1), ' --> ', end="")
        # print()
        # for robotCounter in range(n_Robot):
        #     print("Robot ", robotCounter + 1, ': ', end=""),
        #     for horizonCounter in range(1, n_Horizon):
        #         if m.evaluate(loop[horizonCounter]):
        #             print('<{0}> '.format(horizonCounter + 1), end="")
        #     for horizonCounter in range(n_Horizon):
        #         for counter in range(n_Region):
        #             indexShift = horizonCounter * n_Robot * n_Region + robotCounter * n_Region
        #             if m.evaluate(Robot_Region_Horizon[indexShift + counter]):
        #                 try:
        #                     print(colored('%2s' % tri_dict[counter], 'yellow'), ' --> ', end="")
        #                 except KeyError:
        #                     print('%2s' % counter, ' --> ', end=""),
        #                 break
        #     print()
        #
        # break

        start_cplex = datetime.datetime.now()
        num_decision_var = 2 * n_Robot * n_Horizon
        #                 robot1                --------                           robot2
        #  horizon 1      position 2
        #     |
        #     |
        #  horizon 2

        # Establish the Linear Programming Model
        myProblem = cplex.Cplex()

        # Add the decision variables and set their lower bound and upper bound (if necessary)
        myProblem.variables.add(names=["x" + str(i) for i in range(num_decision_var)])
        for i in range(n_Robot * n_Horizon):
            myProblem.variables.set_lower_bounds(2 * i, 0.0)
            myProblem.variables.set_upper_bounds(2 * i, 1.0)
            myProblem.variables.set_lower_bounds(2 * i + 1, 0.0)
            myProblem.variables.set_upper_bounds(2 * i + 1, 1.0)

        # initial position
        for robotCounter in range(n_Robot):
            myProblem.linear_constraints.add(
                lin_expr=[cplex.SparsePair(
                    ind=[j for j in range(robotCounter * 2, robotCounter * 2 + 2)],
                    val=[1, 0])],
                senses=['E'],
                rhs=[init[robotCounter][0]])
            myProblem.linear_constraints.add(
                lin_expr=[cplex.SparsePair(
                    ind=[j for j in range(robotCounter * 2, robotCounter * 2 + 2)],
                    val=[0, 1])],
                senses=['E'],
                rhs=[init[robotCounter][1]])

        # inside the true-labeled region
        for horizonCounter in range(1, n_Horizon):
            for robotCounter in range(n_Robot):
                shift = horizonCounter * n_Robot * n_Region + robotCounter * n_Region
                for region in range(n_Region):
                    if m.evaluate(Robot_Region_Horizon[shift + region]):
                        label = triangles[region]
                        v_a, v_b, v_c = (vertices[label[0]], vertices[label[1]], vertices[label[2]])
                        lhs, rhs, sense_type = contain(v_a, v_b, v_c)

                        for i in range(len(sense_type)):
                            myProblem.linear_constraints.add(
                                lin_expr=[cplex.SparsePair(
                                    ind=[j for j in range(2 * horizonCounter * n_Robot + robotCounter * 2,
                                                          2 * horizonCounter * n_Robot + robotCounter * 2 + 2)],
                                    val=lhs[i])],
                                senses=[sense_type[i]],
                                rhs=[rhs[i]])

                        break

        nIndicatorVars = n_Robot * (n_Robot - 1) * (n_Horizon - 1) * 2
        colname_ind = ["ind" + str(j + 1) for j in range(nIndicatorVars)]
        obj = [0.0] * nIndicatorVars
        lb = [0.0] * nIndicatorVars
        ub = [1.0] * nIndicatorVars
        types = ["B"] * nIndicatorVars
        myProblem.variables.add(lb=lb, ub=ub, types=types, names=colname_ind)

        # collision avoidance
        collisionThreshold = 0.005
        index = num_decision_var
        for horizonCounter in range(1, n_Horizon):
            for robotCounter in range(n_Robot - 1):
                for anotherrobotCounter in range(robotCounter + 1, n_Robot):

                    # Add ind1 + ind2 = 1
                    #     ind3 + ind4 = 1
                    #     ...
                    # constraints
                    # ind = range(index, index+4)
                    # val = [1.0] * 4
                    # row = [[ind, val]]
                    # myProblem.linear_constraints.add(lin_expr=row, senses="E", rhs=[2])

                    ind = range(index, index + 2)
                    val = [1.0] * 2
                    myProblem.linear_constraints.add(lin_expr=[[ind, val]], senses="E", rhs=[1])

                    ind = range(index+2, index + 4)
                    val = [1.0] * 2
                    myProblem.linear_constraints.add(lin_expr=[[ind, val]], senses="E", rhs=[1])

                    # x-coordinate
                    ic_dict = dict()
                    ic_dict["indvar"] = index
                    ic_dict["lin_expr"] = cplex.SparsePair(
                        ind=[2 * horizonCounter * n_Robot + robotCounter * 2,
                             2 * horizonCounter * n_Robot + anotherrobotCounter * 2],
                        val=[1.0, -1.0])
                    ic_dict["rhs"] = collisionThreshold
                    ic_dict["sense"] = "G"
                    ic_dict["complemented"] = 0
                    # if complement = 0, it means ind(index) = 1 -> ...
                    # robot.x - anotherrobot.x >= threshold
                    myProblem.indicator_constraints.add(**ic_dict)

                    # x-coordinate
                    index = index + 1
                    ic_dict = dict()
                    ic_dict["indvar"] = index
                    ic_dict["lin_expr"] = cplex.SparsePair(
                        ind=[2 * horizonCounter * n_Robot + robotCounter * 2,
                             2 * horizonCounter * n_Robot + anotherrobotCounter * 2],
                        val=[-1.0, 1.0])
                    ic_dict["rhs"] = collisionThreshold
                    ic_dict["sense"] = "G"
                    ic_dict["complemented"] = 0
                    # ind(index) = 1 -> ...
                    # - robot.x + anotherrobot.x >= threshold
                    myProblem.indicator_constraints.add(**ic_dict)

                    # y-coordinate
                    index = index + 1
                    ic_dict = dict()
                    ic_dict["indvar"] = index
                    ic_dict["lin_expr"] = cplex.SparsePair(
                        ind=[2 * horizonCounter * n_Robot + robotCounter * 2 + 1,
                             2 * horizonCounter * n_Robot + anotherrobotCounter * 2 + 1],
                        val=[1.0, -1.0])
                    ic_dict["rhs"] = collisionThreshold
                    ic_dict["sense"] = "G"
                    ic_dict["complemented"] = 0
                    # ind(index) = 1 -> ...
                    # robot.y - anotherrobot.y >= threshold
                    myProblem.indicator_constraints.add(**ic_dict)

                    # y-coordinate
                    index = index + 1
                    ic_dict = dict()
                    ic_dict["indvar"] = index
                    ic_dict["lin_expr"] = cplex.SparsePair(
                        ind=[2 * horizonCounter * n_Robot + robotCounter * 2 + 1,
                             2 * horizonCounter * n_Robot + anotherrobotCounter * 2 + 1],
                        val=[-1.0, 1.0])
                    ic_dict["rhs"] = collisionThreshold
                    ic_dict["sense"] = "G"
                    ic_dict["complemented"] = 0
                    # ind(index) = 1 -> ...
                    # - robot.y + anotherrobot.y >= threshold
                    myProblem.indicator_constraints.add(**ic_dict)

                    index = index + 1

        # for i in range(n_Robot * n_Horizon):
        #     for j in range(5, 8):
        #         myProblem.objective.set_linear([(2 * i + j, 1)])
        #
        # myProblem.objective.set_sense(myProblem.objective.sense.minimize)
        # Solve the model and print the answer
        # start = datetime.datetime.now()
        # print("# variables", myProblem.variables.get_num())
        # print("# constraints", myProblem.linear_constraints.get_num() + myProblem.indicator_constraints.get_num())
        myProblem.set_log_stream(None)
        myProblem.set_error_stream(None)
        myProblem.set_warning_stream(None)
        myProblem.set_results_stream(None)
        myProblem.solve()

        cplex_time = (datetime.datetime.now() - start_cplex).total_seconds()
        # print("runtime for cpex", (datetime.datetime.now() - start).total_seconds())
        values = myProblem.solution.get_values()

        x = [[] for i in repeat(None, n_Robot)]
        y = [[] for i in repeat(None, n_Robot)]

        for robotCounter in range(n_Robot):
            for horizonCounter in range(n_Horizon):
                x[robotCounter].append(values[2 * n_Robot * horizonCounter + robotCounter * 2])
                y[robotCounter].append(values[2 * n_Robot * horizonCounter + robotCounter * 2 + 1])

        z = [[] for i in repeat(None, n_Horizon)]

        for horizonCounter in range(n_Horizon):
            for robotCounter in range(n_Robot):
                z[horizonCounter] = z[horizonCounter] + [x[robotCounter][horizonCounter], y[robotCounter][horizonCounter]]

        cost = 0
        for horizonCounter in range(n_Horizon-1):
            cost = cost + np.linalg.norm(np.subtract(z[horizonCounter+1], z[horizonCounter]))
        print(ite[case], n_Horizon, z3_time, z3_time_less, cplex_time, cost/2)
        # workspace, regions, obs, init_state, uni_cost, formula, formula_comp, exclusion, no = problemFormulation().Formulation()
        # ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}
        # # plot the workspace
        # ax = plt.figure(1).gca()
        # region_plot(regions, 'region', ax)
        # region_plot(obs, 'obs', ax)
        # color = ['r', 'y', 'b', 'k', 'c', 'g']
        # for robotCounter in range(n_Robot):
        #     pre = plt.quiver(x[robotCounter][:-1], y[robotCounter][:-1], np.array(x[robotCounter][1:]) - np.array(x[robotCounter][:-1]), np.array(y[robotCounter][1:]) - np.array(y[robotCounter][:-1]), color='{0}'.format(color[robotCounter]),
        #                      scale_units='xy', angles='xy', scale=1)
        #     plt.plot(x[robotCounter], y[robotCounter], '{0}o-'.format(color[robotCounter]))
        # plt.show()
        break
    else:
        print("Failure when # horizon is {0}".format(n_Horizon))
