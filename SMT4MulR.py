"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 9/5/18
"""

"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 9/3/18
"""

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

def BoolVar2Int(b):
    return If(b, 1, 0)


def AND(*b):
    return z3.And(b)


def OR(*b):
    return z3.Or(b)


def NOT(b):
    return z3.Not(b)


def IMPLIES(b1, b2):
    return z3.Implies(b1, b2)


def always(subformula, k, L):
    if k == L:
        return subformula[k - 1]
    else:
        return And(subformula[k - 1], always(subformula, k + 1, L))


def until(subformula, k, L):
    if k == L:
        return Or(subformula[L + k - 1],
                  And([subformula[k - 1]] + [Or([And(loop[i], until_aux(subformula, i, L)) for i in range(1, L)])]))
    else:
        return Or(subformula[L + k - 1], And(subformula[k - 1], until(subformula, k + 1, L)))


def until_aux(subformula, k, L):
    if k == L:
        return subformula[L + k - 1]
    else:
        return Or(subformula[L + k - 1], And(subformula[k - 1], until_aux(subformula, k + 1, L)))


def contain(v_a, v_b, v_c):
    # whether a point is in the triangle
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

    lhs = np.array([[x1 * (-1), y1 * (-1), -1, 0, 0, 0, 0, 0.0],
                    [x2 * (-1), y2 * (-1), 0, -1, 0, 0, 0, 0],
                    [x3, y3, 0, 0, -1, 0, 0, 0],
                    [0, 0, 1, 0, 0, -1, 0, 0],
                    [0, 0, -1, 0, 0, -1, 0, 0],
                    [0, 0, 0, 1, 0, 0, -1, 0],
                    [0, 0, 0, -1, 0, 0, -1, 0],
                    [0, 0, 0, 0, 1, 0, 0, -1],
                    [0, 0, 0, 0, -1, 0, 0, -1],
                    ]
                   )

    rhs = np.array([c1, c2, 1 - c3, 0, 0, 0, 0, 0, 0.0])
    sense = ["L", "L", "L", "L", "L", "L", "L", "L", "L"]

    return lhs, rhs, sense


def case_0(n_Robot, n_Region, n_Horizon, s):
    n_formula = 2
    formula = BoolVector('f', n_formula * n_Horizon)
    robot = 0
    region = 37
    formulaCounter = 0
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + \
                [formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                    horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon)]
    # always_eventually = [Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
    #                                           for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_2 + eventually)


def case_1(n_Robot, n_Region, n_Horizon, region_dict, s):
    # A. Single Robot Planning
    n_formula = 9
    formula = BoolVector('f', n_formula * n_Horizon)
    #             horizon_1      ......      horizon_n
    #
    # formula1_1
    #    .
    #    .
    # formulan_1
    # []!l5_1
    robot = 0
    region = region_dict["l5"]
    formulaCounter = 0
    formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                  horizonCounter + robot * n_Region + region])
                 for horizonCounter in range(n_Horizon)]
    always_not = [always(formula[formulaCounter * n_Horizon: (formulaCounter + 1) * n_Horizon], 1, n_Horizon)]
    s.add(formula_1 + always_not)

    # <> l4_1
    robot = 0
    region = region_dict["l4"]
    formulaCounter = formulaCounter + 1
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + \
                [formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                    horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon)]
    # always_eventually = [Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
    #                                           for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_2 + eventually)

    # <> l1_1
    robot = 0
    region = region_dict["l1"]
    formulaCounter = formulaCounter + 2
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
        formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                           horizonCounter + robot * n_Region + region]
        for horizonCounter in range(n_Horizon)]
    # l3
    region = region_dict["l3"]
    formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                    horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    # l3 && <> l1_1
    formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] ==
                 And(formula[(formulaCounter + 2) * n_Horizon + horizonCounter],
                     until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], horizonCounter + 1,
                           n_Horizon)) for horizonCounter in range(n_Horizon)]
    # []<> (l3 && <> l1_1)
    always_eventually = [
        Or([And(loop[i], Or([formula[(formulaCounter + 3) * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in
            range(1, n_Horizon)])]
    s.add(formula_3 + formula_4 + formula_5 + always_eventually)

    # !l1_1 U l2_1
    robot = 0
    region1 = region_dict["l1"]
    region2 = region_dict["l2"]
    formulaCounter = formulaCounter + 4
    formula_6 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                  horizonCounter + robot * n_Region + region1])
                 for horizonCounter in range(n_Horizon)] + [
                    formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[
                        n_Robot * n_Region *
                        horizonCounter + robot * n_Region + region2] for horizonCounter in range(n_Horizon)]
    not_until = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon)]
    s.add(formula_6 + not_until)


def case_2(n_Robot, n_Region, n_Horizon, region_dict, s):
    # B. Two Robots Planning
    n_formula = 5
    formula = BoolVector('f', n_formula * n_Horizon)

    # []<> l1_1
    robot = 0
    formulaCounter = 0
    region = region_dict["l1"]
    formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                              horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    always_eventually_1 = [
        Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_1 + always_eventually_1)

    # []<> l2_2
    robot = 1
    formulaCounter = formulaCounter + 1
    region = region_dict["l2"]
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                              horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    always_eventually_2 = [
        Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in
            range(1, n_Horizon)])]
    s.add(formula_2 + always_eventually_2)

    # <> l4_2
    robot1 = 1
    region = region_dict["l4"]
    formulaCounter = formulaCounter + 1
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
        formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                           horizonCounter + robot1 * n_Region + region]
        for horizonCounter in range(n_Horizon)]
    # l4_1
    robot2 = 0
    region = region_dict["l4"]
    # formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
    #                    horizonCounter + robot2 * n_Region + region] for horizonCounter in range(n_Horizon)]
    # l4_1 && <> l4_2
    formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] ==
                 And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot2 * n_Region + region],
                     until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], horizonCounter + 1,
                           n_Horizon)) for horizonCounter in range(n_Horizon)]
    # []<> (l4_1 && <> l4_2)
    always_eventually = [
        Or([And(loop[i], Or([formula[(formulaCounter + 2) * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_3 + formula_4 + always_eventually)


def case_2_1(n_Robot, n_Region, n_Horizon, region_dict, s):
    # n_formula = 1
    # formula = BoolVector('f', n_formula * n_Horizon)
    #
    # # []<> (l1_1 && l2_2 && l3_3 && l4_4 && l5_5 && l6_6)
    # formulaCounter = 0
    # formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 0 * n_Region + region_dict["l1"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region_dict["l2"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l3"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 3 * n_Region + region_dict["l4"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 4 * n_Region + region_dict["l5"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region_dict["l6"]]
    #     ) for horizonCounter in
    #              range(n_Horizon)]
    #
    # always_eventually_1 = [
    #     Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
    #                          for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    # s.add(formula_1 + always_eventually_1)

    # []<> (l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)
    n_formula = 9
    formula = BoolVector('f', n_formula * n_Horizon)
    formulaCounter = 0
    # []<> (l1_1 && l1_2)
    formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 0 * n_Region + region_dict["l1"]],
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region_dict["l1"]]) for
                 horizonCounter in
                 range(n_Horizon)]
    always_eventually_1 = [
        Or([And(loop[i],
                Or([formula[formulaCounter * n_Horizon + horizonCounter]
                    for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_1 + always_eventually_1)

    # []<> (l2_2 && l2_3)
    formulaCounter = formulaCounter + 1
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region_dict["l2"]],
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l2"]]) for
                 horizonCounter in
                 range(n_Horizon)]
    always_eventually_2 = [
        Or([And(loop[i],
                Or([formula[formulaCounter * n_Horizon + horizonCounter]
                    for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_2 + always_eventually_2)

    # []<> (l3_3 && l3_4)
    formulaCounter = formulaCounter + 1
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l3"]],
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 3 * n_Region + region_dict["l3"]]) for
                 horizonCounter in
                 range(n_Horizon)]
    always_eventually_3 = [
        Or([And(loop[i],
                Or([formula[formulaCounter * n_Horizon + horizonCounter]
                    for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_3 + always_eventually_3)

    # <> l6_3
    robot1 = 2
    region = region_dict["l6"]
    formulaCounter = formulaCounter + 1
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
        formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                           horizonCounter + robot1 * n_Region + region]
        for horizonCounter in range(n_Horizon)]
    # l5_4 && <> l6_3
    robot2 = 3
    region = region_dict["l5"]
    formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] ==
                 And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot2 * n_Region + region],
                     until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon],
                           horizonCounter + 1,
                           n_Horizon)) for horizonCounter in range(n_Horizon)]
    # <> (l5_4 && <> l6_3)
    formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] for horizonCounter in
                 range(n_Horizon)] + [
                    formula[(formulaCounter + 4) * n_Horizon + horizonCounter] == formula[
                        (formulaCounter + 2) * n_Horizon + horizonCounter]
                    for horizonCounter in range(n_Horizon)]
    # l4_1 && <> (l5_4 && <> l6_3)
    robot3 = 0
    region = region_dict["l4"]
    formula_6 = [formula[(formulaCounter + 5) * n_Horizon + horizonCounter] ==
                 And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot3 * n_Region + region],
                     until(formula[(formulaCounter + 3) * n_Horizon: (formulaCounter + 5) * n_Horizon],
                           horizonCounter + 1,
                           n_Horizon)) for horizonCounter in range(n_Horizon)]

    # []<> (l4_1 && <> (l5_4 && <> l6_3))
    always_eventually = [
        Or([And(loop[i],
                Or([formula[(formulaCounter + 5) * n_Horizon + horizonCounter]
                    for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_3 + formula_4 + formula_5 + formula_6 + always_eventually)


def case_3(n_Robot, n_Region, n_Horizon, region_dict, s):
    # C. Multi-robot Planning
    # n_formula = 9 + 1 # + 2 + 2  # + 2 + 1 + 2
    # n_formula = 12 + 2 + 2 + 1 + 2
    case = int(sys.argv[1])
    if case == 3:
        n_formula = 9
    elif case == 4:
        n_formula = 10
    elif case == 5:
        n_formula = 12
    elif case == 6:
        n_formula = 14
    elif case == 7:
        n_formula = 16

    formula = BoolVector('f', n_formula * n_Horizon)

    formulaCounter = 0

    if case >= 3:
        # []<> (l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)
        region = region_dict["l1"]
        formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 0 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 3 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 4 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region]) for horizonCounter in
                     range(n_Horizon)]
        always_eventually_1 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_1 + always_eventually_1)

        # []<> (l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11)
        formulaCounter = formulaCounter + 1
        region = region_dict["l2"]
        formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 6 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 7 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 8 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 9 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 10 * n_Region + region]) for horizonCounter in
                     range(n_Horizon)]
        always_eventually_2 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_2 + always_eventually_2)

        # []<> (l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16)
        formulaCounter = formulaCounter + 1
        region = region_dict["l3"]
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 10 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 11 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 12 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 13 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 14 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 15 * n_Region + region]) for horizonCounter in
                     range(n_Horizon)]
        always_eventually_3 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + always_eventually_3)

        # formula = 3 up to now

        # <> l6_11
        robot1 = 10
        region = region_dict["l6"]
        formulaCounter = formulaCounter + 1
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                               horizonCounter + robot1 * n_Region + region]
            for horizonCounter in range(n_Horizon)]
        # l5_6 && <> l6_11
        robot2 = 5
        region = region_dict["l5"]
        formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] ==
                     And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot2 * n_Region + region],
                         until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon],
                               horizonCounter + 1,
                               n_Horizon)) for horizonCounter in range(n_Horizon)]
        # <> (l5_6 && <> l6_11)
        formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] for horizonCounter in
                     range(n_Horizon)] + [
                        formula[(formulaCounter + 4) * n_Horizon + horizonCounter] == formula[
                            (formulaCounter + 2) * n_Horizon + horizonCounter]
                        for horizonCounter in range(n_Horizon)]
        # l4_1 && <> (l5_6 && <> l6_11)
        robot3 = 0
        region = region_dict["l4"]
        formula_6 = [formula[(formulaCounter + 5) * n_Horizon + horizonCounter] ==
                     And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot3 * n_Region + region],
                         until(formula[(formulaCounter + 3) * n_Horizon: (formulaCounter + 5) * n_Horizon],
                               horizonCounter + 1,
                               n_Horizon)) for horizonCounter in range(n_Horizon)]

        # []<> (l4_1 && <> (l5_6 && <> l6_11))
        always_eventually = [
            Or([And(loop[i],
                    Or([formula[(formulaCounter + 5) * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + formula_4 + formula_5 + formula_6 + always_eventually)

        # formula = 9 up to now

    if case >= 4:
        # [] !(l3_4 || l3_9)
        region = region_dict["l3"]
        formulaCounter = formulaCounter + 6
        formula_7 = [
            formula[formulaCounter * n_Horizon + horizonCounter] == Not(Or(Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                horizonCounter + 3 * n_Region + region],
                                                                           Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                horizonCounter + 8 * n_Region + region]))
            for horizonCounter in range(n_Horizon)]
        always_not = [always(formula[formulaCounter * n_Horizon: (formulaCounter + 1) * n_Horizon], 1, n_Horizon)]
        s.add(formula_7 + always_not)

        # formula = 10 up to now

    if case >= 5:
        # <> (l4_3 && l5_8 && l6_13)
        formulaCounter = formulaCounter + 1
        formula_8 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And(
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l4"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 7 * n_Region + region_dict["l5"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 12 * n_Region + region_dict["l6"]])
            for horizonCounter in range(n_Horizon)]
        eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon)]

        s.add(formula_8 + eventually)

        # formula = 12 up to now

    if case >= 6:
        # []<> (l2_17 && l2_18 && l4_19 & l4_20)
        formulaCounter = formulaCounter + 2
        formula_9 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 16 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 17 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 18 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 19 * n_Region + region_dict["l4"]]) for
                     horizonCounter in
                     range(n_Horizon)]
        always_eventually_4 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_9 + always_eventually_4)

        # formula = 14 up to now

    if case >= 7:
        # !(l4_3 && l5_8 && l6_13) U (l2_17 && l2_18 && l4_19 & l4_20)
        formulaCounter = formulaCounter + 1
        formula_10 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(
            formula[(formulaCounter - 2) * n_Horizon + horizonCounter]) for horizonCounter in range(n_Horizon)] + [
                         formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == formula[
                             (formulaCounter - 1) * n_Horizon + horizonCounter]
                         for horizonCounter in range(n_Horizon)]
        until_1 = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon)]

        s.add(formula_10 + until_1)

        # formula = 16 up to now

    # # [] <> e_10
    # # 10: '(l5_21 && l5_22 && (l6_23 || l5_23) && (l6_24 || l5_24))',
    #
    # formulaCounter = formulaCounter + 2
    # formula_11 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 20 * n_Region + region_dict["l5"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 21 * n_Region + region_dict["l5"]],
    #     Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 22 * n_Region + region_dict["l5"]],
    #        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 22 * n_Region + region_dict["l6"]]),
    #     Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 23 * n_Region + region_dict["l5"]],
    #        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 23 * n_Region + region_dict["l6"]]))
    #               for horizonCounter in range(n_Horizon)]
    # always_eventually_4 = [
    #     Or([And(loop[i],
    #             Or([formula[formulaCounter * n_Horizon + horizonCounter]
    #                 for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    # s.add(formula_11 + always_eventually_4)
    #
    # # # formula = 17 up to now
    #
    # # <> 11: '(l1_25 && l2_26 && l3_27 && l4_28 && l5_29 && l6_30)'}
    # formulaCounter = formulaCounter + 1
    # formula_12 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
    #     formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And(
    #         Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 24 * n_Region + region_dict["l1"]],
    #         Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 25 * n_Region + region_dict["l2"]],
    #         Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 26 * n_Region + region_dict["l3"]],
    #         Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 27 * n_Region + region_dict["l4"]],
    #         Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 28 * n_Region + region_dict["l5"]],
    #         Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 29 * n_Region + region_dict["l6"]]
    #     )
    #     for horizonCounter in range(n_Horizon)]
    # eventually_12 = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon)]
    # s.add(formula_12 + eventually_12)
    #
    # # formula = 19 up to now


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


# specify the case
case = 3  # case = 3 including 3,4,5,6,7
# number of robots case : #robots
robot = {1: 1,
         2: 2,
         3: 16,
         4: 16,
         5: 16,
         6: 20,
         7: 20,
         21: 4}
n_Robot = robot[int(sys.argv[1])]
# iteration starting point case:= : #robots
ite = {3: 27,
       2: 18,
       1: 21,
       21: 23}

# inilize SAT solver
start_z3 = datetime.datetime.now()

L = 100

r = 0.25
# --------- r = 0.25 ----------------
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
# ------------------------ r = 0.2 -------------------------
# adjacency relation
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

# for n_Horizon in range(ite[case], L):
# for n_Horizon in range(int(sys.argv[2]), L):
for n_Horizon in [27, 28, 27]:
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
    init_c = [Robot_Region_Horizon[robotCounter * n_Region + 36] for robotCounter in range(n_Robot)]
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
        case_1(n_Robot, n_Region, n_Horizon, region_dict, s)
    elif case == 2:
        case_2(n_Robot, n_Region, n_Horizon, region_dict, s)
    elif case == 3:
        case_3(n_Robot, n_Region, n_Horizon, region_dict, s)
    elif case == 21:
        case_2_1(n_Robot, n_Region, n_Horizon, region_dict, s)
    # without considering formulation
    start_z3_less = datetime.datetime.now()
    if s.check() == sat:
        z3_time_less = (datetime.datetime.now() - start_z3_less).total_seconds()
        z3_time = (datetime.datetime.now() - start_z3).total_seconds()
        # print(z3_time, z3_time_less)
        # print("Success when # horizon is {0}".format(n_Horizon))
        m = s.model()
        # print the plan
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
        #                     print("[[", tri_dict[counter], "]]", ' --> ', end="")
        #                 except KeyError:
        #                     print(counter, '--> ', end=""),
        #                 break
        #     print()

        # break
        start_cplex = datetime.datetime.now()
        num_decision_var = 8 * n_Robot * n_Horizon
        #                 robot1                --------                           robot2
        #  horizon 1      position 2 + slack_variable 3 + absolute value 3
        #     |
        #     |
        #  horizon 2

        # Establish the Linear Programming Model
        myProblem = cplex.Cplex()

        # Add the decision variables and set their lower bound and upper bound (if necessary)
        myProblem.variables.add(names=["x" + str(i) for i in range(num_decision_var)])
        for i in range(n_Robot * n_Horizon):
            myProblem.variables.set_lower_bounds(8 * i, 0.0)
            myProblem.variables.set_upper_bounds(8 * i, 1.0)
            myProblem.variables.set_lower_bounds(8 * i + 1, 0.0)
            myProblem.variables.set_upper_bounds(8 * i + 1, 1.0)

        # inside the true-labeled region
        for horizonCounter in range(n_Horizon):
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
                                    ind=[j for j in range(8 * horizonCounter * n_Robot + robotCounter * 8,
                                                          8 * horizonCounter * n_Robot + robotCounter * 8 + 8)],
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
        collisionThreshold = 0.01
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
                        ind=[8 * horizonCounter * n_Robot + robotCounter * 8,
                             8 * horizonCounter * n_Robot + anotherrobotCounter * 8],
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
                        ind=[8 * horizonCounter * n_Robot + robotCounter * 8,
                             8 * horizonCounter * n_Robot + anotherrobotCounter * 8],
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
                        ind=[8 * horizonCounter * n_Robot + robotCounter * 8 + 1,
                             8 * horizonCounter * n_Robot + anotherrobotCounter * 8 + 1],
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
                        ind=[8 * horizonCounter * n_Robot + robotCounter * 8 + 1,
                             8 * horizonCounter * n_Robot + anotherrobotCounter * 8 + 1],
                        val=[-1.0, 1.0])
                    ic_dict["rhs"] = collisionThreshold
                    ic_dict["sense"] = "G"
                    ic_dict["complemented"] = 0
                    # ind(index) = 1 -> ...
                    # - robot.y + anotherrobot.y >= threshold
                    myProblem.indicator_constraints.add(**ic_dict)

                    index = index + 1

        for i in range(n_Robot * n_Horizon):
            for j in range(5, 8):
                myProblem.objective.set_linear([(8 * i + j, 1)])
        myProblem.objective.set_sense(myProblem.objective.sense.minimize)
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
                x[robotCounter].append(values[8 * n_Robot * horizonCounter + robotCounter * 8])
                y[robotCounter].append(values[8 * n_Robot * horizonCounter + robotCounter * 8 + 1])

        z = [[] for i in repeat(None, n_Horizon)]

        for horizonCounter in range(n_Horizon):
            for robotCounter in range(n_Robot):
                z[horizonCounter] = z[horizonCounter] + [x[robotCounter][horizonCounter], y[robotCounter][horizonCounter]]

        cost = 0
        for horizonCounter in range(n_Horizon-1):
            cost = cost + np.linalg.norm(np.subtract(z[horizonCounter+1], z[horizonCounter]))
        # print("cost", cost/2)
        print(z3_time, z3_time_less, cplex_time, cost/2)
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
        # break
    else:
        print("Failure when # horizon is {0}".format(n_Horizon))
