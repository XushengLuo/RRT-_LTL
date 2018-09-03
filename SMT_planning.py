"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 9/3/18
"""

"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 9/1/18
"""
from z3 import *

# n = 9
# X = [ [ Int("x_%s_%s" % (i+1, j+1)) for j in range(n) ]
#       for i in range(n) ]
#
# cells_c  = [ And(1 <= X[i][j], X[i][j] <= n)
#              for i in range(n) for j in range(n) ]
# rows_c   = [ Distinct(X[i]) for i in range(n) ]
#
# cols_c   = [ Distinct([ X[i][j] for i in range(n) ])
#              for j in range(n) ]
# sq_c     = [ Distinct([ X[3*i0 + i][3*j0 + j]
#                         for i in range(3) for j in range(3) ])
#              for i0 in range(3) for j0 in range(3) ]
# sudoku_c = cells_c + rows_c + cols_c + sq_c
#
# # sudoku instance, we use '0' for empty cells
# instance = ((0,0,0,0,9,4,0,3,0),
#             (0,0,0,5,1,0,0,0,7),
#             (0,8,9,0,0,0,0,4,0),
#             (0,0,0,0,0,0,2,0,8),
#             (0,6,0,2,0,1,0,5,0),
#             (1,0,2,0,0,0,0,0,0),
#             (0,7,0,0,0,0,5,2,0),
#             (9,0,0,0,6,5,0,0,0),
#             (0,4,0,9,7,0,0,0,0))
#
# instance_c = [ If(instance[i][j] == 0,
#                   True,
#                   X[i][j] == instance[i][j])
#                for i in range(9) for j in range(9) ]
#
# s = Solver()
# s.add(sudoku_c + instance_c)
# if s.check() == sat:
#     m = s.model()
#     r = [ [ m.evaluate(X[i][j]) for j in range(9) ]
#           for i in range(9) ]
#     print_matrix(r)
# else:
#     print ("failed")

import datetime


# def always(subformula, Robot_Region_Horizon, k, L, robot, region):
#     if k == L-1:
#         return [ subformula[0] == Robot_Region_Horizon[k][robot][region]]
#     else:
#         return [And([ subformula[0] == Robot_Region_Horizon[k][robot][region]] +
#                     always(subformula[1:], Robot_Region_Horizon, k+1, L, robot, region) )]


def always(subformula, k, L):
    if k == L:
        return subformula[k-1]
    else:
        return And(subformula[k-1], always(subformula, k+1, L))

def until(subformula, k, L):
    if k == L:
        return Or(subformula[1][k-1], And([ subformula[0][k-1]] + [Or([And(loop[i], until_aux(subformula, i, L)) for i in range(0, L)])]))
    else:
        return Or(subformula[1][k-1], And(subformula[0][k-1], until(subformula, k+1, L)))

def until_aux(subformula, k, L):
    if k == L:
        return subformula[1][k-1]
    else:
        return Or(subformula[1][k-1], And(subformula[0][k-1], until_aux(subformula, k+1, L)))

start = datetime.datetime.now()
n_Robot = 4
n_Region = 6
n_Horizon = 15
n_formula = 7

Robot_Region_Horizon = []
adj_region = {0: [0, 1],
              1: [0, 1, 2, 3],
              2: [1, 2, 3, 4],
              3: [2, 3, 4],
              4: [3, 4, 5],
              5: [4, 5]}

# boolean variable for loop
loop = [ Bool("l_%s" % (k)) for k in range(0, n_Horizon) ]
loop_c = [ loop[0] == False, sum([ If(loop[k], 1, 0) for k in range(0, n_Horizon) ]) == 1]  # loop starts from the second step

for L in range(n_Horizon, n_Horizon+1):
    # declare Boolean variables denoting robot i is in region j at time k
    #            position           |
    # robot      [               ]  | horizon 1
    #            position           |
    # robot      [               ]  | horizon 2
    Robot_Region_Horizon = [ [ [ Bool("x_%s_%s_%s" % (i, j, k)) for j in range(n_Region)]
                               for i in range(n_Robot)] for k in range(L) ]
    # pp(Robot_Region_Horizon)
    # formula satisfaction at time k
    #Formula_each_Horizon = [ Bool("y_%s" % k) for k in range(n_Horizon)]
    # each robot has to be in one region at any time instance
    one_region_c = [ sum([ If(Robot_Region_Horizon[k][i][j], 1, 0) for j in range(n_Region)]) == 1
                     for k in range(L) for i in range(n_Robot) ]
    # print(one_region_c)
    # initial
    # Robot_Region_Horizon[0][0][0] = True
    # Robot_Region_Horizon[0][1][0] = True

    init_c = [Robot_Region_Horizon[0][0][0] == True, Robot_Region_Horizon[0][1][0] == True, Robot_Region_Horizon[0][2][3] == True, Robot_Region_Horizon[0][3][0] == True]
    #, Robot_Region_Horizon[0][1][0] == True]
    # constraints on adjacent regions
    adj_region_c = []
    for i in range(n_Robot):
        for k in range(1, L):
            adj = []
            for j in range(n_Region):
                # candidate regions next horizon
                adj = adj + [ If(Robot_Region_Horizon[k][i][e], 1, 0) * If(Robot_Region_Horizon[k-1][i][j], 1, 0) for e in adj_region[j] ]
            adj_region_c = adj_region_c + [sum(adj) == 1]


    # pp(adj_region_c)

    # loop constraint
    loop_region_c = [If(loop[k], Robot_Region_Horizon[k - 1][i][j] == Robot_Region_Horizon[-1][i][j], True) for k in range(0, L) for i in range(n_Robot) for j in range(n_Region)]

    # formula = []<>l1_2 && []<>l2_3
    formula = [ [Bool("f_%s_%s" % (f, k)) for k in range(L)] for f in range(n_formula) ]
    # robot 1 visit region 6
    formula_1 = [formula[0][k] == Robot_Region_Horizon[k][0][5] for k in range(L)]
    always_eventually_1 = [Or([And(loop[i], Or([formula[0][j] for j in range(i, L)])) for i in range(0, L)])]
    # robot 2 visit region 5
    formula_2 = [formula[1][k] == Robot_Region_Horizon[k][1][4] for k in range(L)]
    always_eventually_2 = [Or([And(loop[i], Or([formula[1][j] for j in range(i, L)])) for i in range(0, L)])]
    # robot 3 visit region 5 always
    formula_3 = [formula[2][k] == Robot_Region_Horizon[k][2][3] for k in range(L)]
    always_1 =  [ always(formula[2], 1, L) ]
    # robot 4 not visit 4 until 5
    formula_4 = [formula[3][k] == Not(Robot_Region_Horizon[k][3][3]) for k in range(L)] + [formula[4][k] == Robot_Region_Horizon[k][3][4] for k in range(L)]
    until_1 = [until(formula[3:5], 1, L)]
    # eventually robot 4 visit 4
    formula_5 = [formula[5][k] == True for k in range(L)] + [
        formula[6][k] == Robot_Region_Horizon[k][3][3] for k in range(L)]
    until_2 = [until(formula[5:7], 1, L)]
    # pp(until_2)
    workspace_c = loop_c + one_region_c + adj_region_c + loop_region_c
    formula = formula_1 + formula_2 + formula_3 + formula_4 + formula_5 \
              + always_eventually_1 + always_eventually_2 + always_1 \
              + until_1 + until_2 + init_c

    s = Solver()
    s.add(workspace_c + formula)
    if s.check() == sat:
        print((datetime.datetime.now() - start).total_seconds())
        m = s.model()
        for i in range(n_Robot):
            print("Robot ", i+1, ': ', end=""),
            for k in range(L):
                for j in range(n_Region):
                    if m.evaluate(Robot_Region_Horizon[k][i][j]):
                        print(j+1, '--> ', end=""),
                        break
            print()
        # print(m)
        for i in range(1, n_Horizon):
            if m.evaluate(loop[i]):
                print('loop', i+1)
    else:
        print("Failure")


