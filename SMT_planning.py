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

import datetime

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
        return subformula[k-1]
    else:
        return And(subformula[k-1], always(subformula, k+1, L))

def until(subformula, k, L):
    if k == L:
        return Or(subformula[L+k-1], And([ subformula[k-1]] + [Or([And(loop[i], until_aux(subformula, i, L)) for i in range(1, L)])]))
    else:
        return Or(subformula[L+k-1], And(subformula[k-1], until(subformula, k+1, L)))

def until_aux(subformula, k, L):
    if k == L:
        return subformula[L+k-1]
    else:
        return Or(subformula[L+k-1], And(subformula[k-1], until_aux(subformula, k+1, L)))

start = datetime.datetime.now()
n_Robot = 1
L = 40
n_formula = 9

Robot_Region_Horizon = []

adj_region = { 0: [0],
               1: [1, 2, 5],
               2: [2, 1, 3, 7],
               3: [3, 2, 4, 7],
               4: [4, 3, 8],
               5: [5, 1, 6],
               6: [6, 5, 7, 10],
               7: [7, 2, 3, 6, 9],    # l1
               8: [8, 4, 9, 14],
               9: [9, 7, 8, 10],
               10:[10, 6, 9, 11],
               11:[11, 10, 12, 27],
               12:[12, 13, 21, 24, 11],  # l4
               13:[13, 14, 15, 12],
               14:[14, 8, 20, 13],
               15:[15, 13, 16, 21],
               16:[16, 15, 17, 20],
               17:[17, 16, 18],
               18:[18, 17, 19],
               19:[19, 18, 20],
               20:[20, 14, 16, 19],   # l5
               21:[21, 12, 15, 22],
               22:[22, 21, 23],
               23:[23, 22, 24, 41],
               24:[24, 12, 23, 25],
               25:[25, 24, 26, 38],
               26:[26, 25, 27, 28],
               27:[27, 11, 26],
               28:[28, 26, 29, 37],
               29:[29, 28, 30, 33],
               30:[30, 29, 31],
               31:[31, 30, 32, 33],
               32:[32, 31, 34],
               33:[33, 29, 31, 34, 36],  # l2
               34:[34, 32, 33, 35],
               35:[35, 34, 36, 39],
               36:[36, 33, 35, 37],
               37:[37, 28, 36, 38],
               38:[38, 25, 37, 40, 41],  # l3
               39:[39, 35, 40],
               40:[40, 38, 39, 42],
               41:[41, 23, 38, 42],
               42:[42, 40, 41, 43],
               43:[43, 42]}

region_dict = {"l1": 7,
               "l2": 33,
               "l3": 38,
               "l4": 12,
               "l5": 20}

tri_dict = { 7  : "l1",
             33 : "l2",
             38 : "l3",
             12 : "l4",
             20 : "l5"}

n_Region = len(adj_region)


# boolean variable for loop

for n_Horizon in range(22, L):

    s = Solver()

    # declare Boolean variables denoting robot i is in region j at time k
    #            position           |
    # robot      [               ]  | horizon 1
    #            position           |
    # robot      [               ]  | horizon 2

    Robot_Region_Horizon = BoolVector("b", n_Robot * n_Region * n_Horizon)

    loop = BoolVector("l", n_Horizon)
    loop_c = [loop[0] == False, sum([If(loop[k], 1, 0) for k in range(1, n_Horizon)]) == 1]  # loop starts from the second step
    # loop constraint

    loop_region_c = [IMPLIES(loop[k], Robot_Region_Horizon[(k - 1) * n_Region * n_Robot + robotCounter * n_Region + region]
                             == Robot_Region_Horizon[(n_Horizon - 1) * n_Region * n_Robot + robotCounter * n_Region + region])
                     for k in range(1, n_Horizon) for robotCounter in range(n_Robot) for region in range(n_Region)]

    s.add(loop_c + loop_region_c)



    # formula satisfaction at time k
    # Formula_each_Horizon = [ Bool("y_%s" % k) for k in range(n_Horizon)]

    # initial state
    init_c = [Robot_Region_Horizon[43]]
    s.add(init_c)

    # formula
    # formula = [Robot_Region_Horizon[n_Robot * n_Region * (n_Horizon - 1) + 4]]
    # s.add(formula)

    # each robot has to be in one region at any time instance
    one = []
    for horizonCounter in range(0, n_Horizon):
        indexShift = horizonCounter * n_Region * n_Robot
        for robotCounter in range(0, n_Robot):
            one = one + [sum([BoolVar2Int(Robot_Region_Horizon[i + robotCounter * n_Region + indexShift]) for i in range(0, n_Region)]) == 1]
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




    # # formula = []<>l1_2 && []<>l2_3
    formula = BoolVector('f', n_formula * n_Horizon)
    # # robot 1 visit region 6
    # formula_1 = [formula[0][k] == Robot_Region_Horizon[k][0][5] for k in range(L)]
    # always_eventually_1 = [Or([And(loop[i], Or([formula[0][j] for j in range(i, L)])) for i in range(0, L)])]
    # # robot 2 visit region 5
    # formula_2 = [formula[1][k] == Robot_Region_Horizon[k][1][4] for k in range(L)]
    # always_eventually_2 = [Or([And(loop[i], Or([formula[1][j] for j in range(i, L)])) for i in range(0, L)])]
    # # robot 3 visit region 5 always
    # formula_3 = [formula[2][k] == Robot_Region_Horizon[k][2][3] for k in range(L)]
    # always_1 =  [ always(formula[2], 1, L) ]
    # # robot 4 not visit 4 until 5
    # formula_4 = [formula[3][k] == Not(Robot_Region_Horizon[k][3][3]) for k in range(L)] + [formula[4][k] == Robot_Region_Horizon[k][3][4] for k in range(L)]
    # until_1 = [until(formula[3:5], 1, L)]
    # # eventually robot 4 visit 4
    # formula_5 = [formula[5][k] == True for k in range(L)] + [
    #     formula[6][k] == Robot_Region_Horizon[k][3][3] for k in range(L)]
    # until_2 = [until(formula[5:7], 1, L)]
    # pp(until_2)
    # formula = formula_1 + formula_2 + formula_3 + formula_4 + formula_5 \
    # + always_eventually_1 + always_eventually_2 + always_1 + until_1 + until_2 + init_c



    ## <> l4_1
    # formula_1 = [formula[0][k] for k in range(L)] + [
    #     formula[1][k] == Robot_Region_Horizon[k][0][region_dict["l4"]] for k in range(L)]
    # eventually = [until(formula[0:2], 1, L)]
    #
    # ## !l1_1 U l2_1
    # formula_2 = [formula[2][k] == Not(Robot_Region_Horizon[k][0][region_dict["l1"]]) for k in range(L)] + [
    #     formula[3][k] == Robot_Region_Horizon[k][0][region_dict["l2"]] for k in range(L)]
    # nuntil = [until(formula[2:4], 1, L)]
    # #
    # ## []!l5_1
    # formula_3 = [formula[4][k] == Not(Robot_Region_Horizon[k][0][region_dict["l5"]]) for k in range(L)]
    # always_not =  [ always(formula[4], 1, L) ]
    #
    # # phi_1 = l3_1
    # formula_4 = [formula[5][k] == Robot_Region_Horizon[k][0][region_dict["l3"]] for k in range(L)]
    # # phi_2 = <>l1_1
    # formula_5 = [formula[6][k] == True for k in range(L)] + [
    #     formula[7][k] == Robot_Region_Horizon[k][0][region_dict["l1"]] for k in range(L)]
    #
    # # # l3_1 && <>l1_1
    # formula_6 = [ formula[8][k] == And(formula[5][k], until(formula[6:8], k+1, L)) for k in range(L)]
    # # # []<> (l3_1 && <>l1_1)
    # always_eventually = [Or([And(loop[i], Or([formula[8][j] for j in range(i, L)])) for i in range(0, L)])]


    ## []!l5_1
    robot = 0
    region = region_dict["l5"]
    formulaCounter = 0
    formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(Robot_Region_Horizon[n_Robot * n_Region *
                                horizonCounter + robot * n_Region + region]) for horizonCounter in range(n_Horizon)]
    always_not = [always(formula[formulaCounter * n_Horizon: (formulaCounter+1)*n_Horizon], 1, n_Horizon) ]
    s.add(formula_1 + always_not)

    ## <> l4_1
    robot = 0
    region = region_dict["l4"]
    formulaCounter = formulaCounter + 1
    formula_2 = [ formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] +\
                [formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                        horizonCounter + robot * n_Region + region] for horizonCounter in range(n_Horizon)]
    eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter+2)*n_Horizon], 1, n_Horizon)]
    # always_eventually = [Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
    #                                           for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_2 + eventually)

    # <> l1_1
    robot = 0
    region = region_dict["l1"]
    formulaCounter = formulaCounter + 2
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
        formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[ n_Robot * n_Region *
                                horizonCounter + robot * n_Region + region] for horizonCounter in range(n_Horizon)]
    # l3
    region = region_dict["l3"]
    formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] == Robot_Region_Horizon[ n_Robot * n_Region *
                                horizonCounter + robot * n_Region + region] for horizonCounter in range(n_Horizon)]
    # l3 && <> l1_1
    formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] ==
                            And(formula[(formulaCounter + 2) * n_Horizon + horizonCounter],
                                    until(formula[formulaCounter * n_Horizon: (formulaCounter+2)*n_Horizon], horizonCounter+1, n_Horizon)) for horizonCounter in range(n_Horizon)]
    # []<> (l3 && <> l1_1)
    always_eventually = [Or([And(loop[i], Or([formula[(formulaCounter + 3)* n_Horizon + horizonCounter]
                                              for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_3 + formula_4 + formula_5 + always_eventually)

    ## ! l1_1 U l2_1
    robot = 0
    region1 = region_dict["l1"]
    region2 = region_dict["l2"]
    formulaCounter = formulaCounter + 4
    formula_6 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(Robot_Region_Horizon[n_Robot * n_Region *
                                horizonCounter + robot * n_Region + region1]) for horizonCounter in range(n_Horizon)] + [
        formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                horizonCounter + robot * n_Region + region2]   for horizonCounter in range(n_Horizon)]
    not_until = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon)]
    s.add(formula_6 + not_until)

    #
    # # !l5_1 U l1_1
    # formula_6 = [formula[7][k] == Not(Robot_Region_Horizon[k][0][4]) for k in range(L)] + [
    #     formula[8][k] == Robot_Region_Horizon[k][0][0] for k in range(L)]
    # until_2 = [until(formula[7:9], 1, L)]
    if s.check() == sat:
        print((datetime.datetime.now() - start).total_seconds())
        # m = s.model()
        # for robotCounter in range(n_Robot):
        #     print("Robot ", robotCounter+1, ': ', end=""),
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
        # for horizonCounter in range(1, n_Horizon):
        #     if m.evaluate(loop[horizonCounter]):
        #         print('loop', horizonCounter+1)
        #         print(region_dict)
        break
    else:
        print("Failure when # horizon is {0}".format(n_Horizon))
        continue


