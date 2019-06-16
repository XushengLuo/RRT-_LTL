"""
Generate randomly LTL specifications
"""
import numpy as np
from shapely.geometry import Point, Polygon
import sys
import random
import task

class problemFormulation(object):
    def __init__(self):
        # +----------------------------------------------+
        # |                                              |
        # |                 Problem 1                    |
        # |                                              |
        # +----------------------------------------------+


        # +-----+-----+-----+
        # |  l1 |     | l2  |
        # |     +-----+     |
        # |       l4        |
        # |             l3  |
        # |    +-------+    |
        # | l5 |       |    |
        # +----+-------+----+
        # l1: (0.2, 0.8)
        # l2: (0.8, 0.8)
        # l3: (0.8, 0.4)
        # l4: (0.4, 0.4)
        # l5: (0.1, 0.2)

        self.workspace = (1, 1)
        # !! no whitespace in atomic proposation      b:ball s:square
        r = .2  #float(sys.argv[2])
        self.ap = {'l1', 'l2', 'l3', 'l4', 'l5', 'l6'}

        center = [(0.1, 0.7), (0.7, 0.7), (0.7, 0.3), (0.3, 0.3), (0, 0.1), (0, 0.4)]

        self.regions = {'l1': Polygon([(center[0][0], center[0][1]), (center[0][0] + r, center[0][1]),
                                       (center[0][0], center[0][1] + r)]),
                        'l2': Polygon([(center[1][0], center[1][1]), (center[1][0] + r, center[1][1]),
                                       (center[1][0], center[1][1] + r)]),
                        'l3': Polygon([(center[2][0], center[2][1]), (center[2][0] + r, center[2][1]),
                                       (center[2][0], center[2][1] + r)]),
                        'l4': Polygon([(center[3][0], center[3][1]), (center[3][0] + r, center[3][1]),
                                       (center[3][0], center[3][1] + r)]),
                        'l5': Polygon([(center[4][0], center[4][1]), (center[4][0] + r, center[4][1]),
                                       (center[4][0], center[4][1] + r)]),
                        'l6': Polygon([(center[5][0], center[5][1]), (center[5][0] + r, center[5][1]),
                                       (center[5][0], center[5][1] + r)]),
                        }

        self.obs = {'o1': Polygon([(0.3, 0.0), (0.7, 0.0), (0.7, 0.2), (0.3, 0.2)]),
                    'o2': Polygon([(0.4, 0.7), (0.6, 0.7), (0.6, 1.0), (0.4, 1.0)])}
        self.uni_cost = 0.1

        # sz = 8
        # nRobot = 8 * sz
        # init_state = []
        # for i in range(nRobot):
        #     init_state.append((0.8, 0.1))
        # self.init_state = tuple(init_state)
        #
        # group = np.array(range(1, nRobot + 1))
        # np.random.shuffle(group)
        # group = group.reshape(8, sz)
        #
        # formula = []
        # for i in range(8):
        #     subformula = []
        #     for j in range(sz):
        #         subformula.append('l' + str(np.random.randint(1, 7)) + '_' + str(group[i][j]))
        #     for j in range(sz//2):
        #         while True:
        #             g = np.random.randint(7)
        #             robot = group[g][np.random.randint(sz)]
        #             if robot not in group[i]:
        #                 break
        #         subformula.append('l' + str(np.random.randint(1, 7)) + '_' + str(robot))
        #
        #     formula.append('(' + ' && '.join(subformula) + ')')
        #
        # self.formula_comp = {i: formula[i-1] for i in range(1, 9)}
        # print(self.formula_comp)
        self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && <> e7 && []<>e8 && (!e7 U e8)'
        #
        # # subformula = []
        # # group = random.sample(range(1, nRobot+1), sz)
        # # for j in range(sz):
        # #     subformula.append('l' + str(np.random.randint(1, 6)) + '_' + str(group[j]))
        # # self.formula_comp[9] = '(' + ' && '.join(subformula) + ')'

        # self.formula_comp = \
        #     {1: '(l1_5 && l2_33 && l4_7 && l5_54 && l4_11 && l1_19 && l3_53 && l4_55 && l4_48 && l5_41)',
        #      2: '(l4_36 && l4_27 && l2_42 && l6_14 && l4_24 && l5_8 && l2_6 && l3_16 && l1_51 && l1_28)',
        #      3: '(l3_43 && l2_44 && l1_20 && l3_41 && l1_15 && l5_23 && l5_46 && l4_18 && l4_54 && l1_51)',
        #      4: '(l6_45 && l2_4 && l1_47 && l4_55 && l5_35 && l4_51 && l2_38 && l1_15 && l1_42 && l5_10)',
        #      5: '(l6_34 && l2_50 && l5_13 && l3_37 && l6_32 && l6_40 && l3_25 && l1_24 && l1_20 && l5_11)',
        #      6: '(l1_56 && l2_26 && l2_1 && l4_10 && l1_22 && l6_21 && l2_49 && l3_37 && l4_50 && l4_55)',
        #      7: '(l1_16 && l4_17 && l3_48 && l3_3 && l3_28 && l5_18 && l2_2 && l4_51 && l4_45 && l1_46)',
        #      8: '(l5_52 && l4_9 && l4_31 && l6_12 && l4_30 && l6_29 && l1_39 && l2_7 && l4_46 && l4_36)'}

        # self.init_state = \
        #     [(0.017, 0.978), (0.394, 0.23), (0.159, 0.61), (0.854, 0.139), (0.286, 0.065), (0.961, 0.105),
        #      (0.706, 0.06), (0.398, 0.222), (0.072, 0.843), (0.946, 0.631), (0.084, 0.975), (0.947, 0.972),
        #      (0.135, 0.94), (0.622, 0.439), (0.447, 0.66), (0.466, 0.56), (0.941, 0.633), (0.846, 0.926),
        #      (0.716, 0.264), (0.923, 0.086), (0.11, 0.993), (0.702, 0.219), (0.8, 0.183), (0.568, 0.493),
        #      (0.293, 0.897), (0.034, 0.75), (0.007, 0.025), (0.524, 0.494), (0.668, 0.486), (0.971, 0.352),
        #      (0.757, 0.69), (0.392, 0.479), (0.22, 0.387), (0.751, 0.48), (0.515, 0.57), (0.264, 0.322), (0.512, 0.444),
        #      (0.28, 0.107), (0.268, 0.624), (0.495, 0.681), (0.233, 0.883), (0.7, 0.304), (0.735, 0.97), (0.002, 0.399),
        #      (0.924, 0.45), (0.132, 0.695), (0.711, 0.638), (0.906, 0.75), (0.98, 0.537), (0.185, 0.596),
        #      (0.772, 0.938), (0.283, 0.481), (0.287, 0.617), (0.305, 0.205), (0.785, 0.654), (0.619, 0.342)]
        self.init_state = eval('task.s'+sys.argv[1] + '_' + sys.argv[2])
        self.init_state = tuple(self.init_state)
        self.formula_comp = eval('task.t'+sys.argv[1] + '_' + sys.argv[2])

        self.exclusion = []
        self.no = []

    def Formulation(self):
        # print('Task specified by LTL formula: ' + self.formula)
        return self.workspace, self.regions, self.obs, self.init_state, self.uni_cost, self.formula, self.formula_comp, self.exclusion, self.no
