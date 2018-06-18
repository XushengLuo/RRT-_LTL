import matplotlib.pyplot as plt
from matplotlib import rc
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from WorkspacePlot import region_plot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, PathPatch
# register Axes3D class with matplotlib by importing Axes3D
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.text import TextPath
from matplotlib.transforms import Affine2D

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
# z = np.linspace(-2, 2, 100)
# r = z**2 + 1
# x = r * np.sin(theta)
# y = r * np.cos(theta)
# ax.plot(x, y, z, 'b-o')
# from matplotlib import rc
# #
# #
# rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
# rc('text', usetex=True)
#
# x = plt.linspace(0,5)
# plt.plot(x,plt.sin(x))
# # plt.ylabel(r"This is $V_n$")
# plt.show()
#
# x_pre = np.array([0.8       , 0.66463595, 0.48834263, 0.38685675, 0.27827833,
#        0.41151967, 0.56706444, 0.73034491, 0.75510994, 0.8027965 ,
#        0.78697186])
#
# y_pre = np.array([0.1       , 0.23688143, 0.36953218, 0.57693006, 0.74547345,
#        0.64496977, 0.66754569, 0.73907054, 0.58592861, 0.43115557,
#        0.55062343])
#
# z_pre = np.array([1, 1, 1, 3, 3, 5, 5, 5, 6, 6, 7])
#
# fig = plt.figure()
# plt.rc('text', usetex=True)
# plt.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(x_pre, y_pre, z_pre, 'r-o')
# # region_plot(regions, 'region', ax, 3)
# # region_plot(obs, 'obs', ax, 3)
# plt.savefig('3D.png', bbox_inches='tight', dpi=600)
# # plt.show()
#
#
# import matplotlib.pyplot
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.collections import PolyCollection
# import random
#
# dates       = [20020514, 20020515, 20020516, 20020517, 20020520]
# highs       = [1135, 1158, 1152, 1158, 1163]
# lows        = [1257, 1253, 1259, 1264, 1252]
# upperLimits = [1125.0, 1125.0, 1093.75, 1125.0, 1125.0]
# lowerLimits = [1250.0, 1250.0, 1156.25, 1250.0, 1250.0]
#
# zaxisvalues0= [0, 0, 0, 0, 0]
# zaxisvalues1= [1, 1, 1, 1, 1]
# zaxisvalues2= [2, 2, 2, 2, 2]
#
# fig = matplotlib.pyplot.figure()
# ax  = fig.add_subplot(111, projection = '3d')
#
# ax.plot(dates, zaxisvalues1, lowerLimits, color = 'b')
# ax.plot(dates, zaxisvalues2, upperLimits, color = 'r')
#
# verts = []; fcs = []
# for i in range(len(dates)-1):
#    xs = [dates[i],dates[i+1],dates[i+1],dates[i],dates[i]] # each box has 4 vertices, give it 5 to close it, these are the x coordinates
#    ys = [highs[i],highs[i+1],lows[i+1],lows[i], highs[i]]  # each box has 4 vertices, give it 5 to close it, these are the y coordinates
#    verts.append(zip(xs,ys))
#    fcs.append((random.random(),random.random(),random.random(),0.6))
#
# poly = PolyCollection(verts, facecolors = fcs, closed = False)
# ax.add_collection3d(poly, zs=[zaxisvalues0[0]] * len(verts), zdir='y') # in the "z" just use the same coordinate
#
# ax.scatter(dates, zaxisvalues0, highs, color = 'g', marker = "o")
# ax.scatter(dates, zaxisvalues0, lows, color = 'y', marker = "^")
#
# matplotlib.pyplot.show()
# # fig = plt.figure()
# # ax = fig.add_subplot(111, projection='3d')
# #
# # # Draw a circle on the x=0 'wall'
# # p = Circle((5, 5), 3)
# # ax.add_patch(p)
# # art3d.pathpatch_2d_to_3d(p, zdir="z", edgecolor='green')
# # ax.set_xlim(0, 10)
# # ax.set_ylim(0, 10)
# # ax.set_zlim(0, 10)
# plt.show()
#
# import networkx as nx
# from Buchi import buchi_graph
# from Problem import problemFormulation
# import pickle
# def layer_plot(tree, buchi):
#     path = list(nx.dfs_edges(tree))
#
#     # 2D workspace in 3D
#     fig = plt.figure(2)
#     plt.rc('text', usetex=True)
#     plt.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
#     ax = fig.add_subplot(111, projection='3d')
#     for i in range(len(path)):
#         x_pre = np.array([path[i][0][0][0], path[i][1][0][0]])
#         y_pre = np.array([path[i][0][0][1], path[i][1][0][1]])
#         z_pre = np.asarray([buchi[path[i][0][1]], buchi[path[i][1][1]]])
#         ax.plot(x_pre, y_pre, z_pre, 'r--d')
#     # region_plot(regions, 'region', ax)
#     # region_plot(obs, 'obs', ax)
#     plt.savefig('3D.png', bbox_inches='tight', dpi=600)
#     plt.show()
#
# def opt_path_plot(buchi):
#     with open('data_opt_path', 'rb') as filehandle:
#         pre = pickle.load(filehandle)
#
#     fig = plt.figure(2)
#     plt.rc('text', usetex=True)
#     plt.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
#     ax = fig.add_subplot(111, projection='3d')
#
#     x_pre = np.asarray([point[0][0] for point in pre[0]])
#     y_pre = np.asarray([point[0][1] for point in pre[0]])
#     z_pre = np.asarray([buchi[point[1]] for point in pre[0]])
#     ax.plot(x_pre, y_pre, z_pre, 'r--d')
#     plt.show()
#
# workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
# buchi = buchi_graph(formula)
# buchi.formulaParser()
# buchi.execLtl2ba()
# buchi_graph = buchi.buchiGraph()
# buchi_state = dict(zip(list(buchi_graph.nodes()), range(1, buchi_graph.number_of_nodes()+1)))
#
#
# opt_path_plot(buchi_state)

# def sglp2mulp( point):
#     """
#     convert single form point () to multiple form point ((), (), (), ...)
#     :param point: single form point ()
#     :return:  multiple form point ((), (), (), ...)
#     """
#     mp = []
#     for i in range(1):
#         mp.append(point[i*2 :(i+1)*2])
#         return tuple(mp)
#
# def mulp2sglp(point):
#     """
#     convert multiple form point ((),(),(),...) to single form point ()
#     :param point: multiple points ((),(),(),...)
#     :return: signle point ()
#     """
#     sp = []
#     for p in point:
#         sp = sp + list(p)
#     return tuple(sp)
#
# point = (1,2)
# print(sglp2mulp(point))     # (1,2)  -> ((1, 2),)                ((1,2), ) -> (((1, 2),),)
#
# point = ((1,2),)
# print(mulp2sglp(point))    # ((1,2),) -> (1,2)  ((1,2)) ->fault

# import pickle
#
# with open('data_opt_path', 'rb') as filehandle:
#     # store the data as binary data stream
#     prepath, sufpath = pickle.load(filehandle)
#
# print(sufpath)

#--------------------------------------------------------------------------------------------------------
import matplotlib.pyplot as plt


# x= [200, 300, 400, 500, 1000]
# y = [2.53, 2.39, 2.33 ,2.30, 2.21]
# tpre = [9.22, 22.15, 38.85, 57.43, 182.78]
# tsuf = ['72.6s', '362.2s', '0.34h','0.65h', '6.56h']
# p = [23, 48, 87, 105, 259]
# fig = plt.figure(2)
# plt.rc('text', usetex=True)
# plt.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
# ax = fig.add_subplot(111)
# ax.plot(x, y, 'r--d')
# plt.ylabel(r'Cost $J(\tau)$')
# plt.xlabel(r'$n_{\mathrm{max}}^{\mathrm{pre}} = n_{\mathrm{max}}^{\mathrm{suf}}$')
# plt.xticks([200, 300, 400, 500, 1000], ["200", "300", "400", "500", "1000"])
# ymin, ymax = plt.ylim()
# xmin, xmax = plt.xlim()
# for i in range(len(x)):
#     plt.axvline(x=x[i], ls = '--', lw = 0.5, color = 'k', ymin= 0, ymax=(y[i]-ymin)/(ymax-ymin))
#     plt.axhline(y=y[i], ls='--', lw = 0.5, color='k', xmin=0, xmax=(x[i] - xmin) / (xmax - xmin))
#
# plt.text(x[0]+50, y[0]-0.04, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[0]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[0]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[0]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# plt.text(x[1]+30, y[1]+0.025, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[1]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[1]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[1]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# plt.text(x[2], y[2]+0.025, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[2]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[2]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[2]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# plt.text(x[3]+60, y[3]+0.01, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[3]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[3]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[3]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# plt.text(x[4]-120, y[4]+0.04, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[4]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[4]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[4]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# plt.savefig(r'/Users/chrislaw/Box Sync/RRL_LTL_cntsSpace/figures/CostMaxNumIter.png', bbox_inches='tight', dpi=600)
# #
# plt.show()



# plt.show()
#-------------------------------------------------------------------------------------------
# import  pickle
# import networkx as nx
# import numpy as np
# n_pre = []
# n_suf = []
# for i in [0,1,2,3,4,5,6,7,8,10,11,13,14,15,16]:
#     with open('data/data_opt_path_b_010_{0}.0'.format(i), 'rb') as filehandle:
#         # store the data as binary data stream
#         (opt_path_pre, opt_path_suf) = pickle.load(filehandle)
#         tree_pre = pickle.load(filehandle)
#         tree_suf = pickle.load(filehandle)
#         buchi_state = pickle.load(filehandle)
#         sz = pickle.load(filehandle)
#         n_pre.append(tree_pre.tree.number_of_nodes())
#         n_suf.append(tree_suf.tree.number_of_nodes())
# print(np.mean(n_pre))
# print(np.mean(n_suf))
# -------------------------------------------------------------------------------------------
# import pyvisgraph as vg
# polys = [[vg.Point(0.4,1.0), vg.Point(0.4,0.7), vg.Point(0.6,0.7), vg.Point(0.6, 1.0)],
# [vg.Point(0.3,0.2), vg.Point(0.3,0.0), vg.Point(0.7,0.0), vg.Point(0.7, 0.2)]]
# g = vg.VisGraph()
# g.build(polys, status=False)
# shortest = g.shortest_path(vg.Point(1.0,0.8), vg.Point(1.0, 0.8))
# print(shortest[1])
# [Point(1.50, 0.00), Point(3.00, 1.00), Point(4.00, 6.00)]
# from scipy.stats import truncnorm
# def get_truncated_normal( mean=0, sd=1, low=0, upp=10):
#     return truncnorm(
#         (low - mean) / sd, (upp - mean) / sd, loc=mean, scale=sd)
# n = 0
# for i in range(100000):
#     d = get_truncated_normal(0, 4, 0, np.inf).rvs()
#     if d < 4*3:
#         n = n + 1
# print(n/100000)
#--------------------------------------------------------------------------------------------------------
# import re
# def PutNotInside(str):
#     substr = re.findall("(!\(.*?\))", str)  # ['!(p1 && p2)', '!(p4 && p5)']
#     for s in substr:
#         oldstr = s.strip().strip('!').strip('(').strip(')')
#         nstr = ''
#         for ss in oldstr.split():
#             if '&&' in ss:
#                 nstr = nstr + ' or '
#             elif 'or' in ss:
#                 nstr = nstr + ' && '
#             else:
#                 nstr = nstr + '!' + ss
#         str = str.replace(s, nstr)
#     return str
# x = 'p3 && !(p1 && p2) && !(p4 && p5)'
# print(PutNotInside(x))

# from sympy.logic.inference import satisfiable
# from sympy import symbols
# # x, y, z = symbols('x,y,z')
# # p1,p2,p3,p4,p5 = symbols('p1,p2,p3,p4,p5')
# # y = Symbol('l3_1')
# # print(satisfiable(p3 & ~(p1 & p2) & (~(p4 & p5))))
# # print(satisfiable())

import  re
def RobotRegion(exp, robot):
    """
    pair of robot and corresponding in the expression
    :param exp: logical expression
    :param robot: # of robots
    :return: dic of robot index : regions
    exp = 'l1_1 & l3_1 & l4_1 & l4_6 | l3_4 & l5_6'
    {1: ['l1_1', 'l3_1', 'l4_1'], 4: ['l3_4'], 6: ['l4_6', 'l5_6']}
    """

    robot_region_dict = dict()
    for r in range(robot):
        findall = re.findall(r'l\d+?_{0}'.format(r + 1), exp)
        if findall:
            robot_region_dict[str(r+1)] = findall

    return robot_region_dict

def FeasTruthTable(exp, robot_region):
    """

    :param exp:
    :return:
    """
    sgl_value = []
    for key, value in robot_region.items():
        if len(value) == 1:
            sgl_value.append(value[0])

    for prod in itertools.product(*robot_region.values()):
        # set one specific item to be true
        for true_rb_rg in prod:
            # set the other to be false
            value_cp = list(robot_region[true_rb_rg.split('_')[1]])
            if len(value_cp) > 1:
                value_cp.remove(true_rb_rg)
                # replace the rest with same robot to be ~
                for v_remove in value_cp:
                    exp = exp.replace(v_remove, '~' + true_rb_rg)

        exp = to_cnf(exp)
        # all value in expression
        value_in_exp = [value.name for value in exp.atoms()]
        # all single value in expression
        sgl_value_in_exp = [value for value in value_in_exp if value in sgl_value]
        not_sgl_value_in_exp = [value for value in value_in_exp if value not in sgl_value]
        subs1 = {true_rb_rg: True for true_rb_rg in not_sgl_value_in_exp}
        for p in itertools.product(*[[True, False]]*len(sgl_value_in_exp)):
            subs2 = {sgl_value_in_exp[i]:p[i] for i in range(len(sgl_value_in_exp))}
            subs  = {**subs1, **subs2}
            if exp.subs(subs):
                return subs
    return []
# def multireplace(string, replacements):
#     """
#     Given a string and a replacement map, it returns the replaced string.
#     :param str string: string to execute replacements on
#     :param dict replacements: replacement dictionary {value to find: value to replace}
#     :rtype: str
#     """
#     # Place longer ones first to keep shorter substrings from matching where the longer ones should take place
#     # For instance given the replacements {'ab': 'AB', 'abc': 'ABC'} against the string 'hey abc', it should produce
#     # 'hey ABC' and not 'hey ABc'
#     substrs = sorted(replacements, key=len, reverse=True)
#
#     # Create a big OR regex that matches any of the substrings to replace
#     regexp = re.compile('|'.join(map(re.escape, substrs)))
#
#     # For each match, look up the new string in the replacements
#     return regexp.sub(lambda match: replacements[match.group(0)], string)
# #
import itertools
from sympy.logic.boolalg import to_cnf
# from sympy import Symbol
#
# x = Symbol('x')

exp = '(l1_1 & l3_1 & l4_1 & l4_6 ) | ~(l4_3 & l3_5 & l2_1)'
robot_region = RobotRegion(exp,6)
truth_table = FeasTruthTable(exp, robot_region)
print(truth_table)
print(exp)
# # all combinitions of possible robot, region pair








# str = multireplace(str, dict):
# print(str)
# from sympy.logic.boolalg import to_cnf
# str = to_cnf(str)
# print(satisfiable(str))

