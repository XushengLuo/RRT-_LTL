import numpy as np
from Problem import problemFormulation
from WorkspacePlot import region_plot
import matplotlib.pyplot as plt
from Buchi import buchi_graph
import re
import networkx as nx
# def label(ts, x):
#     """
#     generating the label of position state
#     :param x: position
#     :return: label
#     """
#     # whether x lies within obstacle
#     for (obs, boundary) in iter(ts['obs'].items()):
#         if obs[1] == 'b' and np.linalg.norm(np.subtract(x, boundary[0:-1])) <= boundary[-1]:
#             return obs[0]
#         elif obs[1] == 'p':
#             dictator = True
#             for i in range(len(boundary)):
#                 if np.dot(x, boundary[i][0:-1]) + boundary[i][-1] > 0:
#                     dictator = False
#                     break
#             if dictator == True:
#                 return obs[0]
#
#     # whether x lies within regions
#     for (regions, boundary) in iter(ts['region'].items()):
#         if regions[1] == 'b' and np.linalg.norm(x - np.asarray(boundary[0:-1])) <= boundary[-1]:
#             return regions[0]
#         elif regions[1] == 'p':
#             dictator = True
#             for i in range(len(boundary)):
#                 if np.dot(x, np.asarray(boundary[i][0:-1])) + boundary[i][-1] > 0:
#                     dictator = False
#                     break
#             if dictator == True:
#                 return regions[0]
#
#     return ''
#

# c21 = []
# c22 = []
# for i in range(10000):
#     c1 = np.random.uniform(0, 1, 2)
#     d = np.random.normal(0, .1, 1)
#     theta = np.random.uniform(-np.pi, np.pi, 1)
#     c2 = [(c1[0]+ d*np.cos(theta))[0], (c1[1]+ d*np.sin(theta))[0]]
#     c1t = label(ts, c1)
#     c2t = label(ts, np.asarray(c2))
#     if ('o' in c1t and 'o' not in c2t):
#         c21.append(c2[0])
#         c22.append(c2[1])
#     elif  'o' in c2t and 'o' not in c1t :
#         c21.append(c1[0])
#         c22.append(c1[1])
#
#
# # plot the workspace
# ax = plt.figure(1).gca()
# region_plot(regions, 'region', ax)
# region_plot(obs, 'obs', ax)
# plt.plot(c21, c22, 'or')
# plt.show()

def DelInfesEdge(buchi_graph, robot):
    """
    Delete infeasible edge
    :param buchi_graph: buchi automaton
    :param robot: # robot
    """
    TobeDel = []
    for edge in buchi_graph.edges():
        b_label = buchi_graph.edges[edge]['label']
        feas = True
        # split label with ||
        b_label = b_label.split('||')
        for label in b_label:
            feas = True
            # spit label with &&
            for r in range(robot):
                if len(re.findall(r'l.+?{0}'.format(r),label.replace('!l',''))) > 1:
                    feas = False
                    break
            if feas:
                break

        if not feas:
            TobeDel.append(edge)

    for edge in TobeDel:
        buchi_graph.remove_edge(edge[0], edge[1])


def MinLen(buchi_graph):
    """
    search the shorest path from a node to another
    :param buchi_graph:
    :return: dict of pairs of node : length of path
    """
    min_qb_dict = dict()
    for node1 in buchi_graph.nodes():
        for node2 in buchi_graph.nodes():
            if node1 != node2:
                try:
                    l, _ = nx.algorithms.single_source_dijkstra(buchi_graph, source=node1, target=node2)
                except nx.exception.NetworkXNoPath:
                    l = np.inf
                    # path = []
            else:
                l = np.inf
                # path = []
                for succ in buchi_graph.succ[node1]:
                    try:
                        l0, _ = nx.algorithms.single_source_dijkstra(buchi_graph, source=succ, target=node1)
                    except nx.exception.NetworkXNoPath:
                        l0 = np.inf
                        # path0 = []
                    if l0 < l:
                        l = l0
                        # path = path0
            min_qb_dict[(node1, node2)] = l

    return min_qb_dict

def FeasAcpt(buchi_graph, min_qb):
    """
    delte infeasible final state
    :param buchi_graph: buchi automaton
    :param min_qb: dict of pairs of node : length of path
    """
    accept = buchi_graph.graph['accept']
    for acpt in accept:
        if min_qb[(buchi_graph.graph['init'][0], acpt)] == np.inf or min_qb[(acpt, acpt)] == np.inf:
            buchi_graph.graph['accept'].remove(acpt)



workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}
buchi = buchi_graph(formula)
buchi.formulaParser()
buchi.execLtl2ba()
buchi_graph = buchi.buchiGraph()
DelInfesEdge(buchi_graph, len(init_state))
min_qb = MinLen(buchi_graph)
FeasAcpt(buchi_graph, min_qb)
