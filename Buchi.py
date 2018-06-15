# -*- coding: utf-8 -*-

import subprocess
import os.path
import re
import networkx as nx
import numpy as np
from networkx.classes.digraph import DiGraph
import pyvisgraph as vg

class buchi_graph(object):
    """ construct buchi automaton graph
    Parameter:
        formula: LTL formula specifying task
    """

    def __init__(self, formula):
        self.formula = formula

    def formulaParser(self):
        """replace letter with symbol
        """
        indicator = 'FG'

        if [True for i in indicator if i in self.formula]:
            self.formula.replace('F', '<>').replace('G', '[]')

    def execLtl2ba(self):
        """ given formula, exectute the ltl2ba
        Parameter:
            buchi_str: output string of program ltl2ba  (utf-8 format)
        """

        dirname = os.path.dirname(__file__)
        self.buchi_str = subprocess.check_output(dirname + "/./ltl2ba -f \"" + self.formula + "\"", shell=True).decode("utf-8")

    def buchiGraph(self):
        """parse the output of ltl2ba
        Parameter:
            buchi_graph: Graph of buchi automaton
        """
        # find all states
        state_re = re.compile(r'\n(\w+):\n\t')
        state_group = re.findall(state_re, self.buchi_str)

        # find initial and accepting states
        init = [s for s in state_group if 'init' in s]
        accep = [s for s in state_group if 'accept' in s]

        """
        Format:
            buchi_graph.node = NodeView(('T0_init', 'T1_S1', 'accept_S1'))
            buchi_graph.edges = OutEdgeView([('T0_init', 'T0_init'), ('T0_init', 'T1_S1'),....])
            buchi_graph.succ = AdjacencyView({'T0_init': {'T0_init': {'label': '1'}, 'T1_S1': {'label': 'r3'}}})
        """
        self.buchi_graph = DiGraph(type='buchi', init=init, accept=accep)
        for state in state_group:
            # for each state, find transition relation
            # add node
            self.buchi_graph.add_node(state)
            state_if_fi = re.findall(state + r':\n\tif(.*?)fi', self.buchi_str, re.DOTALL)
            if  state_if_fi:
                relation_group = re.findall(r':: \((.*?)\) -> goto (\w+)\n\t', state_if_fi[0])
                for (label, state_dest) in relation_group:
                    # add edge
                    self.buchi_graph.add_edge(state, state_dest, label=label)

        return self.buchi_graph

    def ShorestPathBtRg(self, regions):
        """
        calculate shoresr path between any two labeled regions
        :param regions: regions
        :return: dict (region, region) : length
        """
        polys = [[vg.Point(0.4, 1.0), vg.Point(0.4, 0.7), vg.Point(0.6, 0.7), vg.Point(0.6, 1.0)],
                 [vg.Point(0.3, 0.2), vg.Point(0.3, 0.0), vg.Point(0.7, 0.0), vg.Point(0.7, 0.2)]]
        g = vg.VisGraph()
        g.build(polys, status=False)

        min_len_region = dict()
        for key1, value1 in regions.items():
            for key2, value2 in regions.items():
                init = value1[:2]
                tg = value2[:2]
                # shorest path between init and tg point
                shortest = g.shortest_path(vg.Point(init[0], init[1]), vg.Point(tg[0], tg[1]))
                # (key2, key1) is already checked
                if (key2, key1) in min_len_region.keys():
                    min_len_region[(key1, key2)] = min_len_region[(key2, key1)]
                else:
                    # different regions
                    if key1 != key2:
                        dis = 0
                        for i in range(len(shortest)-1):
                            dis = dis + np.linalg.norm(np.subtract((shortest[i].x, shortest[i].y), (shortest[i+1].x, shortest[i+1].y)))

                        min_len_region[(key1, key2)] = dis
                    # same region
                    else:
                        min_len_region[(key1, key2)] = 0

        return min_len_region

    def DelInfesEdge(self, robot):
        """
        Delete infeasible edge
        :param buchi_graph: buchi automaton
        :param robot: # robot
        """
        TobeDel = []
        for edge in self.buchi_graph.edges():
            b_label = self.buchi_graph.edges[edge]['label']
            feas = True
            # split label with ||
            b_label = b_label.split('||')
            for label in b_label:
                feas = True
                # spit label with &&
                for r in range(robot):
                    if len(re.findall(r'l.+?_{0}'.format(r+1), label.replace('!l', ''))) > 1:
                        feas = False
                        break
                if feas:
                    break

            if not feas:
                TobeDel.append(edge)

        for edge in TobeDel:
            self.buchi_graph.remove_edge(edge[0], edge[1])

    def MinLen(self):
        """
        search the shorest path from a node to another, weight = 1, i.e. # of state in the path
        :param buchi_graph:
        :return: dict of pairs of node : length of path
        """
        min_qb_dict = dict()
        for node1 in self.buchi_graph.nodes():
            for node2 in self.buchi_graph.nodes():
                if node1 != node2:
                    try:
                        l, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph, source=node1, target=node2)
                    except nx.exception.NetworkXNoPath:
                        l = np.inf
                        # path = []
                else:
                    l = np.inf
                    # path = []
                    for succ in self.buchi_graph.succ[node1]:
                        try:
                            l0, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph, source=succ, target=node1)
                        except nx.exception.NetworkXNoPath:
                            l0 = np.inf
                            # path0 = []
                        if l0 < l:
                            l = l0 + 1
                            # path = path0
                min_qb_dict[(node1, node2)] = l

        return min_qb_dict


    # def MinLen_Cost(self):
    #     """
    #     search the shorest path from a node to another, weight = cost
    #     :param buchi_graph:
    #     :return: dict of pairs of node : length of path
    #     """
    #     min_qb_dict = dict()
    #     for node1 in self.buchi_graph.nodes():
    #         for node2 in self.buchi_graph.nodes():
    #             c = np.inf
    #             if node1 != node2:
    #                 try:
    #                     path = nx.all_simple_paths(self.buchi_graph, source=node1, target=node2)
    #                     for i in range(len(path)-2):
    #                         word_init = self.buchi_graph.edges[(path[i], path[i+1])]['label']
    #                         word_tg = self.buchi_graph.edges[(path[i+1], path[i+2])]['label']
    #                         # calculate distance travelled from word_init to word_tg
    #                         t_s_b = True
    #                         # split label with ||
    #                         label_init = word_init.split('||')
    #                         label_tg = word_tg.split('||')
    #                         for label in b_label:
    #                             t_s_b = True
    #                             # spit label with &&
    #                             atomic_label = label.split('&&')
    #                             for a in atomic_label:
    #                                 a = a.strip()
    #                                 a = a.strip('(')
    #                                 a = a.strip(')')
    #                                 if a == '1':
    #                                     continue
    #                                 # whether ! in an atomic proposition
    #                                 if '!' in a:
    #                                     if a[1:] in x_label:
    #                                         t_s_b = False
    #                                         break
    #                                 else:
    #                                     if not a in x_label:
    #                                         t_s_b = False
    #                                         break
    #                             # either one of || holds
    #                             if t_s_b:
    #                                 return t_s_b
    #                 except nx.exception.NetworkXNoPath:
    #                     c = np.inf
    #             else:
    #                 c = 0
    #             min_qb_dict[(node1, node2)] = c
    #
    #     return min_qb_dict

    def FeasAcpt(self, min_qb):
        """
        delte infeasible final state
        :param buchi_graph: buchi automaton
        :param min_qb: dict of pairs of node : length of path
        """
        accept = self.buchi_graph.graph['accept']
        for acpt in accept:
            if min_qb[(self.buchi_graph.graph['init'][0], acpt)] == np.inf or min_qb[(acpt, acpt)] == np.inf:
                self.buchi_graph.graph['accept'].remove(acpt)