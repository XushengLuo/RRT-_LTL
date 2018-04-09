from random import uniform
from networkx.classes.digraph import DiGraph
import math
import numpy as np

class construction_tree(object):
    """ construction of prefix and suffix tree
    """
    def __init__(self, acpt, ts, buchi_graph, init, para):
        """
        :param acpt:  accepting state
        :param ts: transition system
        :param buchi_graph:  Buchi graph
        :param init: product initial state
        :param n_pre: maximum prefix iteration
        """
        self.acpt = acpt
        self.ts = ts
        self.buchi_graph = buchi_graph
        self.init = init
        self.step_size = para[0]
        self.dim = len(self.ts['workspace'])
        self.gamma = np.ceil(4 * np.power(1/3.14, 1./self.dim))   # unit workspace
        self.n_pre = para[1]
        self.tree = DiGraph(type='PBA')
        self.tree.add_node((np.asarray(init[0]), init[1]), cost=0)

    def sample(self):
        """
        sample point from the workspace
        :return: sampled point, array type
        """
        x_rand = np.empty([1, self.dim])
        for i in range(1, self.dim):
            x_rand[i-1] =  uniform(0, self.ts['workspace'][i])

        return x_rand

    def nearest(self, x_rand):
        """
        find the nearest vertex in the tree
        :return: nearest vertex
        """
        min_dis = math.inf
        x_nearest = x_rand
        for vertex in self.tree.nodes:
            dis = np.linalg.norm(x_rand - vertex[0])
            if dis < min_dis:
                x_nearest = vertex[0]
                min_dis = dis
        return x_nearest

    def steer(self, x_rand, x_nearest):
        """
        steer
        :return: new point, array type
        """
        if np.linalg.norm(x_rand - x_nearest) <= self.step_size:
            return x_rand
        else:
            return x_nearest + self.step_size * (x_rand - x_nearest)

    def extend(self, q_new, near_v, label, obs_check):
        """
        :param: q_new: new state
        :param: near_v: near state
        :param: obs_check: check obstacle free
        :return: extending the tree
        """
        added = 0
        cost = np.inf
        q_min = []
        for near_vertex in near_v:
            if obs_check[(q_new[0], near_vertex[0])] and self.checkTranB(near_vertex[1], self.tree.nodes[near_vertex[0]]['label'], q_new[1]):
                c = self.tree.nodes[near_vertex]['cost'] + np.linalg.norm(q_new[0] - near_vertex[0])      # don't consider control
                if c <= cost:
                    added = 1
                    q_min = list(near_vertex)
                    cost  = c
        if added == 1:
            self.tree.add_node(q_new, cost = cost, label=label)
            self.tree.add_edge(tuple(q_min), q_new)

        return added

    def rewire(self, q_new, near_v, obs_check):
        """
        :param: q_new: new state
        :param: near_v: near state
        :param: obs_check: check obstacle free
        :return: rewiring the tree
        """
        for near_vertex in near_v:
            if obs_check[(q_new[0], near_vertex[0])] and self.checkTranB(q_new[1], self.tree.nodes[q_new[0]]['label'], near_vertex[1]):
                c = self.tree.nodes[near_vertex]['cost'] + np.linalg.norm(q_new[0] - near_vertex[0])      # without considering control
                if self.tree.nodes[near_vertex]['cost'] > c:
                    self.tree.nodes[near_vertex]['cost'] = c
                    self.tree.remove_edge(self.tree.pred[near_vertex], near_vertex)
                    self.tree.add_edge(q_new, near_vertex)


    def near(self, q_new):
        """
        find the states in the near ball
        :param q_new: new product state
        :return: p_near: near state
        """
        p_near = []
        r = min(self.gamma * np.power(np.log(self.tree.number_of_nodes())/self.tree.number_of_nodes(),1./self.dim), self.step_size)
        for vertex in self.tree.nodes:
            if np.linalg.norm(q_new[0] - vertex[0]) <= r:
                p_near.append(vertex)

        return p_near


    def obs_check(self, x_near, x_new):
        """
        check whether obstacle free along the line from x_near to x_new
        :param x_near: position in the near ball
        :param x_new: new position
        :return: dict (x_near, x_new): true (obs_free)
        """
        obs_check_dict = {}
        for x in x_near:
            for i in range(1, 11):
                mid = x + i * (x_new - x)
                mid_label = self.label(mid)
                if 'o' in mid_label or (mid_label != self.tree.nodes[x_near]['label'] and mid_label != self.tree.nodes[x_new]['label']):
                    # obstacle             pass through one region more than once
                    obs_check_dict[(x_new, x)] = False
                    break

            obs_check_dict[(x_new, x)] = True
        return obs_check_dict



    def label(self, x):
        """
        generating the label of position state
        :param x: position
        :return: label
        """
        # whether x lies within obstacle
        for (obs, boundary) in iter(self.ts['obs'].items()):
            if obs[1] == 's':
                for i in range(0, len(boundary)):
                    if np.dot(x, boundary[i]) > 0:
                        break
                return obs[0]
            elif obs[1] == 'b' and np.linalg.norm(x - boundary[0:-1]) <= boundary[-1]:
                return obs[0]

        # whether x lies within regions
        for (regions, boundary) in iter(self.ts['region'].items()):
            if regions[1] == 's':
                for i in range(0, len(boundary)):
                    if np.dot(x, boundary[i]) > 0:
                        break
                return regions[0]
            elif regions[1] == 'b' and np.linalg.norm(x - boundary[0:-1]) <= boundary[-1]:
                return regions[0]

        return ''

    def checkTranB(self, b_state, x_label, q_b_new):
        """ decide valid transition, whether b_state --L(x)---> q_b_new
             Algorithm2 in Chapter 2 Motion and Task Planning
             :param b_state: buchi state
             :param x_label: label of x
             :param q_b_new buchi state
             :return d: -1 not satisdied; 0 satisfied
        """
        d = -1
        b_state_succ = self.buchi_graph.succ[b_state]
        # q_b_new is not the successor of b_state
        if q_b_new not in b_state_succ:
             return d

        b_label = self.buchi_graph.edges[(b_state, q_b_new)]['label']
        if self.t_satisfy_b(x_label, b_label):
           d = 0
           return d


    def t_satisfy_b(self, x_label, b_label):
        """ decide whether label of self.ts_graph can satisfy label of self.buchi_graph
            :param x_label: label of x
            :param b_label: label of buchi state
            :return t_s_b: true if satisfied
        """
        t_s_b = True
        # split label with ||
        b_label = b_label.split('||')
        for label in b_label:
            t_s_b = True
            # spit label with &&
            atomic_label = label.split('&&')
            for a in atomic_label:
                a = a.strip()
                if a == '1':
                    continue
                # whether ! in an atomic proposition
                if '!' in a:
                    if a[1:] in x_label:
                        t_s_b = False
                        break
                else:
                    if not a in x_label:
                       t_s_b = False
                       break
        return t_s_b

