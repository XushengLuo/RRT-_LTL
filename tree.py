from random import uniform
from networkx.classes.digraph import DiGraph
import math
import numpy as np
from collections import OrderedDict

class tree(object):
    """ construction of prefix and suffix tree
    """
    def __init__(self, acpt, ts, buchi_graph, init, seg, step_size):
        """
        :param acpt:  accepting state
        :param ts: transition system
        :param buchi_graph:  Buchi graph
        :param init: product initial state
        """
        self.acpt = acpt
        self.goals = []
        self.ts = ts
        self.buchi_graph = buchi_graph
        self.init = init
        self.seg = seg
        self.step_size = step_size
        self.dim = len(self.ts['workspace'])
        self.gamma = np.ceil(4 * np.power(1/3.14, 1./self.dim))   # unit workspace
        self.tree = DiGraph(type='PBA')
        self.tree.add_node(init, cost=0, label=self.label(init[0]))

    def sample(self):
        """
        sample point from the workspace
        :return: sampled point, tuple
        """
        x_rand = []
        for i in range(self.dim):
            x_rand.append(uniform(0, self.ts['workspace'][i]))

        return tuple(x_rand)

    def nearest(self, x_rand):
        """
        find the nearest vertex in the tree
        :return: nearest vertex
        """
        min_dis = math.inf
        x_nearest = x_rand
        for vertex in self.tree.nodes:
            dis = np.linalg.norm(np.subtract(x_rand, vertex[0]))
            if dis < min_dis:
                x_nearest = vertex[0]
                min_dis = dis
        return x_nearest

    def steer(self, x_rand, x_nearest):
        """
        steer
        :return: new point, array type
        """
        #return np.asarray([0.8,0.4])
        if np.linalg.norm(np.subtract(x_rand, x_nearest)) <= self.step_size:
            return x_rand
        else:
            return tuple(np.asarray(x_nearest) + self.step_size * (np.subtract(x_rand, x_nearest)))

    def extend(self, q_new, near_v, label, obs_check):
        """
        :param: q_new: new state
        :param: near_v: near state
        :param: obs_check: check obstacle free
        :return: extending the tree
        """
        added = 0
        cost = np.inf
        q_min = ()
        for near_vertex in near_v:
            if obs_check[(q_new[0], near_vertex[0])] and self.checkTranB(near_vertex[1], self.tree.nodes[near_vertex]['label'], q_new[1]):
                c = self.tree.nodes[near_vertex]['cost'] + np.linalg.norm(np.subtract(q_new[0], near_vertex[0]))      # don't consider control
                if c < cost:
                    added = 1
                    q_min = near_vertex
                    cost = c
        if added == 1:
            self.tree.add_node(q_new, cost = cost, label=label)
            self.tree.add_edge(q_min, q_new)
            if self.seg == 'pre' and q_new[1] in self.acpt:
                self.goals.append(q_new)
            if self.seg == 'suf' and self.init in near_v and obs_check[(q_new[0], self.init[0])]  and  self.checkTranB(q_new[1], label, self.init[1]):
                self.goals.append(q_new)
        return added

    def rewire(self, q_new, near_v, obs_check):
        """
        :param: q_new: new state
        :param: near_v: near state
        :param: obs_check: check obstacle free
        :return: rewiring the tree
        """
        for near_vertex in near_v:
            if obs_check[(q_new[0], near_vertex[0])] and self.checkTranB(q_new[1], self.tree.nodes[q_new]['label'], near_vertex[1]):
                c = self.tree.nodes[q_new]['cost'] + np.linalg.norm(np.subtract(q_new[0], near_vertex[0]))      # without considering control
                if self.tree.nodes[near_vertex]['cost'] > c:
                    self.tree.nodes[near_vertex]['cost'] = c
                    self.tree.remove_edge(list(self.tree.pred[near_vertex].keys())[0], near_vertex)
                    self.tree.add_edge(q_new, near_vertex)


    def near(self, x_new):
        """
        find the states in the near ball
        :param x_new: new point
        :return: p_near: near state, tuple
        """
        p_near = []
        r = min(self.gamma * np.power(np.log(self.tree.number_of_nodes()+1)/self.tree.number_of_nodes(),1./self.dim), self.step_size)
        for vertex in self.tree.nodes:
            if np.linalg.norm(np.subtract(x_new, vertex[0])) <= r:
                p_near.append(vertex)

        return p_near


    def obs_check(self, q_near, x_new, label):
        """
        check whether obstacle free along the line from x_near to x_new
        :param q_near: states in the near ball, tuple
        :param q_new: new product state, tuple
        :return: dict (x_near, x_new): true (obs_free)
        """
        obs_check_dict = {}
        for x in q_near:
            obs_check_dict[(x_new, x[0])] = True
            for i in range(1, 11):
                mid = tuple(np.asarray(x[0]) + i/10. * np.subtract(x_new, x[0]))
                mid_label = self.label(mid)
                if 'o' in mid_label or (mid_label != self.tree.nodes[x]['label'] and mid_label != label):
                    # obstacle             pass through one region more than once
                    obs_check_dict[(x_new, x[0])] = False
                    break

        return obs_check_dict



    def label(self, x):
        """
        generating the label of position state
        :param x: position
        :return: label
        """
        # whether x lies within obstacle
        for (obs, boundary) in iter(self.ts['obs'].items()):
            if obs[1] == 'b' and np.linalg.norm(np.subtract(x, boundary[0:-1])) <= boundary[-1]:
                return obs[0]
            elif obs[1] == 's':
                dictator = True
                for i in range(len(boundary)):
                    if np.dot(x, boundary[i][0:-1]) + boundary[i][-1] > 0:
                        dictator = False
                        break
                if dictator == True:
                    return obs[0]


        # whether x lies within regions
        for (regions, boundary) in iter(self.ts['region'].items()):
            if regions[1] == 'b' and np.linalg.norm(x - np.asarray(boundary[0:-1])) <= boundary[-1]:
                return regions[0]
            elif regions[1] == 's':
                dictator = True
                for i in range(len(boundary)):
                    if np.dot(x, np.asarray(boundary[i][0:-1])) + boundary[i][-1] > 0:
                        dictator = False
                        break
                if dictator == True:
                    return regions[0]

        return ''

    def checkTranB(self, b_state, x_label, q_b_new):
        """ decide valid transition, whether b_state --L(x)---> q_b_new
             Algorithm2 in Chapter 2 Motion and Task Planning
             :param b_state: buchi state
             :param x_label: label of x
             :param q_b_new buchi state
             :return True satisfied
        """
        b_state_succ = self.buchi_graph.succ[b_state]
        # q_b_new is not the successor of b_state
        if q_b_new not in b_state_succ:
             return False

        b_label = self.buchi_graph.edges[(b_state, q_b_new)]['label']
        if self.t_satisfy_b(x_label, b_label):
            return True


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

    def findpath(self, goals):
        """
        find the path backwards
        :param goal: goal state
        :return: dict path : cost
        """
        paths= OrderedDict()
        for goal in goals:
            path = [goal]
            s = goal
            while s!= self.init:
                s = list(self.tree.pred[s].keys())[0]
                path.insert(0, s)
            if self.seg == 'pre':
                paths[self.tree.nodes[goal]['cost']] = path
            elif self.seg == 'suf':
                paths[self.tree.nodes[goal]['cost'] + np.linalg.norm(np.subtract(goal[0], self.init[0]))] = path
        return paths


def construction_tree(tree, buchi_graph, n_max):
    for n in range(n_max):
        # sample
        x_rand = tree.sample()
        # nearest
        x_nearest = tree.nearest(x_rand)
        # steer
        x_new = tree.steer(x_rand, x_nearest)
        # label
        label = tree.label(x_new)
        # sampled point lies within obstacles
        if 'o' in label:
            continue

        # near state
        near_v = tree.near(x_new)
        # check obstacle free
        obs_check = tree.obs_check(near_v, x_new, label)

        # iterate over each buchi state
        for b_state in buchi_graph.nodes:
            # new product state
            q_new = (x_new, b_state)
            # extend
            added = tree.extend(q_new, near_v, label, obs_check)
            # rewire
            if added == 1:
                tree.rewire(q_new, near_v, obs_check)

    cost_path = tree.findpath(tree.goals)
    return cost_path