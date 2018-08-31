"""
Tree construction for TL-RRT
"""

from random import uniform
from networkx.classes.digraph import DiGraph
from networkx.algorithms import dfs_labeled_edges
import math
import numpy as np
from collections import OrderedDict


class tree(object):
    """ construction of prefix and suffix tree
    """
    def __init__(self, n_robot, acpt, ts, buchi_graph, init, seg, step_size, no):
        """
        :param acpt:  accepting state
        :param ts: transition system
        :param buchi_graph:  Buchi graph
        :param init: product initial state
        """
        self.robot = n_robot
        self.acpt = acpt
        self.goals = []
        self.ts = ts
        self.buchi_graph = buchi_graph
        self.init = init
        self.seg = seg
        self.step_size = step_size
        self.dim = len(self.ts['workspace'])
        uni_ball = [1, 2, 3.142, 4.189, 4.935, 5.264, 5.168, 4.725, 4.059, 3.299, 2.550]
        self.gamma = np.ceil(4 * np.power(1/uni_ball[self.robot*self.dim], 1./(self.dim*self.robot)))   # unit workspace
        self.tree = DiGraph(type='PBA', init=init)

        label = []
        for i in range(self.robot):
            l = self.label(init[0][i])
            # exists one sampled point lies within obstacles
            if l != '':
                l = l + '_' + str(i+1)
            label.append(l)

        self.tree.add_node(init, cost=0, label=label)
        self.no = no

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
        :param: x_rand randomly sampled point form: single point ()
        :return: nearest vertex form: single point ()
        """
        # min_dis = math.inf
        # x_nearest = x_rand
        # for vertex in self.tree.nodes:
        #     x_vertex = self.mulp2sglp(vertex[0])
        #     dis = np.linalg.norm(np.subtract(x_rand, x_vertex))
        #     if dis < min_dis:
        #         x_nearest = x_vertex
        #         min_dis = dis
        # return x_nearest

        min_dis = math.inf
        q_nearest = []
        for vertex in self.tree.nodes:
            x_vertex = self.mulp2sglp(vertex[0])
            dis = np.linalg.norm(np.subtract(x_rand, x_vertex))
            if dis < min_dis:
                q_nearest = list()
                q_nearest.append(vertex)
                min_dis = dis
            elif dis == min_dis:
                q_nearest.append(vertex)
        return q_nearest

    def steer(self, x_rand, x_nearest):
        """
        steer
        :param: x_rand randomly sampled point form: single point ()
        :param: x_nearest nearest point in the tree form: single point ()
        :return: new point single point ()
        """
        #return np.asarray([0.8,0.4])
        if np.linalg.norm(np.subtract(x_rand, x_nearest)) <= self.step_size:
            return x_rand
        else:
            return tuple(np.asarray(x_nearest) + self.step_size * (np.subtract(x_rand, x_nearest))/np.linalg.norm(np.subtract(x_rand, x_nearest)))

    def extend(self, q_new, q_nearest, label, obs_check):
        for v_nearest in q_nearest:
            # print(v_nearest)
            if obs_check[(q_new[0], v_nearest[0])] and self.checkTranB(v_nearest[1],
                                                                       self.tree.nodes[v_nearest]['label'], q_new[1]):
                if 'T1_S4' in v_nearest:
                    print(self.tree.nodes[v_nearest]['label'])
                    if 'l3_1' in self.tree.nodes[v_nearest]['label']:
                        print(2)
                cost = self.tree.nodes[v_nearest]['cost'] + np.linalg.norm(np.subtract(self.mulp2sglp(q_new[0]),self.mulp2sglp(v_nearest[0])))

                self.tree.add_node(q_new, cost=cost, label=label)
                self.tree.add_edge(v_nearest, q_new)
                if self.seg == 'pre' and q_new[1] in self.acpt:
                    q_n = list(list(self.tree.pred[q_new].keys())[0])
                    cost = self.tree.nodes[tuple(q_n)]['cost']
                    label = self.tree.nodes[tuple(q_n)]['label']
                    q_n[1] = q_new[1]
                    q_n = tuple(q_n)
                    self.tree.add_node(q_n, cost=cost, label=label)
                    self.tree.add_edge(v_nearest, q_n)
                    self.goals.append(q_n)
                    # self.goals.append(q_new)
                # if self.seg == 'suf' and self.init in near_v and obs_check[(q_new[0], self.init[0])]  and  self.checkTranB(q_new[1], label, self.init[1]):
                if self.seg == 'suf' and self.obs_check([self.init], q_new[0], label, 'final')[
                    (q_new[0], self.init[0])] and self.checkTranB(q_new[1], label, self.init[1]):
                    self.goals.append(q_new)
                break

    def obs_check(self, q_near, x_new, label, stage):
        """
        check whether obstacle free along the line from x_near to x_new
        :param q_near: states in the near ball, tuple (mulp, buchi)
        :param x_new: new state form: multiple point
        :return: dict (x_near, x_new): true (obs_free)
        """

        obs_check_dict = {}
        for x in q_near:
            obs_check_dict[(x_new, x[0])] = True
            flag = True       # indicate whether break and jump to outer loop
            for r in range(self.robot):
                for i in range(1, 11):
                    mid = tuple(np.asarray(x[0][r]) + i/10. * np.subtract(x_new[r], x[0][r]))
                    mid_label = self.label(mid)
                    if mid_label != '':
                        mid_label = mid_label +  '_' + str(r+1)
                    if stage == 'reg' and ('o' in mid_label or (mid_label != self.tree.nodes[x]['label'][r] and mid_label != label[r])):
                        # obstacle             pass through one region more than once
                        obs_check_dict[(x_new, x[0])] = False
                        flag = False
                        break
                    elif stage == 'final' and ('o' in mid_label or (mid_label != self.tree.nodes[x]['label'][r] and mid_label != label[r] and mid_label in self.no)):
                        obs_check_dict[(x_new, x[0])] = False
                        flag = False
                        break
                if not flag:
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
            elif obs[1] == 'p':
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
            elif regions[1] == 'p':
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
                a = a.strip('(')
                a = a.strip(')')
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
            # either one of || holds
            if t_s_b:
                return t_s_b
        return t_s_b

    def findpath(self, goals):
        """
        find the path backwards
        :param goal: goal state
        :return: dict path : cost
        """
        paths = OrderedDict()
        for i in range(len(goals)):
            goal = goals[i]
            path = [goal]
            s = goal
            while s != self.init:
                s = list(self.tree.pred[s].keys())[0]
                path.insert(0, s)
            if self.seg == 'pre':
                paths[i] = [self.tree.nodes[goal]['cost'], path]
            elif self.seg == 'suf':
                # path.append(self.init)
                paths[i] = [self.tree.nodes[goal]['cost'] + np.linalg.norm(np.subtract(goal[0], self.init[0])), path]
        return paths

    def mulp2sglp(self, point):
        """
        convert multiple form point ((),(),(),...) to single form point ()
        :param point: multiple points ((),(),(),...)
        :return: signle point ()
        """
        sp = []
        for p in point:
            sp = sp + list(p)
        return tuple(sp)

    def sglp2mulp(self, point):
        """
        convert single form point () to multiple form point ((), (), (), ...)
        :param point: single form point ()
        :return:  multiple form point ((), (), (), ...)
        """
        mp = []
        for i in range(self.robot):
            mp.append(point[i*self.dim :(i+1)*self.dim])
        return tuple(mp)

def construction_tree(tree, buchi_graph, n_max):
    sz = [0]
    # trival suffix path
    if tree.seg == 'suf' and tree.checkTranB(tree.init[1], tree.tree.nodes[tree.init]['label'], tree.init[1]):
        return {0:[0, []]}, sz

    for n in range(n_max):

        # sample form: multiple
        x_rand = list()
        for i in range(tree.robot):
            x_rand.append(tree.sample())
        x_rand = tree.mulp2sglp(tuple(x_rand))

        q_nearest = tree.nearest(x_rand)
        x_new = tree.steer(x_rand, tree.mulp2sglp(q_nearest[0][0]))


        # label
        label = []
        o_id = True
        x_new = tree.sglp2mulp(x_new)
        for i in range(tree.robot):
            l = tree.label(x_new[i])
            # exists one sampled point lies within obstacles
            if 'o' in l:
                o_id = False
                break
            if l != '':
                l = l + '_' + str(i+1)
                # print(l)
            label.append(l)
        if not o_id:
            continue

        # check obstacle free
        obs_check = tree.obs_check([q_nearest[0]], x_new, label, 'reg')

        # iterate over each buchi state
        for b_state in buchi_graph.nodes:

            # new product state
            q_new = (x_new, b_state)

            # extend
            tree.extend(q_new, q_nearest, label, obs_check)

        # number of nodes
        sz.append(tree.tree.number_of_nodes())
        # first accepting state
        if len(tree.goals):
            break

    cost_path = tree.findpath(tree.goals)
    return cost_path, sz