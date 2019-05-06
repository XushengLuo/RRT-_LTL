"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 8/30/18
"""
"""
construct trees for biased sampling optimal task planning for multi-robots
"""

from random import uniform
from networkx.classes.digraph import DiGraph
from networkx.algorithms import dfs_labeled_edges
import math
import numpy as np
from scipy.stats import truncnorm
from collections import OrderedDict
import pyvisgraph as vg
from shapely.geometry import Point, Polygon, LineString
from uniform_geometry import sample_uniform_geometry


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
        # uni_v = uni_ball[self.robot*self.dim]
        uni_v = np.power(np.pi, self.robot*self.dim/2) / math.gamma(self.robot*self.dim/2+1)
        self.gamma = np.ceil(4 * np.power(1/uni_v, 1./(self.dim*self.robot)))   # unit workspace
        self.tree = DiGraph(type='PBA', init=init)
        self.group = dict()
        label = []
        for i in range(self.robot):
            l = self.label(init[0][i])
            # exists one sampled point lies within obstacles
            if l != '':
                l = l + '_' + str(i+1)
            label.append(l)

        self.tree.add_node(init, cost=0, label=label)
        self.add_group(init)
        # probability
        self.p = 0.9
        # threshold for collision avoidance
        self.threshold = 0.01
        # polygon obstacle
        polys = [[vg.Point(0.4, 1.0), vg.Point(0.4, 0.7), vg.Point(0.6, 0.7), vg.Point(0.6, 1.0)],
                 [vg.Point(0.3, 0.2), vg.Point(0.3, 0.0), vg.Point(0.7, 0.0), vg.Point(0.7, 0.2)]]
        self.g = vg.VisGraph()
        self.g.build(polys, status=False)
        # region that has ! preceding it
        self.no = no

        # biased sampling
        # self.acp = np.random.randint(0, len(buchi_graph.graph['accept']))

    def add_group(self, q_state):
        """
        group nodes with same buchi state
        :param q_state: new state added to the tree
        """
        try:
            self.group[q_state[1]].append(q_state)
        except KeyError:
            self.group[q_state[1]] = [q_state]

    def min2final(self, min_qb_dict, b_final, cand):
        """
         collects the buchi state in the tree with minimum distance to the final state
        :param min_qb_dict: dict
        :param b_final: feasible final state
        :return: list of buchi states in the tree with minimum distance to the final state
        """
        l_min = np.inf
        b_min = []
        for b_state in cand:
            if min_qb_dict[(b_state, b_final)] < l_min:
                l_min = min_qb_dict[(b_state, b_final)]
                b_min = [b_state]
            elif min_qb_dict[(b_state, b_final)] == l_min:
                b_min.append(b_state)
        return b_min

    def all2one(self, b_min):
        """
        partition nodes into 2 groups
        :param b_min: buchi states with minimum distance to the finals state
        :return: 2 groups
        """
        q_min2final = []
        q_minNot2final = []
        for b_state in self.group.keys():
            if b_state in b_min:
                q_min2final = q_min2final + self.group[b_state]
            else:
                q_minNot2final = q_minNot2final + self.group[b_state]
        return q_min2final, q_minNot2final

    def get_truncated_normal(self, mean=0, sd=1, low=0, upp=10):
        return truncnorm(
            (low - mean) / sd, (upp - mean) / sd, loc=mean, scale=sd)

    def trunc(self, value):
        if value < 0:
            return 0
        elif value > 1:
            return 1
        else:
            return value

    def collision_avoidance(self, x, index):
        """
        check whether any robots are collision-free from index-th robot
        :param x: all robots
        :param index: index-th robot
        :return: true collision free
        """
        for i in range(len(x)):
            # if i != index and np.linalg.norm(np.subtract(x[i], x[index])) <= self.threshold:
            if i != index and np.fabs(x[i][0]-x[index][0]) <= self.threshold and np.fabs(x[i][1]-x[index][1]) <= self.threshold:
                return False
        return True

    def target(self, init, target, regions):
        """
        find the closest vertex in the short path from init to target
        :param init: inital point
        :param target: target labeled region
        :param regions: regions
        :return: closest vertex
        """
        tg = regions[target].centroid.coords[0]
        shortest = self.g.shortest_path(vg.Point(init[0], init[1]), vg.Point(tg[0], tg[1]))
        return (shortest[1].x, shortest[1].y)

    def gaussian_guided(self, x, target):
        """
        calculate new point following gaussian dist guided by the target
        :param x: mean point
        :param target: target point
        :return: new point
        """

        d = self.get_truncated_normal(0, 1/3, 0, np.inf)
        d = d.rvs()
        # d = np.linalg.norm(np.subtract(x, target))
        angle = np.random.normal(0, np.pi/12/3/3, 1) + np.arctan2(target[1] - x[1], target[0] - x[0])
        # angle = np.arctan2(target[1] - x[1], target[0] - x[0])
        x_rand = np.add(x , np.append(d*np.cos(angle), d*np.sin(angle)))
        x_rand = [self.trunc(x) for x in x_rand]
        return tuple(x_rand)

    def gaussian_unguided(self, x):
        """

        :param x:
        :return:
        """
        d = self.get_truncated_normal(0, min(
            self.gamma * np.power(np.log(self.tree.number_of_nodes() + 1) / self.tree.number_of_nodes(),
                                  1. / (self.dim * self.robot)), self.step_size)/3, 0)
        d = d.rvs()
        angle = np.random.uniform(-np.pi, np.pi, 1)
        x_rand = np.add(x, np.append(d * np.cos(angle), d * np.sin(angle)))
        return tuple([self.trunc(x) for x in x_rand])

    def buchi_guided_sample_by_label(self, x_rand, b_label, x_label, regions):

        if b_label.strip().strip('(').strip(')') == '1':
            return []
        # not or be in some place
        else:
            # label of current position
            blabel = b_label.split('||')[0]
            # blabel = random.choice(b_label.split('||'))
            atomic_label = blabel.split('&&')
            for a in atomic_label:
                # a = a.strip().strip('(').strip(')')
                a = a.strip().strip('(').strip(')').split('or')[0].strip()
                # if in wrong position, sample randomly
                if '!' in a:
                    if a[1:] in x_label:
                        xi_rand = []
                        for i in range(self.dim):
                            xi_rand.append(uniform(0, self.ts['workspace'][i]))
                        ind = int(a[1:].split('_')[1]) - 1
                        x_rand[ind] = tuple(xi_rand)
                else:
                    # move towards target position
                    if not a in x_label:
                        ind = a.split('_')
                        weight = 0.8
                        if np.random.uniform(0, 1, 1) <= weight:
                            tg = self.target(x_rand[int(ind[1]) - 1], ind[0], regions)
                            x_rand[int(ind[1]) - 1] = self.gaussian_guided(x_rand[int(ind[1]) - 1], tg)
                        else:
                            xi_rand = []
                            for i in range(self.dim):
                                xi_rand.append(uniform(0, self.ts['workspace'][i]))
                            x_rand[int(ind[1]) - 1] = tuple(xi_rand)

        return x_rand

    def buchi_guided_sample_by_truthvalue(self, truth, x_rand, q_rand, x_label, regions):
        """
        sample guided by truth value
        :param truth: the value making transition occur
        :param x_rand: random selected node
        :param x_label: label of x_rand
        :param regions: regions
        :return: new sampled point
        """
        if truth == '1':
            return [], []
        # not or be in some place
        else:
            for key in truth:
                # if in wrong position, sample randomly
                # if not truth[key] and key in x_label:
                #     xi_rand = []
                #     for i in range(self.dim):
                #         xi_rand.append(uniform(0, self.ts['workspace'][i]))
                #     ind = key.split('_')
                #     x_rand[int(ind[1]) - 1] = tuple(xi_rand)
                # elif truth[key]:
                #     # move towards target position
                #     if not key in x_label:
                #         ind = key.split('_')
                #         weight = 1
                #         if np.random.uniform(0, 1, 1) <= weight:
                #             tg = self.target(x_rand[int(ind[1]) - 1], ind[0], regions)
                #             # tg = self.target(orig_x_rand, ind[0], regions)
                #             x_rand[int(ind[1]) - 1] = self.gaussian_guided(x_rand[int(ind[1]) - 1], tg)
                #         else:
                #             xi_rand = []
                #             for i in range(self.dim):
                #                 xi_rand.append(uniform(0, self.ts['workspace'][i]))
                #             x_rand[int(ind[1]) - 1] = tuple(xi_rand)
                ind = key.split('_')
                orig_x_rand = x_rand[int(ind[1]) - 1]  # save
                while 1:
                    x_rand[int(ind[1]) - 1] = orig_x_rand  # recover
                    # if in wrong position, sample randomly
                    if not truth[key] and key in x_label:
                        xi_rand = []
                        for i in range(self.dim):
                            xi_rand.append(uniform(0, self.ts['workspace'][i]))
                        # ind = key.split('_')
                        x_rand[int(ind[1])-1] = tuple(xi_rand)
                    elif truth[key]:
                        # move towards target position
                        if not key in x_label:
                            # ind = key.split('_')
                            weight = 0.8
                            if np.random.uniform(0, 1, 1) <= weight:
                                # tg = self.target(x_rand[int(ind[1]) - 1], ind[0], regions)
                                tg = self.target(orig_x_rand, ind[0], regions)
                                x_rand[int(ind[1]) - 1] = self.gaussian_guided(orig_x_rand, tg)
                            else:
                                xi_rand = []
                                for i in range(self.dim):
                                    xi_rand.append(uniform(0, self.ts['workspace'][i]))
                                x_rand[int(ind[1]) - 1] = tuple(xi_rand)
                    else:
                        break
                    if self.collision_avoidance(x_rand, int(ind[1]) - 1):
                        break


            #   x_rand                  x_nearest
        return self.mulp2sglp(x_rand), q_rand
        # return x_rand

    def sample(self, buchi_graph, min_qb_dict, regions):
        """
        sample point from the workspace
        :return: sampled point, tuple
        """
        if self.seg == 'pre':
            b_final = buchi_graph.graph['accept'][np.random.randint(0, len(buchi_graph.graph['accept']))]     # feasible final buchi state
            # b_final = buchi_graph.graph['accept'][self.acp]
        else:
            b_final = buchi_graph.graph['accept']
        # collects the buchi state in the tree with minimum distance to the final state
        b_min = self.min2final(min_qb_dict, b_final, self.group.keys())
        # partition of nodes
        q_min2final, q_minNot2final = self.all2one(b_min)
        # sample random nodes
        p_rand = np.random.uniform(0,1,1)
        if (p_rand <= self.p and len(q_min2final) > 0) or not q_minNot2final:
            q_rand = sample_uniform_geometry(q_min2final)
            # q_rand = q_min2final[np.random.randint(0, len(q_min2final))]
        elif p_rand > self.p or not q_min2final:
            q_rand = sample_uniform_geometry(q_minNot2final)
            # q_rand = q_minNot2final[np.random.randint(0, len(q_minNot2final))]
        # find feasible succssor of buchi state in q_rand
        Rb_q_rand = []
        x_label = []
        for i in range(self.robot):
            l = self.label(q_rand[0][i])
            if l != '':
                l = l + '_' + str(i + 1)
            x_label.append(l)

        for b_state in buchi_graph.succ[q_rand[1]]:
            # if self.t_satisfy_b(x_label, buchi_graph.edges[(q_rand[1], b_state)]['label']):
            if self.t_satisfy_b_truth(x_label, buchi_graph.edges[(q_rand[1], b_state)]['truth']):
                Rb_q_rand.append(b_state)
        # if empty
        if not Rb_q_rand:
            return Rb_q_rand, Rb_q_rand
        # collects the buchi state in the reachable set of qb_rand with minimum distance to the final state
        b_min = self.min2final(min_qb_dict, b_final, Rb_q_rand)

        # collects the buchi state in the reachable set of b_min with distance to the final state equal to that of b_min - 1
        decr_dict = dict()
        for b_state in b_min:
            decr = []
            for succ in buchi_graph.succ[b_state]:
                if min_qb_dict[(b_state, b_final)] - 1 == min_qb_dict[(succ, b_final)] or succ in buchi_graph.graph['accept']:
                    decr.append(succ)
            decr_dict[b_state] = decr
        M_cand = [b_state for b_state in decr_dict.keys() if decr_dict[b_state]]
        # if empty
        if not M_cand:
            return M_cand, M_cand
        # sample b_min and b_decr
        b_min = sample_uniform_geometry(M_cand)
        # b_min = M_cand[np.random.randint(0, len(M_cand))]
        b_decr = sample_uniform_geometry(decr_dict[b_min])
        # b_decr = decr_dict[b_min][np.random.randint(0, len(decr_dict[b_min]))]


        # b_label = buchi_graph.edges[(b_min, b_decr)]['label']
        # x_rand = list(q_rand[0])
        #
        # return self.buchi_guided_sample_by_label(x_rand, b_label, x_label, regions)

        truth = buchi_graph.edges[(b_min, b_decr)]['truth']
        # print(truth)
        x_rand = list(q_rand[0])
        return self.buchi_guided_sample_by_truthvalue(truth, x_rand, q_rand, x_label, regions)


            #   x_rand                  x_nearest
        # return self.mulp2sglp(x_rand), self.mulp2sglp(q_rand[0])

        # return x_rand

    # def nearest(self, x_rand):
    #     """
    #     find the nearest vertex in the tree
    #     :param: x_rand randomly sampled point form: single point ()
    #     :return: nearest vertex form: single point ()
    #     """
    #     min_dis = math.inf
    #     x_nearest = x_rand
    #     for vertex in self.tree.nodes:
    #         x_vertex = self.mulp2sglp(vertex[0])
    #         dis = np.linalg.norm(np.subtract(x_rand, x_vertex))
    #         if dis < min_dis:
    #             x_nearest = x_vertex
    #             min_dis = dis
    #     return x_nearest

    def nearest(self, x_rand):
        """
        find the nearest class of vertices in the tree
        :param: x_rand randomly sampled point form: single point ()
        :return: nearest class of vertices form: single point ()
        """
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

    def extend(self, q_new, near_v, label, obs_check):
        """
        :param: q_new: new state form: tuple (mulp, buchi)
        :param: near_v: near state form: tuple (mulp, buchi)
        :param: obs_check: check obstacle free  form: dict { (mulp, mulp): True }
        :return: extending the tree
        """
        added = 0
        cost = np.inf
        q_min = ()
        for near_vertex in near_v:
            if q_new != near_vertex and obs_check[(q_new[0], near_vertex[0])] and self.checkTranB(near_vertex[1], self.tree.nodes[near_vertex]['label'], q_new[1]):
                c = self.tree.nodes[near_vertex]['cost'] + np.linalg.norm(np.subtract(self.mulp2sglp(q_new[0]), self.mulp2sglp(near_vertex[0])))      # don't consider control
                if c < cost:
                    added = 1
                    q_min = near_vertex
                    cost = c
        if added == 1:
            self.tree.add_node(q_new, cost = cost, label=label)
            self.tree.add_edge(q_min, q_new)
            self.add_group(q_new)
            if self.seg == 'pre' and q_new[1] in self.acpt:
                q_n = list(list(self.tree.pred[q_new].keys())[0])
                cost = self.tree.nodes[tuple(q_n)]['cost']
                label = self.tree.nodes[tuple(q_n)]['label']
                q_n[1] = q_new[1]
                q_n = tuple(q_n)
                self.tree.add_node(q_n, cost = cost, label = label)
                self.tree.add_edge(q_min, q_n)
                self.add_group(q_n)
                self.goals.append(q_n)

            # if self.seg == 'suf' and self.checkTranB(q_new[1], label, self.init[1]):
            #     print('final')

            if self.seg == 'suf' and self.obs_check([self.init], q_new[0], label, 'final')[(q_new[0], self.init[0])] and self.checkTranB(q_new[1], label, self.init[1]):
            # if self.seg == 'suf' and self.init in near_v and obs_check[(q_new[0], self.init[0])] and self.checkTranB(q_new[1], label, self.init[1]):
                self.goals.append(q_new)
        return added

    def rewire(self, q_new, near_v, obs_check):
        """
        :param: q_new: new state form: tuple (mul, buchi)
        :param: near_v: near state form: tuple (mul, buchi)
        :param: obs_check: check obstacle free form: dict { (mulp, mulp): True }
        :return: rewiring the tree
        """
        for near_vertex in near_v:
            if obs_check[(q_new[0], near_vertex[0])] and self.checkTranB(q_new[1], self.tree.nodes[q_new]['label'], near_vertex[1]):
                c = self.tree.nodes[q_new]['cost'] + np.linalg.norm(np.subtract(self.mulp2sglp(q_new[0]), self.mulp2sglp(near_vertex[0])))      # without considering control
                delta_c = self.tree.nodes[near_vertex]['cost'] - c
                # update the cost of node in the subtree rooted at near_vertex
                if delta_c > 0:
                    # self.tree.nodes[near_vertex]['cost'] = c
                    if not list(self.tree.pred[near_vertex].keys()):
                        print('empty')
                    self.tree.remove_edge(list(self.tree.pred[near_vertex].keys())[0], near_vertex)
                    self.tree.add_edge(q_new, near_vertex)
                    edges = dfs_labeled_edges(self.tree, source=near_vertex)
                    for _, v, d in edges:
                        if d == 'forward':
                            self.tree.nodes[v]['cost'] = self.tree.nodes[v]['cost'] - delta_c

    def near(self, x_new):
        """
        find the states in the near ball
        :param x_new: new point form: single point
        :return: p_near: near state, form: tuple (mulp, buchi)
        """
        p_near = []
        r = min(self.gamma * np.power(np.log(self.tree.number_of_nodes()+1)/self.tree.number_of_nodes(),1./(self.dim*self.robot)), self.step_size)
        for vertex in self.tree.nodes:
            if np.linalg.norm(np.subtract(x_new, self.mulp2sglp(vertex[0]))) <= r:
                p_near.append(vertex)
        return p_near

    def obs_check(self, q_near, x_new, label, stage):
        """
        check whether obstacle free along the line from x_near to x_new
        :param q_near: states in the near ball, tuple (mulp, buchi)
        :param x_new: new state form: multiple point
        :param label: label of x_new
        :param stage: regular stage or final stage, deciding whether it's goal state
        :return: dict (x_near, x_new): true (obs_free)
        """

        obs_check_dict = {}
        checked = set()

        for x in q_near:
            if x[0] in checked:
                continue
            checked.add(x[0])
            obs_check_dict[(x_new, x[0])] = True
            flag = True       # indicate whether break and jump to outer loop
            for r in range(self.robot):
                # the line connecting two points crosses an obstacle
                for (obs, boundary) in iter(self.ts['obs'].items()):
                    if LineString([Point(x[0][r]), Point(x_new[r])]).intersects(boundary):
                        obs_check_dict[(x_new, x[0])] = False
                        flag = False
                        break

                if not flag:
                    break

                for (region, boundary) in iter(self.ts['region'].items()):
                    if LineString([Point(x[0][r]), Point(x_new[r])]).intersects(boundary) \
                            and region + '_' + str(r + 1) != label[r] \
                            and region + '_' + str(r + 1) != self.tree.nodes[x]['label'][r]:
                        if stage == 'reg' or (stage == 'final' and region in self.no):
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

        point = Point(x)
        # whether x lies within obstacle
        for (obs, boundary) in iter(self.ts['obs'].items()):
            if point.within(boundary):
                return obs

        # whether x lies within regions
        for (region, boundary) in iter(self.ts['region'].items()):
            if point.within(boundary):
                return region
        # x lies within unlabeled region
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

        # b_label = self.buchi_graph.edges[(b_state, q_b_new)]['label']
        # if self.t_satisfy_b(x_label, b_label):
        #     return True

        truth = self.buchi_graph.edges[(b_state, q_b_new)]['truth']
        if self.t_satisfy_b_truth(x_label, truth):
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

    def t_satisfy_b_truth(self, x_label, truth):
        """
        check whether transition enabled under current label
        :param x_label: current label
        :param truth: truth value making transition enabled
        :return: true or false
        """
        if truth == '1':
            return True

        true_label = [truelabel for truelabel in truth.keys() if truth[truelabel]]
        for label in true_label:
            if label not in x_label:
                return False

        false_label = [falselabel for falselabel in truth.keys() if not truth[falselabel]]
        for label in false_label:
            if label in x_label:
                return False

        return True

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
                if s == path[0]:
                    print("loop")
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


def construction_tree(tree, buchi_graph, min_qb_dict, regions, n_max):
    sz = [0]
    # trivial suffix path
    if tree.seg == 'suf' and tree.checkTranB(tree.init[1], tree.tree.nodes[tree.init]['label'], tree.init[1]):
        return {0:[0, []]}, sz

    # for n in range(n_max):
    while 1:
        # sample form: multiple
        x_new, q_rand = tree.sample(buchi_graph, min_qb_dict, regions)
        # x_rand, _ = tree.sample(buchi_graph, min_qb_dict, regions)
        # couldn't find
        if not x_new:
            continue
        # nearest

        # q_nearest = tree.nearest(x_new)
        # x_new = tree.steer(x_new, tree.mulp2sglp(q_nearest[0][0]))
        # steer

        x_new = tree.steer(x_new, tree.mulp2sglp(q_rand[0]))
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
        # print(label)
        # near state
        near_v = tree.near(tree.mulp2sglp(x_new))
        # add q_rand
        if q_rand not in near_v:
            near_v = near_v + [q_rand]
        # add q_nearest
        # if q_nearest[0] not in near_v:
        #     near_v = near_v + q_nearest
        # check obstacle free
        obs_check = tree.obs_check(near_v, x_new, label, 'reg')

        # iterate over each buchi state
        for b_state in buchi_graph.nodes:

            # new product state
            q_new = (x_new, b_state)

            # extend
            added = tree.extend(q_new, near_v, label, obs_check)
            # rewire
            # if added == 1:
            #     print(q_new)
                # tree.rewire(q_new, near_v, obs_check)

        # number of nodes
        sz.append(tree.tree.number_of_nodes())
        # first accepting state
        if len(tree.goals) >= 5:
            break

    cost_path = tree.findpath(tree.goals)
    return cost_path, sz
