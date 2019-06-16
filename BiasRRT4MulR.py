"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 8/30/18
"""
"""
construct trees for biased sampling optimal task planning for multi-robots
"""

from random import uniform, choice
from networkx.classes.digraph import DiGraph
from networkx.algorithms import dfs_labeled_edges
import math
import numpy as np
from scipy.stats import truncnorm
from collections import OrderedDict
import pyvisgraph as vg
from shapely.geometry import Point, Polygon, LineString
from uniform_geometry import sample_uniform_geometry


class second_tree(object):
    """ construction of prefix and suffix tree
    """
    def __init__(self, n_robot, acpt, ts, buchi_graph, init, step_size, no):
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
        self.step_size = step_size
        self.dim = len(self.ts['workspace'])
        uni_v = np.power(np.pi, self.robot*self.dim/2) / math.gamma(self.robot*self.dim/2+1)
        self.gamma = np.ceil(4 * np.power(1/uni_v, 1./(self.dim*self.robot)))   # unit workspace
        self.tree = DiGraph(type='PBA', init=init)
        self.group = dict()
        # label of init
        label = []
        for i in range(self.robot):
            l = self.label(self.init[0][i])
            # exists one sampled point lies within obstacles
            if l != '':
                l = l + '_' + str(i+1)
            label.append(l)

        self.tree.add_node(self.init, cost=0, label=label)
        # label of acpt
        label = []
        for i in range(self.robot):
            l = self.label(self.init[0][i])
            # exists one sampled point lies within obstacles
            if l != '':
                l = l + '_' + str(i + 1)
            label.append(l)
        self.tree.add_node(self.acpt, cost=0, label=label)
        # index of robot whose location needs to be changed
        self.change = self.match(label)
        # best node
        self.x_min = self.init
        self.c = np.linalg.norm(np.subtract(self.init[0], self.acpt[0]))
        # probability for selecting a node
        self.p = 0.9
        # target point
        self.weight = 0.5
        # threshold for collision avoidance
        self.threshold = 0.005
        # polygon obstacle
        polys = [[vg.Point(0.4, 1.0), vg.Point(0.4, 0.7), vg.Point(0.6, 0.7), vg.Point(0.6, 1.0)],
                 [vg.Point(0.3, 0.2), vg.Point(0.3, 0.0), vg.Point(0.7, 0.0), vg.Point(0.7, 0.2)]]
        self.g = vg.VisGraph()
        self.g.build(polys, status=False)
        # region that has ! preceding it
        self.no = no

        # biased sampling

    def match(self, label):
        """
        match between self.init and self.acpt
        :param label: label of acpt
        :return:
        """
        x = self.init
        x_new = self.acpt
        change = []

        for r in range(self.robot):
            flag = True
            # the line connecting two points crosses an obstacle
            for (obs, boundary) in iter(self.ts['obs'].items()):
                if LineString([Point(x[0][r]), Point(x_new[0][r])]).intersects(boundary):
                    change.append(r)
                    flag = False
                    break

            if not flag:
                continue

            for (region, boundary) in iter(self.ts['region'].items()):
                if LineString([Point(x[0][r]), Point(x_new[0][r])]).intersects(boundary) \
                        and region + '_' + str(r + 1) != label[r] \
                        and region + '_' + str(r + 1) != self.tree.nodes[x]['label'][r]:

                        change.append(r)
        return change

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

    def target(self, init, tg):
        """
        find the closest vertex in the short path from init to target
        :param init: inital point
        :param target: target labeled region
        :param regions: regions
        :return: closest vertex
        """
        shortest = self.g.shortest_path(vg.Point(init[0], init[1]), vg.Point(tg[0], tg[1]))
        return shortest[1].x, shortest[1].y

    def gaussian_guided(self, x, target):
        """
        calculate new point following gaussian dist guided by the target
        :param x: mean point
        :param target: target point
        :return: new point
        """
        # d = np.linalg.norm(np.subtract(x, target))
        # angle = np.arctan2(target[1] - x[1], target[0] - x[0])
        d = self.get_truncated_normal(0, 1/3, 0, np.inf)
        d = d.rvs()
        angle = np.random.normal(0, np.pi/12/3/3, 1) + np.arctan2(target[1] - x[1], target[0] - x[0])
        x_rand = np.add(x, np.append(d*np.cos(angle), d*np.sin(angle)))
        x_rand = [self.trunc(x) for x in x_rand]
        return tuple(x_rand)

    def guided_sample_by_destination(self, x_rand):
        """
        sample guided by truth value
        :param truth: the value making transition occur
        :param x_rand: random selected node
        :param x_label: label of x_rand
        :param regions: regions
        :return: new sampled point
        """
        x = list(x_rand)
        for ind in self.change:
            # same component with self.acpt
            if x[ind] == self.acpt[0][ind]:
                continue
            orig_x_rand = x_rand[ind]  # save
            while 1:
                x[ind] = orig_x_rand  # recover
                if np.random.uniform(0, 1, 1) <= self.weight:
                    tg = self.target(orig_x_rand, self.acpt[0][ind])
                    x[ind] = self.gaussian_guided(orig_x_rand, tg)
                else:
                    xi_rand = []
                    for i in range(self.dim):
                        xi_rand.append(uniform(0, self.ts['workspace'][i]))
                    x[ind] = tuple(xi_rand)

                if self.collision_avoidance(x, ind):
                    break
            #   x_rand
        return self.mulp2sglp(x)

    def sample(self):
        """
        sample point from the workspace
        :return: sampled point, tuple
        """
        while True:
            # random select node as q_rand
            if np.random.uniform(0, 1, 1) <= self.p:
                q_rand = self.x_min
            else:
                q_rand = choice(list(self.tree.nodes))
            # since self.acpt is in the tree, maybe it's rootless
            if q_rand == self.acpt:
                continue
            else:
                break
        return self.guided_sample_by_destination(q_rand[0]), q_rand

    def steer(self, x_rand, x_nearest):
        """
        steer
        :param: x_rand randomly sampled point form: single point ()
        :param: x_nearest nearest point in the tree form: single point ()
        :return: new point single point ()
        """
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
            self.tree.add_node(q_new, cost=cost, label=label)
            self.tree.add_edge(q_min, q_new)
            # update the distance from acpt
            if np.linalg.norm(np.subtract(q_new[0], self.acpt[0])) < self.c:
                self.x_min = q_new
                self.c = np.linalg.norm(np.subtract(q_new[0], self.acpt[0]))
            if self.obs_check([self.acpt], q_new[0], label, 'final')[(q_new[0], self.acpt[0])]:
                self.goals.append(q_new)
        return added

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
            # if len(p_near) > 0:
            #     break
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
            mp.append(point[i*self.dim:(i+1)*self.dim])
        return tuple(mp)


def second_construction_tree(tree, buchi_graph):

    # for n in range(n_max):
    while 1:
        # sample form: multiple
        x_new, q_rand = tree.sample()
        x_new = tree.sglp2mulp(x_new)
        label = []
        o_id = True
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
        # # near state
        # near_v = tree.near(tree.mulp2sglp(x_new))
        # # add q_rand
        # if q_rand not in near_v:
        #     near_v = near_v + [q_rand]
        near_v = [q_rand]
        # print(tree.x_min)
        # check obstacle free
        obs_check = tree.obs_check(near_v, x_new, label, 'reg')

        if not list(obs_check.items())[0][1]:
            continue
        # iterate over each buchi state
        for b_state in buchi_graph.nodes:

            # new product state
            q_new = (x_new, b_state)

            # extend
            tree.extend(q_new, near_v, label, obs_check)

        # first accepting state
        if len(tree.goals):
            break

    cost_path = tree.findpath(tree.goals)
    return cost_path
