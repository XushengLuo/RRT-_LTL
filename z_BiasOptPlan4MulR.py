import Buchi
from Problem import problemFormulation
import datetime
from z_Biastree4MulR import tree, construction_tree
from collections import OrderedDict
import numpy as np
from WorkspacePlot import region_plot, path_plot, layer_plot
import matplotlib.pyplot as plt
import networkx as nx
import pickle
import sys

# +------------------------------------------+
# |     construct transition system graph    |
# +------------------------------------------+

workspace, regions, obs, init_state, uni_cost, formula, formula_comp, exclusion, no = problemFormulation().Formulation()
ts = {'workspace':workspace, 'region':regions, 'obs':obs, 'uni_cost':uni_cost}
# plot the workspace
# ax = plt.figure(1).gca()
# region_plot(regions, 'region', ax)
# region_plot(obs, 'obs', ax)
# +------------------------------------------+
# |            construct buchi graph         |
# +------------------------------------------+
# start1 = datetime.datetime.now()
# #
# buchi = Buchi.buchi_graph(formula, formula_comp, exclusion)
# buchi.formulaParser()
# buchi.execLtl2ba()
# _ = buchi.buchiGraph()
# buchi.DelInfesEdge(len(init_state))
# min_qb = buchi.MinLen()
# buchi.FeasAcpt(min_qb)
# buchi_graph = buchi.buchi_graph
# buchi_state = dict(zip(list(buchi_graph.nodes()), range(1, buchi_graph.number_of_nodes() + 1)))  # dict
# # # # # # #
# print((datetime.datetime.now() - start1).total_seconds())
# # #
# with open('data/buchi_case2_false', 'wb') as filehandle:
#     # store the data as binary data stream
#     pickle.dump(buchi_graph, filehandle)
#     pickle.dump(buchi_state, filehandle)
#     pickle.dump(min_qb, filehandle)
with open('data/buchi_case2_false', 'rb') as filehandle:
    # store the data as binary data stream
    buchi_graph = pickle.load(filehandle)
    buchi_state = pickle.load(filehandle)
    min_qb = pickle.load(filehandle)
# print(buchi_graph.number_of_nodes())
# print(buchi_graph.number_of_edges())
# +------------------------------------------+
# |            construct prefix path         |
# +------------------------------------------+
# print(1)
n_max = 1000
n_robot = len(init_state)
step_size = 0.25*n_robot
cost_path = OrderedDict()


for b_init in buchi_graph.graph['init']:
    # initialization
    opt_cost = (np.inf, np.inf)
    opt_path_pre = []
    opt_path_suf = []
    opt_tree_suf = nx.DiGraph()

    """
     #----------------------------------------------#
     |                Prefix Path                   |
     #----------------------------------------------#
    """
    start = datetime.datetime.now()
    init = (init_state, b_init)
    acpt = buchi_graph.graph['accept']
    tree_pre = tree(n_robot, acpt, ts, buchi_graph, init, 'pre', step_size, no)
    # print('--------------prefix path---------------------')
    # prefix path with cost
    cost_path_pre, sz = construction_tree(tree_pre, buchi_graph, min_qb, regions, n_max)
    # print(cost_path_pre[0][1])
    if len(tree_pre.goals):
        pre_time = (datetime.datetime.now() - start).total_seconds()
        # print('Time for prefix path: {0}'.format(pre_time))
        # print('{0} accepting goals found'.format(len(tree_pre.goals)))

        # write into file
        nx.write_gpickle(tree_pre, "data_pre_tree")

        ## plot the distribution of accepting states
        # x = np.asarray([point[0][0] for point in tree_pre.goals])
        # y = np.asarray([point[0][1] for point in tree_pre.goals])
        # plt.plot(x, y, 'g*')
        # workspace_plot(regions, obs)
    else:
        print('Couldn\'t find the path within predetermined iteration')
        break

    """
     #----------------------------------------------#
     |                Suffix Path                   |
     #----------------------------------------------#
    """
    start = datetime.datetime.now()
    tree_suf = tree_pre
    # each initial state <=> multiple accepting states
    # for i in range(len(tree_pre.goals)):
    for i in range(1):
        # goal product state
        goal = tree_pre.goals[i]
        tree_suf = tree(n_robot,'', ts, buchi_graph, goal, 'suf', step_size, no)
        # update accepting buchi state
        buchi_graph.graph['accept'] = goal[1]
        # construct suffix tree
        cost_path_suf_cand, _ = construction_tree(tree_suf, buchi_graph, min_qb, regions, n_max)

        # print('--------------suffix path for {0}-th goal (of {1} in total)---------------------'.format(i, len(tree_pre.goals)))
        # print('{0}-th goal: {1} accepting goals found'.format(i, len(tree_suf.goals)))
        # couldn't find the path
        try:
            # order according to cost
            cost_path_suf_cand = OrderedDict(sorted(cost_path_suf_cand.items(), key=lambda x: x[1][0]))
            mincost = list(cost_path_suf_cand.keys())[0]
        except IndexError:
            del cost_path_pre[i]
            # print('delete {0}-th item in cost_path_pre, {1} left'.format(i, len(cost_path_pre)))
            continue
        cost_path_suf = cost_path_suf_cand[mincost]

        if cost_path_pre[i][0] + cost_path_suf[0] < opt_cost[0] + opt_cost[1]:
            opt_path_pre = cost_path_pre[i][1]      # plan of [(position, buchi)]
            opt_path_suf = cost_path_suf[1]
            opt_cost = (cost_path_pre[i][0], cost_path_suf[0])    # optimal cost (pre_cost, suf_cost)
            opt_tree_suf = tree_suf

        nx.write_gpickle(opt_tree_suf, "data_suf_tree")

    # first pre + suf path
    # first_path = (cost_path_pre[0][0] + cost_path_suf[0][0], (cost_path_pre[0][1], cost_path_suf[0][1]))
    # path_plot(first_path[1])

    """
     #----------------------------------------------#
     |                  Pre + Suf                   |
     #----------------------------------------------#
    """

    # print('Total cost = prefix Cost + suffix Cost: {0} = {1} + {2}'.format(opt_cost[0]+opt_cost[1], opt_cost[0], opt_cost[1]))
    suf_time = (datetime.datetime.now() - start).total_seconds()
    # print('Time to find the surfix path: {0}'.format(suf_time))
    print(pre_time, suf_time, opt_cost[0], opt_cost[1], pre_time + suf_time, (opt_cost[0] + opt_cost[1])/2)
    # plot optimal path

    # path_plot((opt_path_pre, opt_path_suf), regions, obs, tree_pre.robot, tree_pre.dim)
    # draw 3D layer graph
    # layer_plot(tree_pre.tree, opt_path_pre, buchi_state)
    # layer_plot(tree_suf.tree, opt_path_suf, buchi_state)


    # write into file
    # with open('data/data_opt_path_b_case3_010_{0}'.format(float(sys.argv[1])), 'wb') as filehandle:
    #     # store the data as binary data stream
    #     pickle.dump((opt_path_pre, opt_path_suf), filehandle)
    #     pickle.dump(tree_pre, filehandle)
    #     pickle.dump(tree_suf, filehandle)
    #     pickle.dump(buchi_state, filehandle)
    #     pickle.dump(sz, filehandle)

    # plot size of tree versus iteration
    # plt.figure(3)
    # plt.gca().set_aspect('auto', adjustable='box')
    # plt.rc('text', usetex=True)
    # plt.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
    # plt.plot(np.asarray(range(0,len(sz))), np.asarray(sz))
    # plt.xlabel(r'Iteration $n$')
    # plt.ylabel(r"$|V_T^n|$")
    # plt.savefig('size_VS_n.png', bbox_inches='tight', dpi=600)
    # plt.show()