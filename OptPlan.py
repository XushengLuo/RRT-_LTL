from Buchi import buchi_graph
from z_Problem import problemFormulation
import datetime
from tree import tree, construction_tree
from WorkspacePlot import region_plot, path_plot
from collections import OrderedDict
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

# +------------------------------------------+
# |     construct transition system graph    |
# +------------------------------------------+

workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
ts = {'workspace':workspace, 'region':regions, 'obs':obs, 'uni_cost':uni_cost}
# plot the workspace
ax = plt.figure(1).gca()
region_plot(regions, 'region', ax)
region_plot(obs, 'obs', ax)
# +------------------------------------------+
# |            construct buchi graph         |
# +------------------------------------------+

buchi = buchi_graph(formula)
buchi.formulaParser()
buchi.execLtl2ba()
buchi_graph = buchi.buchiGraph()

# +------------------------------------------+
# |            construct prefix path         |
# +------------------------------------------+

n_max = 200
step_size = 0.25
cost_path = OrderedDict()


for b_init in buchi_graph.graph['init']:
    # prefix path
    start = datetime.datetime.now()
    init = (init_state, b_init)
    acpt = buchi_graph.graph['accept']
    tree_pre = tree(acpt, ts, buchi_graph, init, 'pre', step_size)
    print('--------------prefix path---------------------')
    # prefix path with cost
    cost_path_pre, sz = construction_tree(tree_pre, buchi_graph, n_max)

    if len(tree_pre.goals):
        print('Time for prefix path: {0}'.format((datetime.datetime.now() - start).total_seconds()))
        # print(tree_pre.goals)
        print('{0} accepting goals found'.format(len(tree_pre.goals)))

        nx.write_gpickle(tree_pre, "data_pre_tree")
        ## plot the distribution of accepting states
        # x = np.asarray([point[0][0] for point in tree_pre.goals])
        # y = np.asarray([point[0][1] for point in tree_pre.goals])
        # plt.plot(x, y, 'g*')
        # workspace_plot(regions, obs)
    else:
        print('Couldn\'t find the path within predetermined iteration')
        break

    #suffix path
    start = datetime.datetime.now()
    cost_path_suf = OrderedDict()
    # each initial state <=> multiple accepting states
    for i in range(len(tree_pre.goals)):
        goal = tree_pre.goals[i]
        tree_suf = tree('', ts, buchi_graph, goal, 'suf', step_size)
        cost_path_suf_cand, _ = construction_tree(tree_suf, buchi_graph, n_max)

        print('--------------suffix path for {0}-th goal (of {1} in total)---------------------'.format(i, len(tree_pre.goals)))
        print('{0}-th goal: {1} accepting goals found'.format(i, len(tree_suf.goals)))
        # couldn't find the path
        try:
            # order according to cost
            cost_path_suf_cand = OrderedDict(sorted(cost_path_suf_cand.items(), key=lambda x: x[1][0]))
            mincost = list(cost_path_suf_cand.keys())[0]
        except IndexError:
            del cost_path_pre[i]
            print('delete {0}-th item in cost_path_pre, {1} left'.format(i, len(cost_path_pre)))
            continue
        cost_path_suf[i] = cost_path_suf_cand[mincost]

    # first pre + suf path
    # first_path = (cost_path_pre[0][0] + cost_path_suf[0][0], (cost_path_pre[0][1], cost_path_suf[0][1]))
    # path_plot(first_path[1])

    # pre + suf cost

    for i in cost_path_pre.keys():
        cost_path[i] = (cost_path_pre[i][0]+cost_path_suf[i][0], (cost_path_pre[i][1], cost_path_suf[i][1]))

    cost_path= OrderedDict(sorted(cost_path.items(), key=lambda x: x[1][0]))
    mincost = list(cost_path.keys())[0]
    opt_path = cost_path[mincost]
    print('Total cost = prefix Cost + suffix Cost: {0} = {1} + {2}'.format(opt_path[0], cost_path_pre[mincost][0], cost_path_suf[mincost][0]))
    print('Time to find the surfix path: {0}'.format((datetime.datetime.now() - start).total_seconds()))

    # plot optimal path
    buchi_state = dict(zip(list(buchi_graph.nodes()), range(1, buchi_graph.number_of_nodes()+1)))
    path_plot(opt_path[1])


    # plot size of tree versus iteration
    plt.figure(3)
    plt.gca().set_aspect('auto', adjustable='box')
    plt.rc('text', usetex=True)
    plt.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
    plt.plot(np.asarray(range(0,len(sz))), np.asarray(sz))
    plt.xlabel(r'Iteration $n$')
    plt.ylabel(r"$|V_T^n|$")
    plt.show()
    plt.savefig('size_VS_n.png', bbox_inches='tight', dpi=600)




