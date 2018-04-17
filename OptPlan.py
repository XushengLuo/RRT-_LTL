from Buchi import buchi_graph
from Problem import problemFormulation
import datetime
from tree import tree, construction_tree
from WorkspacePlot import region_plot, path_plot
from collections import OrderedDict
import matplotlib.pyplot as plt

start = datetime.datetime.now()
# +------------------------------------------+
# |     construct transition system graph    |
# +------------------------------------------+

workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
fig, ax = plt.subplots()
region_plot(regions, 'region', fig, ax)
region_plot(obs, 'obs', fig, ax)
ts = {'workspace':workspace, 'region':regions, 'obs':obs, 'uni_cost':uni_cost}

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

n_max = 100
step_size = 0.5
cost_path = OrderedDict()
for b_init in buchi_graph.graph['init']:
    # prefix path
    init = (init_state, b_init)
    acpt = buchi_graph.graph['accept']
    tree_pre = tree(acpt, ts, buchi_graph, init, 'pre', step_size)
    print('--------------prefix path---------------------')
    # prefix path with cost
    cost_path_pre = construction_tree(tree_pre, buchi_graph, n_max)

    print('{0} accepting goals found'.format(len(tree_pre.goals)))
    if not len(tree_pre.goals):
        print('Couldn\'t find the path within predetermined iteration')
        break

    #suffix path
    cost_path_suf = OrderedDict()
    # each initial state <=> multiple accepting states
    for i in range(len(tree_pre.goals)):
        print('--------------suffix path for {0}-th goal (of {1} in total)---------------------'.format(i, len(tree_pre.goals)))
        goal = tree_pre.goals[i]
        tree_suf = tree('', ts, buchi_graph, goal, 'suf', step_size)
        cost_path_suf_cand = construction_tree(tree_suf, buchi_graph, n_max)

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

    # pre + suf cost

    for i in cost_path_pre.keys():
        cost_path[i] = (cost_path_pre[i][0]+cost_path_suf[i][0], (cost_path_pre[i][1], cost_path_suf[i][1]))

    cost_path= OrderedDict(sorted(cost_path.items(), key=lambda x: x[1][0]))
    mincost = list(cost_path.keys())[0]
    opt_path = cost_path[mincost]
    print('Total cost = prefix Cost + suffix Cost: {0} = {1} + {2}'.format(opt_path[0], cost_path_pre[mincost][0], cost_path_suf[mincost][0]))
    print('Time cost: {0}'.format((datetime.datetime.now() - start).total_seconds()))
    path_plot(opt_path[1])




