from Buchi import buchi_graph
from Problem import problemFormulation
import datetime
from tree import tree, construction_tree
from WorkspacePlot import region_plot, path_plot
from collections import OrderedDict

start = datetime.datetime.now()
# +------------------------------------------+
# |     construct transition system graph    |
# +------------------------------------------+

workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
region_plot(regions, 'region')
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

n_max = 200
step_size = 0.5
cost_path = OrderedDict()
for b_init in buchi_graph.graph['init']:
    # prefix path
    init = (init_state, b_init)
    acpt = buchi_graph.graph['accept']
    tree_pre = tree(acpt, ts, buchi_graph, init, 'pre', step_size)
    cost_path_pre = construction_tree(tree_pre, buchi_graph, n_max)
    pre_k = list(cost_path_pre.keys())

    #suffix path
    cost_path_suf = OrderedDict()
    # each initial state <=> multiple accepting states
    for i in range(len(tree_pre.goals)):
        goal = tree_pre.goals[i]
        tree_suf = tree('', ts, buchi_graph, goal, 'suf', step_size)
        cost_path_suf_cand = construction_tree(tree_suf, buchi_graph, n_max)
        # couldn't find the path
        try:
            mincost = min(cost_path_suf_cand)
        except ValueError:
            del cost_path_pre[pre_k[i]]
            continue
        cost_path_suf[mincost] = cost_path_suf_cand[mincost]

    # pre + suf cost
    pre_k = list(cost_path_pre.keys())
    suf_k = list(cost_path_suf.keys())
    for i in range(len(cost_path_pre)):
        cost_path[pre_k[i]+suf_k[i]] = (cost_path_pre[pre_k[i]], cost_path_suf[suf_k[i]])

    opt_path = (min(cost_path), cost_path[min(cost_path)])
    print(opt_path)
    path_plot(opt_path[1])




