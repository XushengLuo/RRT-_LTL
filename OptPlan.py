from Buchi import buchi_graph
from Problem import problemFormulation
import datetime
from tree import construction_tree
start = datetime.datetime.now()
# +------------------------------------------+
# |     construct transition system graph    |
# +------------------------------------------+

workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
# ts_graph = ts_graph(regions, init_state, uni_cost).tsGraph()
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

n_pre = 4000
step_size = 0.5
for b_init in buchi_graph.graph['init']:
    init = (init_state, b_init)
    acpt = buchi_graph.graph['accept']
    pre_tree, pre_acpt = construction_tree(acpt, ts, buchi_graph, init, [step_size, n_pre])
