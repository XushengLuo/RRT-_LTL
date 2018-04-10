import matplotlib.pyplot as plt
from Problem import problemFormulation
def region_plot(regions, flag):

    fig, ax = plt.subplots()
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    ax.set_xlim((0, 1))
    ax.set_ylim((0, 1))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(b=True, which='major', color='k', linestyle='--')
    for key in regions.keys():
        if key[1] == 'b':
            circle = plt.Circle(regions[key][0:-1], regions[key][-1], color='b', fill=(flag!='region'))
            ax.add_artist(circle)
            ax.text(regions[key][0], regions[key][1], r'$l_{}$'.format(key[0][1:]), fontsize=16)

def path_plot(path):
    x = [point[0][0] for point in path[0]]
    y = [point[0][1] for point in path[0]]
    plt.plot(x, y, 'r-')

    x = [point[0][0] for point in path[1]]
    y = [point[0][1] for point in path[1]]
    plt.plot(x, y, 'b-')
    plt.savefig('formula.png', bbox_inches='tight', dpi=600)
    plt.show()


workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
region_plot(regions, 'region')
path = (2.50032615691877, ([((0.8, 0.1), 'T0_init'), ((0.5981724872087247, 0.3377620547972048), 'T0_init'), ((0.49389453454964805, 0.4276146121762987), 'T0_init'), ((0.44573857157429486, 0.5078975412511033), 'T1_S8'), ((0.2408944180045951, 0.7160394591937538), 'T1_S8'), ((0.16881448811963862, 0.45388479130328774), 'T2_S8'), ((0.07373085190437723, 0.20101216973373925), 'T2_S8'), ((0.08908170012726457, 0.39929418348934276), 'accept_S8')], [((0.08908170012726457, 0.39929418348934276), 'accept_S8'), ((0.16156655801415953, 0.7450495775375108), 'T1_S8'), ((0.10289083291471668, 0.4017562120131536), 'T2_S8'), ((0.08025318053203045, 0.2943047234035542), 'T2_S8')]))
path_plot(path[1])