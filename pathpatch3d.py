"""
============================
Draw flat objects in 3D plot
============================

Demonstrate using pathpatch_2d_to_3d to 'draw' shapes and text on a 3D plot.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, PathPatch
# register Axes3D class with matplotlib by importing Axes3D
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.text import TextPath
from matplotlib.transforms import Affine2D
from Problem import problemFormulation
from matplotlib.patches import Polygon
import pickle
import networkx as nx

#----------------------------------------------------------------------------------------------------
# layer plot
# def intersection(l1, l2):
#     """
#     calculate the point of intersection between adjacent edges
#     :param l1: line 1
#     :param l2: line 2
#     :return: point of intersection
#     """
#     if not l1[0]:
#         y = -l1[2]/l1[1]
#         x = (-l2[2] - l2[1]*y)/l2[0]
#     elif not l2[0]:
#         y = -l2[2] / l2[1]
#         x = (-l1[2] - l1[1] * y) / l1[0]
#     else:
#         y = -(l1[2]*l2[0] - l2[2]*l1[0])/(l1[1]*l2[0]-l2[1]*l1[0])
#         x = (-l1[2]*l2[0]-l1[1]*l2[0]*y)/(l1[0]*l2[0])
#     return (x,y)
#
# def text3d(ax, xyz, s, zdir="z", size=None, angle=0, usetex=False, **kwargs):
#     '''
#     Plots the string 's' on the axes 'ax', with position 'xyz', size 'size',
#     and rotation angle 'angle'.  'zdir' gives the axis which is to be treated
#     as the third dimension.  usetex is a boolean indicating whether the string
#     should be interpreted as latex or not.  Any additional keyword arguments
#     are passed on to transform_path.
#
#     Note: zdir affects the interpretation of xyz.
#     '''
#     x, y, z = xyz
#     if zdir == "y":
#         xy1, z1 = (x, z), y
#     elif zdir == "y":
#         xy1, z1 = (y, z), x
#     else:
#         xy1, z1 = (x, y), z
#
#     text_path = TextPath((0, 0), s, size=size, usetex=usetex)
#     trans = Affine2D().rotate(angle).translate(xy1[0], xy1[1])
#
#     p1 = PathPatch(trans.transform_path(text_path), **kwargs)
#     ax.add_patch(p1)
#     art3d.pathpatch_2d_to_3d(p1, z=z1, zdir=zdir)
#
# def workspaceplot(regions, flag, ax):
#
#     plt.rc('text', usetex=True)
#     plt.rc('font', family='serif')
#     # Draw a circle on the x=0 'wall'
#     for key in regions.keys():
#         if key[1] == 'b':
#             p = Circle(regions[key][0:-1], regions[key][-1]*5, color='m', fill=(flag != 'region'))
#             ax.add_patch(p)
#             art3d.pathpatch_2d_to_3d(p, z=-2, zdir="z")
#             text3d(ax, (regions[key][0], regions[key][1], -2),
#                    r'${}_{}$'.format(key[0][0], key[0][1:]),
#                zdir="z", size=0.1, usetex=True,
#                ec="none", fc="k")
#
#         if key[1] == 'p':
#             x = []
#             y = []
#             for i in range(len(regions[key])-1):
#                 x0, y0 = intersection(regions[key][i], regions[key][i+1])
#                 x.append(x0)
#                 y.append(y0)
#             x0, y0 = intersection(regions[key][0], regions[key][-1])
#             x.append(x0)
#             y.append(y0)
#             polygon = Polygon(np.column_stack((x,y)), True, fill=False)
#             ax.add_patch(polygon)
#             # patches.append(polygon)
#             # p = PatchCollection(patches)
#             # ax.add_collection(p)
#             art3d.pathpatch_2d_to_3d(polygon, z=-2, zdir="z")
#             text3d(ax, (np.mean(x), np.mean(y),-2),
#                    r'${}_{}$'.format(key[0][0], key[0][1:]),
#                    zdir="z", size=0.1, usetex=True,
#                    ec="none", fc="k")
#
#     # Write a Latex formula on the z=0 'floor'
#     ax.set_xlabel('x', usetex=True)
#     ax.set_ylabel('y', usetex=True)
#     ax.set_zlabel('z', usetex=True)
#     ax.set_xlim(0, 1)
#     ax.set_ylim(0, 1)
#     ax.set_zlim(-2, 11)
#     ax.set_zticks(range(1,12))
#     ax.set_zticklabels(range(1, 12), usetex=True)
#     ax.set_xticklabels([0.0, 0.2, 0.4, 0.6, 0.8, 1.0], usetex=True)
#     ax.set_yticklabels([0.0, 0.2, 0.4, 0.6, 0.8, 1.0], usetex=True)
#
# def layer_plot(tree, opt_path, buchi, ax, seg):
#     """
#     plot 3D layer graph
#     :param tree: tree built by alg
#     :param buchi: buchi state
#     :return: none
#     """
#     ax.grid(False)
#     ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#     ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#     ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#     path = list(nx.dfs_edges(tree))
#
#     # 2D workspace in 3D
#     plt.rc('text', usetex=True)
#     plt.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
#     for i in range(len(path)):
#         x_pre = np.array([path[i][0][0][0][0], path[i][1][0][0][0]])
#         y_pre = np.array([path[i][0][0][0][1], path[i][1][0][0][1]])
#         z_pre = np.asarray([buchi[path[i][0][1]], buchi[path[i][1][1]]])
#         ax.plot(x_pre, y_pre, z_pre, 'b--d', markerfacecolor='None')
#
#     x_pre = np.asarray([point[0][0][0] for point in opt_path])
#     y_pre = np.asarray([point[0][0][1] for point in opt_path])
#     z_pre = np.asarray([buchi[point[1]] for point in opt_path])
#     ax.plot(x_pre, y_pre, z_pre, 'r--d')
#     ax.plot([x_pre[0]], [y_pre[0]], [z_pre[0]], 'ks',markersize=10)
#     # region_plot(regions, 'region', ax)
#     # region_plot(obs, 'obs', ax)
#
#     plt.savefig('/Users/chrislaw/Box Sync/RRL_LTL_cntsSpace/figures/{0}_3d.png'.format(seg), bbox_inches='tight', dpi=600)
#
#
# workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
# fig = plt.figure(1)
# ax = fig.add_subplot(111, projection='3d')

# workspaceplot(regions, 'region', ax)
# workspaceplot(obs, 'obstacle', ax)

# read from file
# with open('data_opt_path_1st', 'rb') as filehandle:
#     # store the data as binary data stream
#     (opt_path_pre, opt_path_suf) = pickle.load(filehandle)
#     tree_pre = pickle.load(filehandle)
#     tree_suf = pickle.load(filehandle)
#     buchi_state = pickle.load(filehandle)
#     sz = pickle.load(filehandle)
#
# layer_plot(tree_pre.tree, opt_path_pre, buchi_state, ax, 'pre')
#
# fig = plt.figure(2)
# ax2 = fig.add_subplot(111, projection='3d')
#
# workspaceplot(regions, 'region', ax2)
# workspaceplot(obs, 'obstacle', ax2)
# layer_plot(tree_suf.tree, opt_path_suf, buchi_state, ax2, 'suf')
#
#
# plt.show()

#-------------------------------------------------------------------------------------------------------------
# delta vs time
# xs = [0.15, 0.12, 0.1, 0.07, 0.04, 0.02]
# y1 = [2.576, 5.0, 6.06, 21.34, 294.9866, 1786.18, ]
# y2 = [18.0867, 36.3658, 93.8261, 747.1555, 5185.8660, 31320.182514]
#
#
# plt.rc('text', usetex=True)
# plt.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
# fig = plt.figure()
# # f, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
# ax = fig.add_subplot(111)    # The big subplot
# x = range(len(xs))
#
# # ax.plot(x, y1, 'r--d')
# # ax.plot(x, y2, 'b--d')
# # ax.set_ylabel(r'Time(s)')
# # ax.set_xlabel(r'$\delta$')
# # ax.set_xticks(x)
# # ax.set_xticklabels(xs)
# # for i in range(3,len(x)):
# #     ymin, ymax = plt.ylim()
# #     xmin, xmax = plt.xlim()
# #     plt.axvline(x=x[i], ls = '--', lw = 0.5, color = 'k', ymin= 0, ymax=(y1[i]-ymin)/(ymax-ymin))
# #     plt.axhline(y=y1[i], ls='--', lw = 0.5, color='k', xmin=0, xmax=(x[i] - xmin) / (xmax - xmin))
# #     plt.axvline(x=x[i], ls='--', lw=0.5, color='k', ymin=0, ymax=(y2[i] - ymin) / (ymax - ymin))
# #     plt.axhline(y=y2[i], ls='--', lw=0.5, color='k', xmin=0, xmax=(x[i] - xmin) / (xmax - xmin))
# #
# # plt.savefig(r'delta_vs_time.png', bbox_inches='tight', dpi=600)
# # plt.show()
#
# ax1 = fig.add_subplot(211)
# ax2 = fig.add_subplot(212)
# # ax, ax2 = f.add_subplots(2, 1, sharex=True)
# # Turn off axis lines and ticks of the big subplot
# ax.spines['top'].set_color('none')
# ax.spines['bottom'].set_color('none')
# ax.spines['left'].set_color('none')
# ax.spines['right'].set_color('none')
# ax.tick_params(labelcolor='w', top='off', bottom='off', left='off', right='off')
#
# # hide the spines between ax and ax2
# ax1.spines['bottom'].set_visible(False)
# ax2.spines['top'].set_visible(False)
# ax1.spines['top'].set_color('none')
# ax1.spines['right'].set_color('none')
# ax2.spines['right'].set_color('none')
# # ax1.get_xaxis().set_visible(False)
# ax1.tick_params(labeltop='off')  # don't put tick labels at the top
# ax2.xaxis.tick_bottom()
# ax1.xaxis.tick_top()
# empty_string_labels = ['']*len(x)
# ax1.set_xticklabels(empty_string_labels)
# s = 5
# ax1.plot(x, y1, 'r--d', markersize = s, label='Algorithm 1')
# ax1.plot(x, y2, 'b--s', markersize = s, label='Sparse RRG')
# ax1.legend()
# ax2.plot(x, y1, 'r--d', markersize = s)
# ax2.plot(x, y2, 'b--s', markersize = s)
#
# # zoom-in / limit the view to different portions of the data
# ax1.set_ylim(4000, 35000)  # outliers only
# ax2.set_ylim(-100, 2000)  # most of the data
# ax1.set_yticks([4000,  10000, 16000, 22000,  28000, 34000])
# ax2.set_yticks([0,  500, 1000, 1500,  2000])
# ax.set_ylabel(r'Time(s)',)
# ax.yaxis.set_label_coords(-0.1,0.5)
# ax.set_xlabel(r'$\delta$')
# # ax2.set_xticklabels(x)
# # plt.xticks(x, xs)
# ax2.set_xticks(x)
# ax2.set_xticklabels(xs)
# ax1.grid(linestyle='--')
# ax2.grid(linestyle='--')
# # plt.xticks([0.15, 0.12, 0.10, 0.07, 0.04], ["0.15", "0.12", "0.10", "0.07", "0.04"])
# ymin, ymax = plt.ylim()
# xmin, xmax = plt.xlim()
# # for i in range(4,len(x)):
# #     ax1.axvline(x=x[i], ls = '--', lw = 0.5, color = 'k', ymin= 0, ymax=(y1[i]-ymin)/(ymax-ymin))
# #     plt.axhline(y=y1[i], ls='--', lw = 0.5, color='k', xmin=0, xmax=(x[i] - xmin) / (xmax - xmin))
# #     plt.axvline(x=x[i], ls='--', lw=0.5, color='k', ymin=0, ymax=(y2[i] - ymin) / (ymax - ymin))
# #     plt.axhline(y=y2[i], ls='--', lw=0.5, color='k', xmin=0, xmax=(x[i] - xmin) / (xmax - xmin))
#
# # plt.text(x[0]+50, y[0]-0.04, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[0]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[0]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[0]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# # plt.text(x[1]+30, y[1]+0.025, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[1]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[1]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[1]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# # plt.text(x[2], y[2]+0.025, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[2]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[2]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[2]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# # plt.text(x[3]+60, y[3]+0.01, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[3]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[3]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[3]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
# d = .015  # how big to make the diagonal lines in axes coordinates
# # arguments to pass to plot, just so we don't keep repeating them
# kwargs = dict(transform=ax1.transAxes, color='k', clip_on=False)
# ax1.plot((-d, +d), (-d, +d), **kwargs)        # top-left diagonal
# # ax1.plot((1 - d, 1 + d), (-d, +d), **kwargs)  # top-right diagonal
#
#
# kwargs.update(transform=ax2.transAxes)  # switch to the bottom axes
# ax2.plot((-d, +d), (1 - d, 1 + d), **kwargs)  # bottom-left diagonal
# # ax2.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs)  # bottom-right diagonal
# # plt.text(x[4]-120, y[4]+0.04, r'$t_{\mathrm{pre}} = \,$'+ r'{0}s'.format(tpre[4]) + '\n'+ r'$t_{\mathrm{suf}} =\, $'+ r'{0}'.format(tsuf[4]) + '\n' +r'$|\mathcal{P}|=\,$' + r'{0}'.format(p[4]), bbox={'facecolor':'grey', 'alpha':0.2, 'pad':3})
#
# plt.savefig(r'/Users/chrislaw/Box Sync/RRL_LTL_cntsSpace/figures/runTime_r.png', bbox_inches='tight', dpi=600)
# plt.show()

#-----------------------------------------------------------------------------------------------------------------------------
xs = [0.15, 0.12, 0.1, 0.07, 0.04, 0.02]
x = range(len(xs))
y0 = [1.64, 1.86, 2.09, 2.19, 2.00, 2.14]
y1 = [3.01, 3.06, 3.29, 3.11, 3.57, 3.33]
y2 = [6.41, 6.21, 6.36, 6.5, 6.18, 7.72]
# y = np.subtract(y1, y0)
# yy = np.subtract(y2, y0)
f = plt.figure(1)
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

ax = f.add_subplot(111)
ax.plot(x, y0, 'r--d', label=r'$J_b(\tau)$')
ax.plot(x, y1, 'b--d', label = r'$J_u(\tau)$')
ax.plot(x, y2, 'g--s', label = r'$J_s(\tau)$')
ax.set_xticks(x)
ax.set_ylim(1.5, 8)
ax.set_xticklabels(xs)
ax.set_xlabel(r'$\delta$')
ax.set_ylabel(r'$J(\tau)$')
ax.legend()
# ax.tick_params('y', colors='r')

# ax2 = ax.twinx()
# ax2.set_ylim(1.5, 8)
# ax2.plot(x, y0, 'g--s', label = r'$J(\tau)$')
# ax2.set_ylabel(r'$J(\tau)$')

# ax2.tick_params('y', colors='b')
plt.savefig(r'/Users/chrislaw/Box Sync/RRL_LTL_cntsSpace/figures/delta_dis.png', bbox_inches='tight', dpi=600)
plt.show()




