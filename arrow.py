import numpy as np
import matplotlib.pyplot as plt
from WorkspacePlot import region_plot
from z_Problem import problemFormulation
# x = np.linspace(0, 2*np.pi, 10)
# y = np.sin(x)
workspace, regions, obs, init_state, uni_cost, formula = problemFormulation().Formulation()
region_plot(regions, 'region')
x = np.asarray([0.8       , 0.67571907, 0.36951452, 0.35260914, 0.17873134,
       0.21171805, 0.12471302, 0.22513551])
y = np.asarray([0.1       , 0.24808197, 0.4946492 , 0.66462852, 0.7127742 ,
       0.54580806, 0.19047892, 0.21349494])

suf = plt.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], color = 'r', scale_units='xy', angles='xy', scale=1, label = 'suf')
plt.legend(handles = [suf])
plt.show()