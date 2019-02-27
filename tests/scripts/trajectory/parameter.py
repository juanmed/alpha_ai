import numpy as np


order = 6               # order
n = 4                   # flatness output x y z psi
gate = 11               # gate
m = gate + 1            # keyframe (with returning)  final point = initial point
mass = 1

# set time interval depending on distance between gate
t = 1 * np.array([0, 1, 2, 3, 3.5, 4.5, 5, 5.5, 6, 6.5, 7.5, 8.5])

# time_interval = 1
# t = np.linspace(0, time_interval*m, m+1)
