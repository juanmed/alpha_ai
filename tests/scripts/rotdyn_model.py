import control as ctl
#print(ctl)
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(20,10))
fig.suptitle("Angular velocity dynamics modelling")
ax0 = fig.add_subplot(2,1,1)
ax1 = fig.add_subplot(2,1,2)


# define simulation time
t_max = 10
dt = 0.01
t = np.arange(0.0,t_max,dt)


# define step input
refx = np.ones_like(t)

# wx dynamics
ts = 10	# settling time
tr = 0.3 	# rise time
mp = 0.0625	# overshoot

sigma = 4.6/ts
wn = 1.8/tr
etta = 0.75

# define 2nd order system
num = np.array([wn**2])
den = np.array([1.0, 2*etta*wn, wn**2])
wx_tf = ctl.tf(num,den)

print(" Transfer Function \n{}".format(wx_tf))

# evaluate step response
tx, wx, s = ctl.forced_response(wx_tf, T=t, U=refx)

ax0.plot(t, wx,linestyle = '-',color ='r', label = "wx")
ax0.plot(t, refx,linestyle = '--', color = "k", label = 'x ref')
ax0.set_title("Wx dynamics", fontsize='small')
ax0.legend(loc='center right', shadow=True, fontsize='small')
ax0.set_xlabel("time {s}")
ax0.set_ylabel("wx {rad/s}")
ax0.set_xticks(np.arange(0,10,0.5))

plt.show()

