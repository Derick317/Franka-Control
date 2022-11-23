from matplotlib import pyplot as plt
import math
import time

 
fig=plt.figure()
ax=fig.add_subplot(1,1,1)

ax.set_xlabel('Time')
ax.set_ylabel('cos(t)')
ax.set_title('')

line = None
plt.grid(True) #
plt.ion()  #interactive mode on
obsX = []
obsY = []

t0 = time.time()

while True:
    t = time.time()-t0
    obsX.append(t)
    obsY.append(math.cos(2*math.pi*1*t))

    if line is None:
        line = ax.plot(obsX,obsY,'-g',marker='*')[0]

    line.set_xdata(obsX)
    line.set_ydata(obsY)

    ax.set_xlim([t-10,t+1])
    ax.set_ylim([-1,1])

    plt.pause(0.01)

