# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import system_params



m,g,m1,m2,l1,l2 = system_params.get_params()



def visualization(x, tend, ts, d):
    y = np.zeros([3,int(np.ceil(tend/ts))+1])
    #y = np.zeros([3,N*Ni+1])
    y[:,:] = x[range(3),:] 
    
    # position of cart in cartesian
    px_0 = y[0] 
    # position of pendulum 1 mass
    px_1 = px_0 + l1*np.sin(y[1])# 'x' position of pendulum1 mass in cartesian
    py_1 = l1*np.cos(y[1]) # 'y'
    # position of pendulum 2 mass
    px_2 = px_1 + l2*np.sin(y[2])
    py_2 = py_1 + l2*np.cos(y[2])
    
    
    fig, ax = plt.subplots()
    ax.set_xlim([np.min(y[0,:])-l1-l2,np.max(y[0,:])+l1+l2])
    ax.set_ylim([-l1-l2-1,l1+l2+1])
    
    
    pend_plt, = ax.plot([], [], 'o-',lw=3)
    trace_plt, = ax.plot([],[],'-',lw=1)
    
    trace_x = []
    trace_y = []
    
    def update(frame):
        pos_x = [px_0[frame], px_1[frame], px_2[frame]]
        pos_y = [0, py_1[frame], py_2[frame]]
        
        if frame == 0:
                trace_x.clear()
                trace_y.clear()
                
        trace_x.append(px_2[frame])
        trace_y.append(py_2[frame])
        
        pend_plt.set_data(pos_x, pos_y)
        trace_plt.set_data(trace_x, trace_y)
        
        return pend_plt, trace_plt
    
    ani = animation.FuncAnimation(fig, update, int(tend/ts), interval=1000*ts, blit = True)
    
    #plt.show()
    plt.grid()