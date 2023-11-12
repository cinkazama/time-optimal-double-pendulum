# -*- coding: utf-8 -*-

import numpy as np
from numpy import matlib
from casadi import *
import system_params
import visualize
import dynamics as dyn

# casadi ocp object
opti = Opti()

m,g,m1,m2,l1,l2 = system_params.get_params()

# control intervals and end time
tend = opti.variable()
opti.set_initial(tend, 3)
N = 50   
ts = (tend/N)

# limits for force and track
ubound = 30
track_lim = 2

# start and end positions in state space
x0 = np.array([0, pi, pi, 0, 0, 0])
xend = np.array([0, 0, 0 ,0 ,0, 0])

# Get collocation points
d = 3 # degree of polynomial 
tau = collocation_points(d, 'legendre');

# Collocation linear maps for inter- and extrapolation
[C, D, B] = collocation_coeff(tau);

# decision variables
u = MX.sym('u') 
x = MX.sym('x', 6, 1)

xdot = dyn.dynamic_cart(0, x, 0, u)
F = Function('F', [x, u], [xdot])

cost = 0
# ensure correct initial value
Xk = opti.variable(6,1)
opti.subject_to(Xk == x0)
opti.set_initial(Xk, x0)

# create linspace for interpolation to give good inital values to helper states (crucial!)
x1_init = np.linspace(x0[1], xend[1], N)
x2_init = np.linspace(x0[2], xend[2], N)

X = Xk
U = MX.zeros(0,0)

for i in range(N):
    Ui = opti.variable()
    U = horzcat(U, Ui)
    opti.subject_to(Ui <= ubound)
    opti.subject_to(Ui >= -ubound)
    opti.set_initial(Ui, 0)
    
    # states at interpolation points
    X_co = opti.variable(6, d)
    X = horzcat(X, X_co)
    opti.subject_to(X_co[0] <= track_lim)
    opti.subject_to(X_co[0] >= -track_lim)
    
    # duplicate inital guess d times for d collocation points
    opti.set_initial(X_co, np.matlib.repmat([0, x1_init[i], x2_init[i], 0, 0, 0],3,1).transpose())

    # initial point and collocation points
    Z = horzcat(Xk, X_co)
    ode = F(X_co, Ui)
    
    # time scaled derivative of the polynomial at colloc points
    Pi_dt = Z@C/ts
    
    # polynomial has to match the system dynamics at collocation points
    opti.subject_to(Pi_dt == ode)
    
    # state at end of the interval from polynomial 
    X_co_end = Z@D

    # end state/start state for next interval
    Xk = opti.variable(6, 1)
    
    # ensure continuity
    opti.subject_to(Xk == X_co_end)
    
    # ensure track constraints
    opti.subject_to(Xk[0] <= track_lim)
    opti.subject_to(Xk[0] >= -track_lim)
    
    X = horzcat(X, Xk)

# ensure upright steady state position as terminal constraints
opti.subject_to(fabs(Xk[1:3]-pi) == pi)
opti.subject_to(Xk[3:6] == 0)
opti.subject_to(Xk[0] == 0)

## for xend[1] == pi and xend[2] == 0
# opti.subject_to(Xk[1] == pi)
# opti.subject_to(fabs(Xk[2]-pi) == pi)
# opti.subject_to(Xk[3:6] == 0)
# opti.subject_to(Xk[0] == 0)

# down swing
#opti.subject_to(Xk == xend)

opti.minimize(tend)
opti.solver('ipopt')

sol = opti.solve()

#extract solutions
xsol = sol.value(X)
usol = sol.value(U)
tend_sol = sol.value(tend)


def integrator_ode_cvodes():
    x = MX.sym('x',6,1)
    u = MX.sym('u',1)
    ts = MX.sym('ts',1)

    xdot = ts*dyn.dynamic_cart(0, x, 0, u)
    
    ode = {'x':x, 'p': vertcat(u, ts), 'ode':xdot}
    F = integrator('I', 'cvodes', ode)
    
    return F
    
def solve_dynamics_ode(U, x0):
    X = MX.zeros(6, N+1)
    x = x0
    X[:,0] = x
    F = integrator_ode_cvodes()
    for i in range(N):
        xnew = F(x0=x, p=vertcat(U[i],ts) )
        x = xnew['xf']
        X[:,i+1] = x
    return X
    
# verify results
ts = tend_sol/N
xsol_ver = evalf(solve_dynamics_ode(usol, x0)).full()

visualize.visualization(xsol[:,0::d+1], tend_sol, ts, d)
visualize.visualization(xsol_ver, tend_sol, ts, d)