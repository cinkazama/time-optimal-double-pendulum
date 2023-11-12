# time-optimal-double-pendulum
time optimal swing-up of a double pendulum on a cart with casadi

Basic code to formulate and solve a optimal control problem (OPC) by using CasADi's Pyhton API.
Animations are created to illustrate the solution. 
The Minimum-Time OPC is formulated as 'direct collocation'.'single shooting' and 'multiple shooting' methods were tried but did not yield satisfactory results and are therefore not concluded.
The 'direct collocation' example from CasADi's docs helped in to build this.

By running the main script 'casadi_direct_collocation' the problem is formulated and solved. Some variations are given in the code and marked as comments.
Two animations are shown at the end. The first one visualizes the OCP solution. The second one uses 'cvodes' ODE solver to compute the 'exact' trajectory out of the solution for the manipulated variable.
