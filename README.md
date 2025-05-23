# Nonlinear-Model-Predictive-Control-for-Spacecraft-Rendezvous-and-Docking-with-a-Rotating-Target

Model of the Chaser Spacecraft is expressed by the following double-integrator type equations of motion:

$$\ddot{x}= \frac{F_x}{m},\quad\ddot{y}= \frac{F_y}{m},\quad \ddot{\theta}= \frac{\tau}{I_z}$$

There are six system states and three control inputs:

$$x=[x,y,\theta,\dot{x},\dot{y},\dot{\theta}]^T,\quad u=[F_x,F_y,\tau]^T$$

In state-space representation we may write:

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$$

Using Euler discretization method, we obtain:

$$\mathbf{x}(k+1) = A_d \mathbf{x}(k) + B_d \mathbf{u}(k)$$

Let us pose the Model Predictive Control-Control Barrier Function Optimization problem:

$$\\begin{aligned}
\\min\_{u_{t: t+N-1 \\mid t}} \\quad & \\frac{1}{2} \\tilde{x}\_N^T Q_x \\tilde{x}\_N+\\sum\_{k=0}^{N-1} \\frac{1}{2} \\tilde{x}\_k^T Q_x \\tilde{x}\_k\\\\
\\textrm{s.t.} \\quad 
 & x\_{t+k+1 \\mid t}=A\_d x\_{t+k \\mid t} + B\_d u\_{t+k \\mid t}, \\quad k=0,1,...,N-1 \\\\
 & x\_{t \\mid t}=x\_t,   \\\\
 & \\Delta h\\left(x\_{t+k \\mid t}, u\_{t+k \\mid t}\\right) \\geq-\\gamma h\\left(x\_{t+k \\mid t}\\right), \\quad k=0, \\ldots, N-1 \\\\
\\end{aligned}$$

where $\tilde x=x_k - x_{target,k}$. Also, The last constraint is non-linear which makes it a non-linear problem.

## Results
**Simulation**

The blue circle represents the chaser spacecraft(or robot) while the rotating target in green is the target spacecraft. Chaser Holding Radius/Position is in the direction of the docking port which decreases as the distance between the chaser and docking port decreases. This is done to ensure smooth docking and to avoid any collisions. The objects in black are the obstacles which serve as **debris** in outer space which the chaser spacecraft must avoid. This is done by leveraging **Control Barrier Functions** which ensure safety in chaser's overall path. I have discretized the control barrier function for each time step. The following simulation was done for a much shorter sampling time of 0.01s (IPOPT solver was taking less time than this at each iteration so this is fine)
<p align="center" width="100%">
    <img src="/images/path_animation1.gif" width="100%">
    <br>Path Animation
</p>

**Trajectory Plot**
<p align="center" width="100%">
    <img src="/images/plot_results2.png" width="100%">
    <br>Results Plot(sampling time of 0.1s and for 100 timesteps)
</p>

## References
[Nonlinear model predictive control for spacecraft rendezvous and docking with a rotating target](https://core.ac.uk/download/pdf/81223756.pdf)

[Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function](https://arxiv.org/pdf/2007.11718)
