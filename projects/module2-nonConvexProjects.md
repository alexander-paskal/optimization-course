## Project 2.1: Local Obstacle Avoidance (Potential Fields)

Objective: Navigate a robot through a "forest" of circular obstacles. 

While simple, this demonstrates the "local minima" trap inherent in non-convexity.

Problem Type: Unconstrained Non-Convex Optimization.
The Math: $$\min J(x) = \|x - x_{goal}\|^2 + \sum \frac{\eta}{dist(x, obs_i)}$$.

Visualization: A robot moving toward a goal. If it gets stuck behind an obstacle, the "Potential Field" (the cost landscape) is visualized as a heatmap showing the trap.

High-Level Pseudocode:

```Python
# 1. Define attractive potential (Quadratic to goal).
# 2. Define repulsive potential (Exponential or Inverse-square from obstacles).
# 3. Use Gradient Descent: x_new = x_old - step * grad(J).
# 4. If grad(J) approx 0 but x != x_goal, robot is in a local minimum.
# 5. GUI: Draw the "force" vectors acting on the robot.
```


## Project 2.2: 2-Link Arm Inverse Kinematics (IK)

**Objective**: Find the joint angles $(\theta_1, \theta_2)$ to place an end-effector at a specific $(x, y)$ coordinate. The non-linearity of $\sin/\cos$ makes this non-convex.

**Problem Type**: Constrained Non-Convex Optimization.

**The Math**: $$\min \|\text{ForwardKinematics}(\theta) - \text{Target}\|^2$$ $$\text{s.t.}$$ $$\theta_{min} \leq \theta \leq \theta_{max}$$

**Visualization**: A 2-segment arm moving to follow a mouse cursor in real-time.High-Level 

**Pseudocode**:

```Python
# 1. Forward Kinematics: x = l1*cos(t1) + l2*cos(t1+t2); y = ...
# 2. Variables: theta1, theta2 = cp.Variable(2) # (Using a non-linear solver like Ipopt via CasADi)
# 3. Solver Setup:
#    - Define Objective: Euclidean distance error.
#    - Define Constraints: Joint limits (e.g., no backward bending).
# 4. Loop: Solve at each mouse movement; update arm segments in GUI.
```

## Project 2.3: Trajectory Optimization (SQP)

**Objective**: Solve for a full path for a car-like vehicle (non-holonomic) to park in a tight spot. This combines non-linear dynamics with non-convex collision constraints.

**Problem Type**: Sequential Quadratic Programming (SQP)

**The Math**: $$\min \sum \|u\|^2$$ $$\text{s.t.}$$ $$\dot{x} = f(x, u), dist(x, \text{walls}) > d$$.

**Visualization**: A vehicle "warping" its path iteratively until it finds a feasible, collision-free trajectory

```Python
# 1. Use CasADi to define the "Bicycle Model" (Non-linear differential equations).
# 2. Transcribe: Convert the continuous path into discrete points (N=50).
# 3. Set Constraints: 
#    - Non-linear: x[k+1] = integrator(x[k], u[k])
#    - Obstacle: (x-x_obs)^2 + (y-y_obs)^2 > r^2
# 4. Solve: Run SQP solver (Ipopt).
# 5. Animation: Display the solver's "thinking" process (iterations).
```
