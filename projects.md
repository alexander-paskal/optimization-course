# Module 1 - Convex Optimization

- create visualizations using PyGame
- use cvxpy as your optimization library

## Project 1 - Optimal State Estimation

Objective: Estimate the 2D position of a robot based on "noisy" distance measurements from static beacons. This demonstrates how Convex Optimization (specifically Least Squares) is the backbone of Localization.
- **Problem Type**: Unconstrained Convex Optimization.
- **The Math**: $\min_x \sum w_i \|x - b_i\|^2$, where $b_i$ are beacon locations and $w_i$ are confidence weights.
- **Visualization**: Beacons as static dots; the "true" robot position vs. the "estimated" position (moving average).

Peudocode:

```python
# 1. Setup: Define beacon locations (fixed) and weights (inverse of sensor noise).
# 2. Variable: x = cp.Variable(2) # Estimated (x, y)
# 3. Cost: cost = sum(w[i] * cp.norm(x - beacons[i], 2)**2)
# 4. Loop: 
#    - Generate noisy measurements: dist = true_dist + GaussianNoise()
#    - Solve: cp.Problem(cp.Minimize(cost)).solve()
#    - Update GUI: Draw a circle at x.value
```

## Project 2

Objective: Control a 1D or 2D "lander" to touch down at a specific coordinate with zero velocity while minimizing fuel consumption. This introduces Inequality Constraints (thrust limits).
- **Problem Type**: Linear Programming (LP) or Quadratic Programming (QP).
- **The Math**:

$$
\min \sum u_t \text{  (fuel usage) }
$$

$$
\text{    subject to   }
$$


$$
x_{t+1} = Ax_t + Bu_t \text{   (dynamics)  }
$$

$$
u_{min} \leq u_t \leq u_{max}  \text{   (fuel constraints)   }
$$

- **Visualization**:  A "lander" icon moving vertically. Thrust vectors shown as flames that change size based on $u_t$.

Pseudocode

```python
# 1. Dynamics: Define A and B matrices for a falling mass (Gravity).
# 2. Variables: X (states over horizon T), U (thrust over horizon T).
# 3. Constraints:
#    - Dynamics: X[t+1] == A @ X[t] + B @ U[t]
#    - Final State: X[T] == [target_height, 0] (pos, vel)
#    - Thrust Limit: U <= MaxThrust, U >= 0
# 4. Objective: cp.Minimize(cp.norm(U, 1)) # L1 norm promotes sparsity/low fuel
# 5. Loop: Solve once, play back trajectory as an animation.
```

## Project 3

Objective:
- **Problem Type**: The "Crown Jewel" of Module 1. A vehicle must track a curved centerline while staying within road boundaries. This introduces Model Predictive Control (MPC).
- **The Math**:

$$
\min \sum (x_{err}^T Q x_{err} + u^T R u)
$$ 

$$
\text {subject to }
$$

$$
y_{min} \leq y \leq y_{max} \text {  (road boundaries)}
$$


- **Visualization**: A top-down road view. The vehicle "sees" a green predicted trajectory line stretching forward, which it updates every frame.

Pseudocode

```python
# 1. State: [lateral_error, heading_error], Control: [steering_angle]
# 2. Objective: Penalize deviation from center and high steering rates (smoothness).
# 3. Constraints:
#    - Lateral position must stay within +/- RoadWidth.
#    - Steering angle limit (e.g., +/- 30 degrees).
# 4. Execution (Receding Horizon):
#    - At each frame: Solve QP for the next 10 steps.
#    - Apply ONLY the first step's steering.
#    - Move vehicle in PyGame, repeat.
```
