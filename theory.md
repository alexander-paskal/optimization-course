# Module 1: Convex Optimization Theory

In autonomous systems, we prioritize convex formulations because they guarantee global optimality and possess polynomial-time complexity, making them suitable for high-frequency real-time loops (e.g., 100Hz control).


## 1. Mathematical Foundations

- A set $\mathcal{C}$ is convex if the line segment between any two points in $\mathcal{C}$ lies entirely within $\mathcal{C}$.
- A function $f: \mathbb{R}^n \to \mathbb{R}$ is convex if its Epigraph (the set of points on or above the graph) is a convex set.

### The Standard Form
A convex optimization problem is expressed as:

  $$\min f_0(x)$$
  $$ \text{subject to } f_i(x) \leq 0, \quad i = 1, \dots, m$$
  $$a_i^T x = b_i, \quad i = 1, \dots, p$$
  
- $f_0(x)$: The Objective Function Must be convex.
- $f_i(x)$: Inequality Constraints. Must be convex.
- $a_i^T x = b_i$: Equality Constraints. Must be affine (linear). Non-linear equalities destroy convexity.

## 2. Key Problem Classes in Autonomy
Most autonomous tasks fall into these three sub-classes:
  
A. Linear Programming (LP)The objective and all constraints are linear.
- Form: $\min c^T x$ s.t. $Ax \leq b$.
- Use Case: Fuel-optimal path planning or simple resource allocation.
  
B. Quadratic Programming (QP)The objective is quadratic ($P$ is positive semi-definite), and constraints are linear.
- Form: $\min \frac{1}{2}x^T P x + q^T x$ s.t. $Gx \leq h, Ax = b$.
- Use Case: The industry standard for MPC. $P$ usually weights the "cost" of error and control effort.
  
C. Second-Order Cone Programming (SOCP)Generalizes QPs by allowing "cone" constraints, like $\|Ax + b\|_2 \leq c^T x + d$.
- Use Case: Robotic grasping (friction cones) and rocket landing (thrust vector constraints).
  
## 3. Optimality Conditions (KKT)For a point $x^*$ to be the optimal solution, it must satisfy the Karush-Kuhn-Tucker (KKT) conditions (assuming Slater's condition holds):
- Primal Feasibility: All original constraints are satisfied ($f_i(x^*) \leq 0$).
- Dual Feasibility: Lagrange multipliers $\lambda$ for inequalities must be $\geq 0$.
- Complementary Slackness: $\lambda_i f_i(x^*) = 0$. (The multiplier is zero unless the constraint is "active" or "touching").
- Stationarity: The gradient of the Lagrangian is zero:

  $$\nabla f_0(x^*) + \sum \lambda_i \nabla f_i(x^*) + \sum \nu_i \nabla h_i(x^*) = 0$$

## 4. Duality

Every Primal problem has a Dual problem.
- Weak Duality: The dual solution provides a lower bound on the primal optimal value.
- Strong Duality: For convex problems, the primal and dual values are equal.
- Application: We often solve the Dual problem (or use Primal-Dual Interior Point methods) because it can be computationally cheaper for certain constraint structures.

## 5. Numerical Solvers
Autonomous systems rarely use "gradient descent" for these. Instead, we use:
- Active-Set Methods: Good for "warm-starting" (using the previous solution as a guess).
- Interior-Point Methods: Robust for large-scale problems.
- ADMM (Alternating Direction Method of Multipliers): Highly parallelizable for distributed systems.


