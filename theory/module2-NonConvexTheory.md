For a problem to be convex, it requires two things:

- An objective function that is convex
- A convex space or "feasible set" to optimize over

If we lose either one of those, we have non-convex spaces. Therefore, we need to take different approaches to optimizing these spaces

Examples of this include:
- shortest path with obstacles
- non-linear dynamics
- optimizing over a rotation manifold (SO(3))

When we have a non-convex feasible set but a convex function, we can
use Sequential Quadratic Programming to navigate around our (still) ocnvex 
function within the feasible set

### Steps to SQP

1. Take a guess of a parameter $x_k$
2. Linearize at that parameter via first-order taylor expansion of the constraints
    - **what does this mean?** - basically, we change our definition of a constraint to be the value of
3. Solve a quadratic program to find a step $d$ where $d = ||x^{*}_{k} - x_k||$
4. Move $x_{k+1} -> x_k + \alpha d$, where d is the value


### Interior Point Optimizer (Ipopt)

IPOPT handles inequality constraints by using a logarithmic barrier function, which penalizes invalid points and minimizes the objective + the log of an equality constraints

$$ \min f_0(x) - \mu \sum^{m}_{i=1}\ln(-f_i(x)) $$

