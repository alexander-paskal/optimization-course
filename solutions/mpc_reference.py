import numpy as np
import cvxpy as cp

def solve_mpc(start_state, ref_traj, v, dt, N):
    """
    start_state: [X, Y, psi] (Global)
    ref_traj: List of [X_r, Y_r, psi_r, kappa] for the next N steps
    v: Constant velocity
    dt: Sampling time
    N: Prediction horizon
    """
    # 1. Setup Weights
    Q = np.diag([10.0, 1.0])  # Penalize [ey, psie]
    R = np.diag([0.1])        # Penalize [delta_omega]
    
    # 2. Extract Initial Error State
    # Find ey (lateral error) and psie (heading error)
    dx = start_state[0] - ref_traj[0][0]
    dy = start_state[1] - ref_traj[0][1]
    psi_r = ref_traj[0][2]
    
    # Lateral error (projection onto the normal vector of the path)
    ey_0 = -dx * np.sin(psi_r) + dy * np.cos(psi_r)
    psie_0 = start_state[2] - psi_r
    
    # Normalize psie to [-pi, pi]
    psie_0 = (psie_0 + np.pi) % (2 * np.pi) - np.pi
    
    x_0 = np.array([ey_0, psie_0])

    # 3. Define Discrete Error Dynamics
    # x_{k+1} = A x_k + B u_k
    A = np.array([[1.0, v * dt],
                  [0.0, 1.0]])
    B = np.array([[0.0],
                  [dt]])

    # 4. Formulate Optimization
    u = cp.Variable((1, N))
    x = cp.Variable((2, N + 1))
    
    cost = 0
    constraints = [x[:, 0] == x_0]
    
    for k in range(N):
        # Quadratic Cost: x.T @ Q @ x + u.T @ R @ u
        cost += cp.quad_form(x[:, k], Q) + cp.quad_form(u[:, k], R)
        
        # Dynamics Constraint
        constraints += [x[:, k+1] == A @ x[:, k] + B @ u[:, k]]
        
        # Physical Constraints (Example: max angular velocity deviation)
        constraints += [cp.abs(u[:, k]) <= 1.0] 

    # Terminal Cost
    cost += cp.quad_form(x[:, N], Q)

    # 5. Solve
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP)

    # 6. Reconstruct Absolute States and Controls
    u_opt = u.value.flatten() # This is delta_omega
    predicted_abs_states = []
    predicted_abs_controls = []
    
    curr_abs_state = np.array(start_state, dtype=float)
    
    for k in range(N):
        # Calculate absolute omega: omega = v*kappa + delta_omega
        kappa_k = ref_traj[k][3]
        omega_cmd = v * kappa_k + u_opt[k]
        
        predicted_abs_controls.append(omega_cmd)
        
        # Integrate for predicted global trajectory (Simple Unicycle)
        curr_abs_state[0] += v * np.cos(curr_abs_state[2]) * dt
        curr_abs_state[1] += v * np.sin(curr_abs_state[2]) * dt
        curr_abs_state[2] += omega_cmd * dt
        predicted_abs_states.append(curr_abs_state.copy())

    return predicted_abs_states, predicted_abs_controls