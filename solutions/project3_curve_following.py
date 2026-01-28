# 1. State: [lateral_error, heading_error], Control: [steering_angle]
# 2. Objective: Penalize deviation from center and high steering rates (smoothness).
# 3. Constraints:
#    - Lateral position must stay within +/- RoadWidth.
#    - Steering angle limit (e.g., +/- 30 degrees).
# 4. Execution (Receding Horizon):
#    - At each frame: Solve QP for the next 10 steps.
#    - Apply ONLY the first step's steering.
#    - Move vehicle in PyGame, repeat.


import pygame
import numpy as np
import cvxpy as cp
from scipy.interpolate import CubicSpline

# Parameters
WIDTH = 800
HEIGHT = 600

WAYPOINTS = np.array([
    (100,100),
    (100,300),
    (300,300),
    (300,100),
    (500,100),
    (500,300),
    (700,300),
    (700,100)
])

def calculate_spline(points, num_samples=200):
    """Calculate cubic spline interpolation through the waypoints."""
    if len(points) < 2:
        return []
    
    # Separate x and y coordinates
    x_coords = np.array([p[0] for p in points])
    y_coords = np.array([p[1] for p in points])
    
    # Create parameter t for each waypoint
    t = np.arange(len(points))
    
    # Create cubic splines for x and y
    cs_x = CubicSpline(t, x_coords)
    cs_y = CubicSpline(t, y_coords)
    
    # Generate smooth curve points
    t_smooth = np.linspace(0, len(points) - 1, num_samples)
    x_smooth = cs_x(t_smooth)
    y_smooth = cs_y(t_smooth)
    
    return np.array(list(zip(x_smooth, y_smooth)))

SPLINE = calculate_spline(WAYPOINTS, num_samples=200)
SPLINE_LENGTH = np.sum(np.linalg.norm(SPLINE[1:] - SPLINE[:-1], axis=1))
print(SPLINE_LENGTH)

### Vehicle Dynamics
# Ax + By
# A = Dynamics Matrix
# B = Control Matrix
FRAMERATE = 5
DT = 1/FRAMERATE
TARGET_SPEED = 30  # pixels/second

X = np.array([
    WAYPOINTS[0][0],
    WAYPOINTS[0][1],
    0,
    0
])

# Todo - fix ackerman dynamics and visualize reference trajectories



def get_dynamics():
    A = np.array([
        [1, 0, DT, 0],
        [0, 1, 0, DT],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    B = np.array([
        [0.5*DT**2, 0],
        [0, 0.5*DT**2],
        [DT, 0],
        [ 0, DT],
    ])
    return A, B

def wrap_theta(state):
    if state[2] > 2*np.pi:
        state[2] -= 2*np.pi 
    elif state[2] < 0:
        state[2] += 2*np.pi



def vehicle_dynamics(state, control, use_cvxpy=False):
    # state is (x,y,theta,v)
    # control is (theta',v')
    A, B = get_dynamics()
    next_state = A@state + B@control
    # wrap_theta(next_state)
    return next_state

POLYGON = np.array([
    [0, 20],
    [10, 10],
    [0, 0]
])



def calculate_curvature(points):
    """points: array of shape (N, 2)"""
    x = points[:, 0]
    y = points[:, 1]
    
    # First derivatives
    dx = np.gradient(x)
    dy = np.gradient(y)
    
    # Second derivatives
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # Curvature formula
    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    return curvature

# CURVATURE = calculate_curvature(SPLINE)


# optimization
def run_mpc(cur_state, ref_traj, dt):

    # change to error states
    # define an initial error state

    
    # define error dynamics, incorporating curvature

    controls = [cp.Variable(2) for i in range(len(ref_traj))]

    constraints = []
    # control constraints
    for i in range(len(ref_traj) - 1):
        constraints.append(cp.abs(controls[i]) <= np.array([20, 20]))
    
    # Objective
    states = [cur_state]
    state = cur_state
    for i in range(len(ref_traj)-1):
        A, B = get_dynamics()
        state = A@state + B@controls[i]
        states.append(state)
    
    discount = 0.9
    objective = cp.sum(
        [cp.norm(states[i][:2] - ref_traj[i][:2])*discount**i for i in range(1, len(ref_traj))]
    )

    problem = cp.Problem(cp.Minimize(objective), constraints)
    result = problem.solve()

    return np.array([state.value if not isinstance(state, np.ndarray) else state for states in states]), np.array([control.value if control.value is not None else [0,0] for control in controls])


# build reference
def get_ref_traj(x, spline, v_target, dt, lookahead):
    thetas = np.arctan2(
        *(spline[1:] - spline[:-1]).T
    )
    thetas = np.pad(thetas, (1,0))
    segments = np.linalg.norm(spline[1:]-spline[:-1], axis=1)
    segments = np.pad(segments, (1,0))

    # Find nearest point
    p_ind = np.argmin(
        np.linalg.norm(spline - x[:2].reshape((-1, 2)), axis=1)
    )
    # Step along it at time intervals according to my velocity
    # 

    ref_traj = [
        (spline[p_ind][0],spline[p_ind][1])
    ]

    while len(ref_traj) < lookahead:
        dist = 0
        while dist < v_target * dt:
            p_ind += 1

            if p_ind == len(segments):
                return np.array(ref_traj), True
            
            dist += segments[p_ind]
        ref_traj.append(
            (
                spline[p_ind][0],
                spline[p_ind][1],
            )
        )
    
    ref_traj = np.array(ref_traj)
    return ref_traj, False

    

def transformed_polygon():

    theta = np.arctan2(X[3], X[2])
    if theta == np.nan:
        theta = 0
    rotation_matrix = np.array([
        [np.cos(theta), np.sin(theta)],
        [-np.sin(theta), np.cos(theta)],
    ])
    rotated = POLYGON @ rotation_matrix
    translated = rotated + X[:2].reshape(1, 2)
    return translated

def main():
    global X
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    pygame.init()

    running = True
    keydown = None
    prev_control = None
    step = 0
    ref_traj = []
    while running:
        screen.fill((0, 0, 0))
        control = np.array([0, 0], dtype=float)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        keypressed = pygame.key.get_pressed()

        # print(event.unicode)
        if keypressed[pygame.K_a]:
            control[1] = 1
        elif keypressed[pygame.K_w]:
            control[0] = 10
        elif keypressed[pygame.K_d]:
            control[1] = -1
        elif keypressed[pygame.K_x]:
            control[0] = -10
        elif keypressed[pygame.K_s]:
            X[2:] = [0, 0]
                
            

        # draw lines
        pygame.draw.lines(screen, "blue", False, WAYPOINTS, width=5)
        for i in range(len(WAYPOINTS)):
            pygame.draw.circle(screen, (10,255,10), WAYPOINTS[i], 20)
        pygame.draw.lines(screen, "green", False, SPLINE, width=3)

        

        # show ref trajectory
        # if step % FRAMERATE == 0:
        ref_traj, early_termination = get_ref_traj(X, SPLINE, TARGET_SPEED, DT, 10)
        if early_termination:
            break
        
        # mpc_states, mpc_controls = run_mpc(X, ref_traj, DT*FRAMERATE)
        mpc_states, mpc_controls = run_mpc(X, ref_traj, DT)
        # for s in mpc_states:
        #     pygame.draw.circle(screen, "yellow", s[:2], radius=2)

        control = mpc_controls[0]

        for p in ref_traj:
            pygame.draw.circle(screen, "turquoise", p[:2], radius=5)


        print(X)
        X = vehicle_dynamics(X, control)

        # draw car
        polygon = transformed_polygon()
        pygame.draw.polygon(screen, "white", polygon)
        pygame.display.flip()   
        clock.tick(FRAMERATE)
        step += 1
    
    print("All Done")
    pygame.quit()



if __name__ == "__main__":
    main()