# 1. Dynamics: Define A and B matrices for a falling mass (Gravity).
# 2. Variables: X (states over horizon T), U (thrust over horizon T).
# 3. Constraints:
#    - Dynamics: X[t+1] == A @ X[t] + B @ U[t]
#    - Final State: X[T] == [target_height, 0] (pos, vel)
#    - Thrust Limit: U <= MaxThrust, U >= 0
# 4. Objective: cp.Minimize(cp.norm(U, 1)) # L1 norm promotes sparsity/low fuel
# 5. Loop: Solve once, play back trajectory as an animation.



import pygame
import sys
import numpy as np
import cvxpy as cp

# Constants
WIDTH, HEIGHT = 800, 600
G = 32.17 # fps^2
DT = 0.1  # delta time
MAX_THRUST = np.array([500, 500])



# Dynamics ---> x_1 = A x_0 + B u, 
# where x is state
# A is transition matrix for vel to pos
# u is thrust input
# B is 
x = np.array([
    WIDTH/2, # x
    50,      # y
    0,       # x'
    0,       # y'
])

A = np.array([
    [1, 0, DT, 0],
    [0, 1, 0, DT],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

W = np.array([     # Models Wind Resistance
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0.99, 0],
    [0, 0, 0, 0.99]
])

g = np.array([
    0, 
    G  # fps^2 gravity
])

B = np.array([
    [0.5*DT**2, 0],
    [0, 0.5*DT**2],
    [DT, 0],
    [0, DT]
])




def get_controls(start_state, goal_state, look_ahead=10):
    # lookahead of 20 timesteps
    controls = [ # decision variable - control
        cp.Variable(2) for _ in range(look_ahead)
    ]


    # constraints
    constraints = []

    # Dynamics equality constraints
    states = [cp.Variable(4) for _ in range(look_ahead)]
    constraints.append(states[0] == start_state)

    for i in range(look_ahead - 1):
        constraints.append(
            states[i+1] == W @ A @ states[i] + B @ (g + controls[i])
        )

    # Thrust Cap inequality constraints
    for control in controls:
        constraints.append(cp.abs(control) <= MAX_THRUST)
    
    # set not-first controls to 0
    for control in controls[1:]:
        constraints.append(control == np.array([0, 0]))

    cost = cp.Minimize(
        0.0001 * cp.sum([cp.norm(c) for c in controls]) + 
        cp.norm(states[-1] - goal_state)
    )

    problem = cp.Problem(cost, constraints)
    result = problem.solve()
    return controls[0].value


pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Lander properties
pos = np.array([WIDTH // 2, 50])
vel = [0, 0]
lander_poly = [(0, -15), (10, 10), (-10, 10)] # Triangle shape




i = 0
while True:
    screen.fill((0, 0, 0)) # Space background
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()


    if i % (60*5) == 0:
        GOAL_STATE = np.array([*np.random.randint(100, 500, 2), 0, 0])
        
    # Draw Goal
    pygame.draw.circle(screen, (10, 255, 10), GOAL_STATE[:2], 10)

    lookahead = 10
    if i % lookahead == 0:
        controls = get_controls(x, GOAL_STATE, lookahead)
    else:
        controls = np.array([0, 0])

    x = W @ A @ x + B @ (g + controls)
    # x = A @ x + B @ g
    print(x)

    
    # Draw Lander
    transformed_poly = [(p[0] + x[0], p[1] + x[1]) for p in lander_poly]
    pygame.draw.polygon(screen, (255, 255, 255), transformed_poly)

    # Draw controls
    pygame.draw.line(screen, (120,120,120),(x[:2]), (x[0]-controls[0] // 2, x[1] - controls[1]//2))


    if x[1] > HEIGHT:
        pygame.quit()
        sys.exit()

    i += 1
    pygame.display.flip()
    clock.tick(10)