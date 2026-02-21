import pygame
import numpy as np
import casadi as cd



class State:
    
    HEIGHT = 800
    WIDTH = 800
    CENTER = np.array([HEIGHT // 2, WIDTH // 2])
    
    
    NUM_WAYPOINTS = 50
    VEHICLE_LENGTH = 60
    VEHICLE_WIDTH = 20

    VEHICLE_POS = np.array((0, 0))
    OBSTACLE_LENGTHS = [60, 60]
    OBSTACLE_WIDTHS = [20, 20]
    OBSTACLE_POSITIONS = [
        (70, 0),
        (70, 160)
    ]
    VEHICLE_HEADING = -np.radians(0)
    VEHICLE_VELOCITY = 1


    MAX_X = 100
    MAX_Y = 150
    MIN_X = -10
    MIN_Y = -10 

    DT = 1



def get_kinematics(x, y, theta, v, alpha, delta, dt, L):
    '''
    Kinematic Bicycle Model
    x' = v cos (theta)
    y' = v sin(theta)
    theta' = v / L tan(delta)

    theta -> heading
    delta -> steering angle of front wheels
    '''
    A = np.eye(4) + np.array([
        [0, 0, -np.sin(theta) * v, -np.cos(theta)],
        [0, 0, np.cos(theta) * v, np.sin(theta)],
        [0, 0, 0, np.tan(delta) / L],
        [0, 0, 0, 0]
    ]) * dt

    B = np.array([
        [0,  0],
        [0, 0],
        [0, v / (L * np.cos(delta)**2)],
        [1, 0]
    ]) * dt

    return A, B

def car_points(pos, theta, L, W):
    back_left = pos - np.array([np.cos(theta)*W/2, np.sin(theta)*W/2])
    back_right = pos + np.array([np.cos(theta)*W/2, np.sin(theta)*W/2])
    front_left = back_left + np.array([np.cos(theta+np.radians(90)) * L, np.sin(theta+np.radians(90))*L])
    front_right = back_right + np.array([np.cos(theta+np.radians(90)) * L, np.sin(theta+np.radians(90))*L])
    return np.vstack([
        back_left, back_right,  front_right, front_left
    ]).astype(int)

def kinematics(state, control):

    A, B = get_kinematics(*state, *control, State.DT, State.VEHICLE_LENGTH)
    new_state = A@state + B@control
    return new_state


def build_solver(num_waypoints):
    traj    = [cd.SX.sym(f"traj_{i}_{j}")    for i in range(num_waypoints) for j in range(4)]
    control = [cd.SX.sym(f"control_{i}_{j}") for i in range(num_waypoints) for j in range(2)]
    dvs = []
    for i in range(num_waypoints):
        dvs.extend(traj[4*i:4*i+4])
        dvs.extend(control[2*i:2*i+2])
    target  = [cd.SX.sym(f"target_{j}")      for j in range(4)]

    constraints = []
    lbg = []
    ubg = []

    # Kinematic equality constraints: x_{k+1} = f(x_k, u_k)
    for i in range(1, num_waypoints):
        x_k  = [traj[(i-1)*4 + j]    for j in range(4)]
        u_k  = [control[(i-1)*2 + j] for j in range(2)]
        x_k1 = [traj[i*4 + j]        for j in range(4)]
        residual = kinematics(x_k, u_k)
        for j in range(4):
            constraints.append(x_k1[j] - residual[j])
            lbg.append(0.0)
            ubg.append(0.0)

    # Terminal equality constraints: x_N == target
    last = (num_waypoints - 1) * 4
    for j in range(4):
        constraints.append(traj[last + j] - target[j])
        lbg.append(0.0)
        ubg.append(0.0)

    # Objective: minimize total control effort
    objective = 0
    for k in range(num_waypoints):
        for j in range(2):
            objective += control[k*2 + j]**2

    nlp = {
        'x': cd.vertcat(*dvs),
        'f': objective,
        'p': cd.vertcat(*target),
        'g': cd.vertcat(*constraints),
    }

    solver = cd.nlpsol('solver', 'ipopt', nlp, {
        'ipopt': {'print_level': 0},
        'print_time': False,
    })

    return solver, lbg, ubg

def run_trajectory_optimization(start, goal, solver, lbg, ubg):
    # interp_traj = linear_interp_states(start, goal, State.NUM_WAYPOINTS)
    # initial_guess = np.zeros((State.NUM_WAYPOINTS, 6))
    # initial_guess[:, :4] = interp_traj
    initial_guess=np.zeros((State.NUM_WAYPOINTS*6))
    sol = solver(x0=initial_guess.flatten(), lbg=lbg,ubg=ubg)
    z_opt = sol['x'].full().flatten()
    z_opt = z_opt.reshape((-1, 6))
    
    traj = z_opt[:, :4]
    controls = z_opt[:, 4:]
    return traj, controls



def linear_interp_states(start, goal, num_waypoints):
    states = np.linspace(start, goal, num_waypoints)
    return states






def main():
    
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((State.HEIGHT, State.WIDTH))

    running = True
    state = np.array([
        *State.VEHICLE_POS,
        State.VEHICLE_HEADING,
        State.VEHICLE_VELOCITY
    ])
    target = [70, 130, np.radians(180), 0]
    # traj = linear_interp_states(state, target, State.NUM_WAYPOINTS)
    solver, lbg, ubg = build_solver(State.NUM_WAYPOINTS)
    traj, controls = run_trajectory_optimization(state, target, solver, lbg, ubg)

    i = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill((0,0,0))


        if i < State.NUM_WAYPOINTS:
            control = controls[i]
            state = kinematics(state, control)

        State.VEHICLE_POS = state[:2]
        State.VEHICLE_HEADING = state[2]
        State.VEHICLE_VELOCITY = state[3]

        # draw car
        for l, w, pos in zip(State.OBSTACLE_LENGTHS, State.OBSTACLE_WIDTHS, State.OBSTACLE_POSITIONS):
            pygame.draw.lines(screen, "red", closed=True, points=car_points(pos, 0, l, w) + State.CENTER, width=2)

        l,w,pos,theta = (State.VEHICLE_LENGTH,State.VEHICLE_WIDTH,State.VEHICLE_POS, State.VEHICLE_HEADING)
        pygame.draw.lines(screen, "green", closed=True, points=car_points(pos,theta,l,w) + State.CENTER, width=2)

        i += 1
        pygame.display.flip()
        clock.tick(20)
    pygame.quit()



if __name__ == "__main__":
    main()