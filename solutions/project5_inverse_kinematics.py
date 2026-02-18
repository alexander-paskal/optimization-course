import pygame
import numpy as np
import casadi as cd



class State:

    HEIGHT = 800
    WIDTH = 800

    THETA_1 = np.radians(0)
    LENGTH_1 = 100

    THETA_2 =  np.radians(90)
    LENGTH_2 = 100


    CENTER = np.array((HEIGHT//2,WIDTH//2))
    CUR_POS = CENTER + np.array([LENGTH_1+LENGTH_2, LENGTH_1+LENGTH_2])

    FRAMERATE = 20



def forward_kinematics(theta1, theta2):
    j1 = np.round([np.cos(theta1) * State.LENGTH_1, np.sin(theta1) * State.LENGTH_1])
    j2 = np.round([
        np.cos(theta1+theta2) * State.LENGTH_2 + j1[0],
        np.sin(theta1+theta2) * State.LENGTH_2 + j1[1]
    ])
    j0 = State.CENTER
    return [j0, j1 + State.CENTER, j2 + State.CENTER]


def build_ik_solver():
    angles = cd.MX.sym('theta', 2)
    theta1 = angles[0]
    theta2 = angles[1]

    target = cd.MX.sym('target', 2)
    p1_x = State.LENGTH_1* np.cos(theta1)
    p1_y = State.LENGTH_1 * np.sin(theta1)

    p2_x = State.LENGTH_2 * np.cos(theta1 + theta2) + p1_x
    p2_y = State.LENGTH_2 * np.sin(theta1 + theta2) + p1_y

    objective = (p2_x - target[0])**2 + (p2_y - target[1])**2

    nlp = {
        'x': angles,
        'f': objective,
        'p': target
    }

    solver = cd.nlpsol('solver', 'ipopt', nlp, {
        'ipopt.print_level': 0,
        'print_time': 0
    })
    
    return solver


def inverse_kinematics(solver, target, initial_guess):
    result = solver(
        x0 = initial_guess,
        p=target,
        lbx=[-cd.inf, -cd.inf],
        ubx=[cd.inf, cd.inf]
    )

    angles = result['x'].full().flatten()
    return angles[0], angles[1]


def main():


    # render
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((State.HEIGHT, State.WIDTH))
    running = True
    solver = build_ik_solver()

    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEMOTION:
                State.CUR_POS = event.pos

        screen.fill((0,0,0))

        angles = inverse_kinematics(solver, State.CUR_POS - State.CENTER, (State.THETA_1, State.THETA_2))

        p0, p1, p2 = forward_kinematics(*angles)
        pygame.draw.circle(screen, "red", p0, 10)
        pygame.draw.line(screen, "green", p0, p1, 2)
        pygame.draw.circle(screen, "red", p1, 10)
        pygame.draw.line(screen, "green", p1, p2, 2)
        pygame.draw.circle(screen, "red", p2, 10)

        State.THETA_1 = angles[0]
        State.THETA_2 = angles[1]
        pygame.display.flip()
        clock.tick(State.FRAMERATE)

    pygame.quit()





if __name__ == "__main__":
    main()








