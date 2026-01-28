# Objective: Estimate the 2D position of a robot based on "noisy" distance measurements from static beacons. 
# This demonstrates how Convex Optimization (specifically Least Squares) is the backbone of Localization.
# 
# 1. Setup: Define beacon locations (fixed) and weights (inverse of sensor noise).
# 2. Variable: x = cp.Variable(2) # Estimated (x, y)
# 3. Cost: cost = sum(w[i] * cp.norm(x - beacons[i], 2)**2)
# 4. Loop: 
#    - Generate noisy measurements: dist = true_dist + GaussianNoise()
#    - Solve: cp.Problem(cp.Minimize(cost)).solve()
#    - Update GUI: Draw a circle at x.value


import pygame
import numpy as np
import cvxpy as cp
import time

# Initialize



# Green = Truth
# Yellow = Measurement
# Blue = Estimate

GT_POS = np.array((400, 300), dtype=int)


def add_sensor_measurement(center, std, n = 1):

    # 1. Assume normal
    # 2. Pick a covariance and assign weight
    # 3. Sample from the distribution

    measurement = np.random.normal(np.repeat(center, n).reshape((-1, n)), std).T
    return measurement, np.tile(std, n)



def estimate(state):
    positions = []
    weights = []
    for (c, pos, _, std) in state:
        if c != "yellow":
            continue
        positions.append(pos)
        weights.append(1/std)
    positions = np.array(positions).astype(float)
    weights = np.array(weights).astype(float)
    
    # define problem
    x = cp.Variable(2)  # decision variable
    cost = cp.Minimize(cp.sum(  # syjmbolic cost function
        [w * cp.norm(x - p, 2)**2 for p, w in zip(positions, weights)]
    ))

    # solve problem
    problem = cp.Problem(cost)
    result = problem.solve()
    return x.value
    

def main():
    global GT_POS

    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    clock = pygame.time.Clock()
    running = True
    # Main Loop
    i = 0
    while running:
        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 2a. State`Logic
        if i % 60 == 0:
            p1, std1 = add_sensor_measurement(GT_POS, 10, 2)
            p2, std2 = add_sensor_measurement(GT_POS, 50, 20)
            p2 += np.array([50, 0]).reshape((1, 2))  # add drift
            positions = np.concatenate([p1, p2])
            stds = np.concatenate([std1, std2])
            state = [  # color, position, radius, std
                ("green", GT_POS, 10, 0)
            ]
            for pos, std in zip(positions, stds):
                state.append([
                    "yellow",
                    np.round(pos),
                    10 - (std/10),
                    std
                ])
            
            # estimate
            est = estimate(state)
            state.append([
                "blue",
                np.round(est),
                10,
                0
            ])

            # move state
            GT_POS += np.random.randint(-5, 5, size=2)
            GT_POS = np.clip(GT_POS, (0,0),(800-1,600-1))

        # 2b. Render Logic
        screen.fill("black")
        for color, pos, radius, std in state:
            pygame.draw.circle(screen, color, pos, radius)

        # 3. Render
        pygame.display.flip()
        clock.tick(60) # Limits to 60 FPS
        i += 1

    pygame.quit()


main()