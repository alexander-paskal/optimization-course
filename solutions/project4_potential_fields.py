"""
Project 2.1: Local Obstacle Avoidance (Potential Fields)

Objective: Navigate a robot through a "forest" of circular obstacles using gradient descent
on a potential field combining attractive (goal) and repulsive (obstacle) forces.

Key improvements:
- Unified objective function for both visualization and gradient computation
- Correct gradient descent direction
- Consistent repulsive potential formulation
- Tunable parameters for better control
"""

import casadi as cd
import numpy as np
import pygame
from pygame.surfarray import blit_array


class PotentialField:
    """Unified potential field computation for both numpy arrays and CasADi symbolic variables"""
    
    def __init__(self, goal, obstacles, eta=2000.0, rho_0=100.0):
        """
        Args:
            goal: Goal position [x, y]
            obstacles: Array of obstacles, each row [x, y, radius]
            eta: Repulsion gain coefficient
            rho_0: Distance threshold - repulsion only applies within this distance
        """
        self.goal = goal
        self.obstacles = obstacles
        self.eta = eta
        self.rho_0 = rho_0
    
    def objective(self, x, use_casadi=False):
        """
        Compute total potential at position x.
        
        Works with both:
        - CasADi symbolic variables (for gradient computation)
        - NumPy arrays with shape (N, 2) for vectorized evaluation
        
        Args:
            x: Position(s) - either symbolic variable or numpy array
            use_casadi: Whether to use CasADi functions (norm_2) or NumPy (linalg.norm)
        
        Returns:
            Total potential J(x)
        """
        if use_casadi:
            norm = lambda vec: cd.norm_2(vec)
        else:
            # For numpy arrays with shape (N, 2), compute norm along axis 1
            norm = lambda vec: np.linalg.norm(vec, axis=1) if len(vec.shape) > 1 else np.linalg.norm(vec)
        
        # Attractive potential: quadratic cost pulling toward goal
        # J_att = 0.5 * ||x - x_goal||^2
        J_att = 0.5 * norm(x - self.goal) ** 2
        
        # Repulsive potential: sum over all obstacles
        J_rep = 0
        for i in range(self.obstacles.shape[0]):
            obs_center = self.obstacles[i, :2]
            obs_radius = self.obstacles[i, 2]
            
            # Distance from position to obstacle surface
            d = norm(x - obs_center) - obs_radius
            
            if use_casadi:
                # Simple inverse-square repulsion: eta / d^2
                # As d->0, cost->infinity; as d->infinity, cost->0
                d_safe = cd.fmax(d, 0.5)  # Prevent division by very small numbers
                repulsion_term = cd.if_else(
                    d < self.rho_0,
                    self.eta / (d_safe ** 2),
                    0
                )
                J_rep = repulsion_term
            else:
                # For numpy arrays
                d_safe = np.maximum(d, 0.5)  # Prevent division by very small numbers
                repulsion = self.eta / (d_safe ** 2)
                
                # Only apply where distance is less than threshold
                if isinstance(repulsion, np.ndarray):
                    repulsion = np.where(d < self.rho_0, repulsion, 0)
                else:
                    repulsion = repulsion if d < self.rho_0 else 0
                
                J_rep += repulsion
        
        return J_att + J_rep


def build_potential_map(potential_field, size):
    """
    Build a 2D visualization of the potential field.
    
    Args:
        potential_field: PotentialField instance
        size: Grid size (size x size)
    
    Returns:
        2D array of potential values, normalized to [0, 255] for display
    """
    # Create grid of coordinates
    coords_x, coords_y = np.meshgrid(np.arange(size), np.arange(size))
    coords = np.vstack([coords_x.flatten(), coords_y.flatten()]).T
    
    # Evaluate potential at all grid points
    potentials = potential_field.objective(coords, use_casadi=False)
    potentials = potentials.reshape((size, size))
    
    # Normalize for visualization using log scale to handle large variations
    potentials = potentials - potentials.min()
    potentials = np.log(potentials + 1)  # Log scale for better visualization
    potentials = (potentials - potentials.min()) / (potentials.max() - potentials.min())
    potentials = (255 * potentials).astype(np.uint8)
    
    return potentials


def main():
    """Main simulation loop"""
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((WORLD_SIZE, WORLD_SIZE))
    pygame.display.set_caption("Potential Field Navigation")
    
    # Create potential field
    potential_field = PotentialField(GOAL, OBSTACLES, eta=2000.0, rho_0=100.0)
    
    # Build visualization
    print("Building potential field map...")
    potential_map = build_potential_map(potential_field, WORLD_SIZE)
    
    # Create CasADi gradient function
    x_sym = cd.SX.sym("x", 2)
    J_sym = potential_field.objective(x_sym, use_casadi=True)
    grad_sym = cd.gradient(J_sym, x_sym)
    f_grad = cd.Function('f_grad', [x_sym], [grad_sym])
    
    # Initial robot position
    x = np.array([10.0, 10.0])
    step_size = 2.0
    
    # Path history for visualization
    path_history = [x.copy()]
    
    running = True
    paused = False
    
    print("Controls:")
    print("  SPACE - Pause/Resume")
    print("  R - Reset")
    print("  Q or ESC - Quit")
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    x = np.array([10.0, 10.0])
                    path_history = [x.copy()]
                elif event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
        
        # Clear screen and draw potential field
        screen.fill((0, 0, 0))
        blit_array(screen, potential_map.astype(np.uint32))
        
        # Update robot position (gradient descent)
        if not paused:
            # Compute gradient at current position
            grad = np.array(f_grad(x)).squeeze()
            grad_magnitude = np.linalg.norm(grad)
            
            # Check if we've reached goal or local minimum
            distance_to_goal = np.linalg.norm(x - GOAL)
            
            if distance_to_goal < 10:
                print(f"Goal reached! Distance: {distance_to_goal:.2f}")
                paused = True
            elif grad_magnitude < 0.1:
                print(f"Stuck in local minimum at {x}")
                paused = True
            else:
                # Gradient descent: move in negative gradient direction
                direction = -grad / grad_magnitude
                x = x + direction * step_size
                path_history.append(x.copy())
                
                # Keep path history reasonable size
                if len(path_history) > 1000:
                    path_history = path_history[-500:]
        
        # Draw path history
        if len(path_history) > 1:
            pygame.draw.lines(screen, (255, 100, 255), False, 
                            [(int(p[0]), int(p[1])) for p in path_history], 2)
        
        # Draw obstacles
        for obs in OBSTACLES:
            pygame.draw.circle(screen, (255, 255, 20), obs[:2].astype(int), int(obs[2]), width=3)
        
        # Draw goal as X
        goal_size = 10
        pygame.draw.line(screen, (0, 255, 0), 
                        GOAL - np.array([goal_size, goal_size]), 
                        GOAL + np.array([goal_size, goal_size]), 3)
        pygame.draw.line(screen, (0, 255, 0), 
                        GOAL - np.array([-goal_size, goal_size]), 
                        GOAL + np.array([-goal_size, goal_size]), 3)
        
        # Draw robot
        robot_pos = (int(x[0]), int(x[1]))
        pygame.draw.circle(screen, (255, 0, 0), robot_pos, 6)
        pygame.draw.circle(screen, (255, 100, 100), robot_pos, 6, width=2)
        
        # Draw gradient vector (force acting on robot)
        if not paused:
            grad = np.array(f_grad(x)).squeeze()
            if np.linalg.norm(grad) > 0.01:
                # Scale for visualization
                force_viz = -grad / np.linalg.norm(grad) * 30
                end_pos = (int(x[0] + force_viz[0]), int(x[1] + force_viz[1]))
                pygame.draw.line(screen, (0, 255, 255), robot_pos, end_pos, 2)
                pygame.draw.circle(screen, (0, 255, 255), end_pos, 4)
        
        # Status text
        font = pygame.font.Font(None, 24)
        status = "PAUSED" if paused else "RUNNING"
        dist_text = f"Distance to goal: {np.linalg.norm(x - GOAL):.1f}"
        status_surface = font.render(f"{status} | {dist_text}", True, (255, 255, 255))
        screen.blit(status_surface, (10, 10))
        
        pygame.display.flip()
        clock.tick(HZ)
    
    pygame.quit()
    print("Simulation ended.")


if __name__ == "__main__":
    # Simulation parameters
    HZ = 60
    WORLD_SIZE = 500
    
    # Obstacles: [x, y, radius]
    OBSTACLES = np.array([
        [300, 300, 30],
        [400, 100, 40],
        [200, 400, 35],
    ])
    
    # Goal position
    GOAL = np.array([450.0, 450.0])
    
    main()