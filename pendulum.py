import pygame
# from scipy.integrate import solve_ivp
import numpy as np
import math

pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()

l1 = 100
l2 = 100
m1 = 10
m2 = 10

axis = (400, 200)

path = []

state: list[float] = [math.pi/2, 0.1, 0.0, 0.1]

def derivatives(t, state, L1, L2, m1, m2, g=-9.81):
    θ1, ω1, θ2, ω2 = state  # angles and angular velocities
    Δθ = θ1 - θ2

    denom1 = L1 * (2*m1 + m2 - m2*np.cos(2*Δθ))
    denom2 = L2 * (2*m1 + m2 - m2*np.cos(2*Δθ))

    dω1 = (
        -g*(2*m1+m2)*np.sin(θ1)
        - m2*g*np.sin(θ1 - 2*θ2)
        - 2*np.sin(Δθ)*m2*(ω2**2*L2 + ω1**2*L1*np.cos(Δθ))
    ) / denom1

    dω2 = (
        2*np.sin(Δθ) * (
            ω1**2*L1*(m1+m2)
            + g*(m1+m2)*np.cos(θ1)
            + ω2**2*L2*m2*np.cos(Δθ)
        )
    ) / denom2

    return np.array([ω1, dω1, ω2, dω2])  # [dθ1, dω1, dθ2, dω2]

def simulate(L1, L2, m1, m2, initial_state, t_max, dt):
    t = 0
    state = np.array(initial_state)  # [θ1, ω1, θ2, ω2]
    trajectory = [state.copy()]

    while t < t_max:
        k1 = derivatives(t,        state,          L1, L2, m1, m2)
        k2 = derivatives(t + dt/2, state + dt/2*k1, L1, L2, m1, m2)
        k3 = derivatives(t + dt/2, state + dt/2*k2, L1, L2, m1, m2)
        k4 = derivatives(t + dt,   state + dt*k3,   L1, L2, m1, m2)

        state = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        t += dt
        trajectory.append(state.copy())

    return np.array(trajectory)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    screen.fill((0, 0, 0))  # clear screen
    # draw things here
    
    state = simulate(l1, l2, m1, m2, state, .1, 0.01)[-1]
    θ1 = state[0]
    θ2 = state[2]

    x1 = l1 * np.sin(θ1) + axis[0]
    y1 = -l1 * np.cos(θ1) + axis[1]

    x2 = x1 + l2 * np.sin(θ2)
    y2 = y1 - l2 * np.cos(θ2)

    for pt in path:
        pygame.draw.circle(screen, (0, 0, 255), pt, 1)

    pygame.draw.line(screen, (255, 255, 255), axis, (x1, y1), 2)
    pygame.draw.line(screen, (255, 255, 255), (x1, y1), (x2, y2), 2)

    pygame.draw.circle(screen, (255, 0, 0), (x1, y1), 20)
    pygame.draw.circle(screen, (255, 0, 0), (x2, y2), 20)
    path.append((x2, y2))

    pygame.display.flip()
    clock.tick(60)  # 60 FPS

pygame.quit()