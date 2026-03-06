import pygame
# from scipy.integrate import solve_ivp
import numpy as np
import math
from ball import Ball
from bob import Bob
import random

pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()

l1 = 100
l2 = 100
m1 = 500
m2 = 500

axis = (400, 200)

path = []

# Collision cooldown tracking
collision_cooldown = {}

state: list[float] = [math.pi/2, 0.1, 0.0, 0.1]

θ1 = state[0]
θ2 = state[2]

x1 = l1 * np.sin(θ1) + axis[0]
y1 = -l1 * np.cos(θ1) + axis[1]

x2 = x1 + l2 * np.sin(θ2)
y2 = y1 - l2 * np.cos(θ2)

bob1 = Bob(θ1, x1, x1, m1, 20, (255, 0, 0), 0.1)
bob2 = Bob(θ2, x2, x2, m2, 20, (255, 0, 0), 0.1)


# balls
vel = [10, 0.01]
vel2 = [4, 0.01]
vel3 = [-6, 0.01]
gravity = 0.3
balls: list[Ball] = []
b = Ball(400, 100, 50, 40, (255, 255, 0), vel)
b2 = Ball(200, 20, 400, 20, (255, 0, 0), vel2)
b3 = Ball(600, 40, 10, 30, (255, 0, 255), vel3)
balls.append(b)
balls.append(b2)
balls.append(b3)

def derivatives(t, state, L1, L2, m1, m2, g=-9.81, damping=0.1):
    θ1, ω1, θ2, ω2 = state  # angles and angular velocities
    Δθ = θ1 - θ2

    denom1 = L1 * (2*m1 + m2 - m2*np.cos(2*Δθ))
    denom2 = L2 * (2*m1 + m2 - m2*np.cos(2*Δθ))

    dω1 = (
        -g*(2*m1+m2)*np.sin(θ1)
        - m2*g*np.sin(θ1 - 2*θ2)
        - 2*np.sin(Δθ)*m2*(ω2**2*L2 + ω1**2*L1*np.cos(Δθ))
        - damping*ω1  # Air damping
    ) / denom1

    dω2 = (
        2*np.sin(Δθ) * (
            ω1**2*L1*(m1+m2)
            + g*(m1+m2)*np.cos(θ1)
            + ω2**2*L2*m2*np.cos(Δθ)
        )
        - damping*ω2  # Air damping
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

def check_collision(ball: Ball):
    # floor collision
    if ball.y + ball.radius > 600:
        ball.y = 600 - ball.radius
        ball.scale_velo(1, -0.8)
        if abs(ball.get_velo()[1]) < 0.5:
            ball.set_velo(ball.get_velo()[0], 0)
    
    # ceiling collision
    if ball.y - ball.radius <= 0:
        ball.y = ball.radius
        ball.scale_velo(1, -1)
    
    # wall collision
    if ball.x + ball.radius >= 800:
        ball.x = 800 - ball.radius
        ball.scale_velo(-0.8, 1)
    
    # wall collision
    if ball.x - ball.radius <= 0:
        ball.x = ball.radius
        ball.scale_velo(-0.8, 1)

def check_ball_collision(b1: Ball, b2: Ball):
    distance = ((b1.x - b2.x)**2 + (b1.y - b2.y)**2) ** 0.5
    if distance <= b1.radius + b2.radius:
        v1x, v1y = b1.get_velo()
        v2x, v2y = b2.get_velo()
        b1.velo[0] = calc_final_velo(b1.mass, b2.mass, v1x, v2x)
        b2.velo[0] = calc_final_velo(b2.mass, b1.mass, v2x, v1x)
        b1.velo[1] = calc_final_velo(b1.mass, b2.mass, v1y, v2y)
        b2.velo[1] = calc_final_velo(b2.mass, b1.mass, v2y, v1y)
        
        # Position correction to separate overlapping balls
        overlap = b1.radius + b2.radius - distance
        angle = math.atan2(b2.y - b1.y, b2.x - b1.x)
        b1.x -= overlap / 2 * math.cos(angle)
        b1.y -= overlap / 2 * math.sin(angle)
        b2.x += overlap / 2 * math.cos(angle)
        b2.y += overlap / 2 * math.sin(angle)

def check_ball_bob_collision(b1: Ball, b2: Bob):
    distance = ((b1.x - b2.x)**2 + (b1.y - b2.y)**2) ** 0.5
    if distance <= b1.radius + b2.radius:
        return True
    else:
        return False

"""
def ball_bob_collision(b1: Ball, b2: Bob):
    v1x, v1y = b1.get_velo()
    v2x, v2y = [-b2.get_velo() * np.sin(b2.θ), b2.get_velo() * np.cos(b2.θ)]
    b1.velo[0] = calc_final_velo(b1.mass, b2.mass, v1x, v2x)
    b1.velo[1] = calc_final_velo(b1.mass, b2.mass, v1y, v2y)

    x_comp = calc_final_velo(b2.mass, b1.mass, v2x, v1x)
    y_comp = calc_final_velo(b2.mass, b1.mass, v2y, v1y)
    if b2.get_velo() >= 0.0:
        return math.sqrt(x_comp**2 + y_comp**2) / b2.radius
    else:
        return -(math.sqrt(x_comp**2 + y_comp**2) / b2.radius)"""

def ball_bob_collision(ball: Ball, bob: Bob, bob_L: float, axis_pos, state, state_index):
    """Apply 2D rigid body collision impulse between ball and bob."""
    # Collision normal (from bob to ball)
    dx = ball.x - bob.x
    dy = ball.y - bob.y
    dist = math.sqrt(dx**2 + dy**2)
    
    if dist < 0.001:
        return
    
    nx = dx / dist
    ny = dy / dist
    
    # Vector from pivot to bob
    r_bob_x = bob.x - axis_pos[0]
    r_bob_y = bob.y - axis_pos[1]
    
    # Ball velocity
    v_ball_x, v_ball_y = ball.get_velo()
    
    # Bob's tangential velocity at collision point
    omega_bob = state[state_index]
    v_bob_x = -omega_bob * r_bob_y
    v_bob_y = omega_bob * r_bob_x
    
    # Relative velocity
    rel_vx = v_ball_x - v_bob_x
    rel_vy = v_ball_y - v_bob_y
    rel_v_normal = rel_vx * nx + rel_vy * ny
    
    # Skip if objects moving apart
    if rel_v_normal > 0:
        return
    
    # Physics parameters
    m_ball = ball.mass
    m_bob = bob.mass
    I_bob = m_bob * bob_L * bob_L  # Moment of inertia for point mass at distance L
    e = 0.3  # Coefficient of restitution (lower = more damping)
    
    # Cross product for rotation
    r_bob_cross_n = r_bob_x * ny - r_bob_y * nx
    
    # Impulse denominator
    denom = 1/m_ball + 1/m_bob + (r_bob_cross_n**2) / I_bob
    
    # Impulse magnitude
    j = -(1 + e) * rel_v_normal / denom
    
    # Apply impulse to ball
    ball.velo[0] += (j * nx) / m_ball
    ball.velo[1] += (j * ny) / m_ball
    
    # Apply impulse to bob's angular velocity
    state[state_index] += (r_bob_cross_n * j) / I_bob


# calculates final velocity after a collision
def calc_final_velo(m1, m2, v1, v2):
    return ((2 * m2 * v2) + m1 * v1 - m2 * v1) / (m1 + m2)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                b.y = random.randint(50, 200)
                b2.y = random.randint(50, 200)
                b3.y = random.randint(50, 200)
                if random.choice([True, False]):
                    b.velo[0] = random.randint(70, 100)
                else:
                    b.velo[0] = random.randint(-10, -7)

                if random.choice([True, False]):
                    b2.velo[0] = random.randint(7, 10)
                else:
                    b2.velo[0] = random.randint(-10, -7)
                
                if random.choice([True, False]):
                    b3.velo[0] = random.randint(70, 100)
                else:
                    b3.velo[0] = random.randint(-10, -7)
    
    screen.fill((0, 0, 0))  # clear screen
    # draw things here
    
    state = simulate(l1, l2, m1, m2, state, .1, 0.01)[-1]

    bob1.velo = bob1.radius * state[1]
    bob2.velo = bob2.radius * state[3]
    bob1.θ = state[0]
    bob2.θ = state[2]
    bob1.x = l1 * np.sin(bob1.θ) + axis[0]
    bob1.y = -l1 * np.cos(bob1.θ) + axis[1]
    bob2.x = bob1.x + l2 * np.sin(bob2.θ)
    bob2.y = bob1.y - l2 * np.cos(bob2.θ)

    for ball in balls:
        ball.move()
        check_collision(ball)

    check_ball_collision(b, b2)
    check_ball_collision(b, b3)
    check_ball_collision(b2, b3)

    for ball in balls:
        if check_ball_bob_collision(ball, bob1):
            # Check collision cooldown
            key = (id(ball), id(bob1))
            if key not in collision_cooldown or collision_cooldown[key] <= 0:
                distance = ((ball.x - bob1.x)**2 + (ball.y - bob1.y)**2) ** 0.5
                overlap = ball.radius + bob1.radius - distance
                angle = math.atan2(ball.y - bob1.y, ball.x - bob1.x)
                ball.x += overlap * math.cos(angle)
                ball.y += overlap * math.sin(angle)
                ball_bob_collision(ball, bob1, l1, axis, state, 1)
                collision_cooldown[key] = 5  # 5 frame cooldown
        if check_ball_bob_collision(ball, bob2):
            # Check collision cooldown
            key = (id(ball), id(bob2))
            if key not in collision_cooldown or collision_cooldown[key] <= 0:
                distance = ((ball.x - bob2.x)**2 + (ball.y - bob2.y)**2) ** 0.5
                overlap = ball.radius + bob2.radius - distance
                angle = math.atan2(ball.y - bob2.y, ball.x - bob2.x)
                ball.x += overlap * math.cos(angle)
                ball.y += overlap * math.sin(angle)
                ball_bob_collision(ball, bob2, l2, axis, state, 3)
                collision_cooldown[key] = 5  # 5 frame cooldown

    check_ball_bob_collision(b, bob1)

    if b.velo != 0:
        b.add_to_velo(0, gravity)

    if b2.velo != 0:
        b2.add_to_velo(0, gravity)
    
    if b3.velo != 0:
        b3.add_to_velo(0, gravity)

    for pt in path:
        pygame.draw.circle(screen, (0, 0, 255), pt, 1)

    pygame.draw.line(screen, (255, 255, 255), axis, (bob1.x, bob1.y), 2)
    pygame.draw.line(screen, (255, 255, 255), (bob1.x, bob1.y), (bob2.x, bob2.y), 2)

    bob1.draw(screen)
    bob2.draw(screen)
    path.append((bob2.x, bob2.y))

    b.draw(screen)
    b2.draw(screen)
    b3.draw(screen)

    pygame.display.flip()
    clock.tick(60)  # 60 FPS
    
    # Decrement cooldown timers
    for key in collision_cooldown:
        collision_cooldown[key] -= 1

pygame.quit()