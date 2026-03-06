# Double-Pendulum

This project is an interactive Python/Pygame simulation of a double pendulum with additional bouncing balls that can collide with each other and with pendulum bobs. It is designed as a visual playground for nonlinear dynamics and collision behavior.

## Demo


https://github.com/user-attachments/assets/80cea082-a2cc-430d-a46c-29351b80a573


## Project Overview

- Simulates a two-link pendulum system in real time.
- Integrates pendulum motion numerically each frame.
- Renders pendulum links, bobs, and ball trajectories.
- Supports ball-ball and ball-bob collisions with position correction to reduce overlap/sticking.
- Includes randomized ball reset behavior for repeated experiments.

## Physics & Math Concepts Used

- **Lagrangian mechanics**: The pendulum equations are based on coupled nonlinear equations of motion for a double pendulum.
- **Runge-Kutta integration (RK4)**: State updates are computed with a 4th-order Runge-Kutta method for better stability/accuracy than simple Euler updates.
- **Chaos theory**: The double pendulum is a classic chaotic system, showing strong sensitivity to initial conditions.
- **Rigid-body style collision impulse ideas (2D)**: Ball-bob interactions use impulse-based velocity updates along collision normals.
- **Coefficient of restitution**: Collisions model bounciness/energy loss.
- **Damping (drag-like term)**: Angular damping is used so pendulum motion can lose energy over time.

## Notes

Because this is a real-time game-loop simulation, behavior depends on timestep, damping, restitution, and collision handling choices. Small parameter changes can produce very different motion, especially due to chaotic dynamics.

## Requirements

- Python 3.10+
- `pygame`
- `numpy`

## Installation

1. Clone this repository.
2. Install dependencies:

	```bash
	pip install pygame numpy
	```

## Running the Simulation

From the project folder, run:

```bash
python3 pendulum.py
```

## Controls

- **R**: Randomize ball spawn heights and horizontal velocities.
- **Window close button**: Exit the simulation.

## Configuration / Tuning

The main parameters are in [pendulum.py](pendulum.py):

- `l1`, `l2`: Pendulum arm lengths
- `m1`, `m2`: Bob masses
- `state`: Initial pendulum angles/angular velocities
- `gravity`: Vertical acceleration applied to free balls
- Collision restitution and damping values in collision/derivative functions

If motion is too fast or unstable, reduce time step size, reduce restitution, or increase damping.

## Project Structure

- [pendulum.py](pendulum.py): Main simulation loop, physics updates, rendering, and collision handling
- `ball.py`: Ball object model and movement methods
- `bob.py`: Pendulum bob object model
- [README.md](README.md): Project documentation

## Known Limitations

- Real-time discrete collision detection may still miss extreme high-speed collisions.
- Collision handling is physically motivated but simplified for interactive performance.
- Chaotic systems are sensitive to small numerical differences across machines/settings.

## Future Improvements

- Continuous collision detection for high-speed bodies
- Better constraint-based pendulum-collision coupling
- UI controls for real-time parameter tweaking
- Optional energy plots / diagnostics

## Contributing

Contributions are welcome. Feel free to open an issue or submit a pull request with improvements to physics accuracy, performance, or usability.
