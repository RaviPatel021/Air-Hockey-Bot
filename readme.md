# PyBullet Air Hockey Simulation

This is a basic air hockey simulation created using the PyBullet physics engine. The simulation includes a puck and two paddles on an air hockey table. One paddle is controlled by the user using the keyboard (W, A, S, D) for movement. The puck and paddles have minimal friction to simulate the sliding effect on an air hockey table.

## Features
- User-controlled paddle via arrow keys (UP, DOWN, LEFT, RIGHT)
- Minimal friction for realistic puck movement
- Simple 2D movement of the paddle on the air hockey table
- Visual simulation using PyBullet's GUI

## Installation

1. Clone this repository:
    ```bash
    git clone https://github.com/RaviPatel021/Air-Hockey-Bot.git
    ```

2. Install the required dependencies:
    ```bash
    pip install -r requirements.txt
    ```

3. Run the simulation:
    ```bash
    python airhockey.py
    ```

## Controls
- **UP**: Move paddle up
- **DOWN**: Move paddle down
- **LEFT**: Move paddle left
- **RIGHT**: Move paddle right

## Dependencies
- [PyBullet](https://github.com/bulletphysics/bullet3): The physics engine used for the simulation.
- [pybullet_data](https://github.com/bulletphysics/bullet3/tree/master/data): Data files for loading URDFs and other objects.

## License
This project is licensed under the MIT License. See the `LICENSE` file for more information.
