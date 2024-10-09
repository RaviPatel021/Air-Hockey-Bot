import pybullet as p
import pybullet_data
import time

# Set up the physics engine and create the simulation environment
physicsClient = p.connect(p.GUI)  # Connect to GUI for visual simulation
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane.urdf data
p.setGravity(0, 0, -9.8)  # Set gravity, though air hockey doesn't have much

# Load a flat plane to act as the air hockey table surface
planeId = p.loadURDF("air_hockey_table.urdf")

# Set the simulation time step and disable real-time simulation
p.setTimeStep(1/150)
p.setRealTimeSimulation(0)

# Create the puck (as a cylinder)
puckId = p.loadURDF("puck.urdf", basePosition=[0, 0, 0.05], globalScaling=1.0, flags=p.URDF_USE_IMPLICIT_CYLINDER)
p.changeDynamics(puckId, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)  # Low friction for sliding

# Create two paddles (as cylinders)
paddle_radius = 0
paddle1 = p.loadURDF("cylinder.urdf", basePosition=[-0.6, 0, 0.2], globalScaling=1, flags=p.URDF_USE_IMPLICIT_CYLINDER)
paddle2 = p.loadURDF("cylinder.urdf", basePosition=[0.6, 0, 1.08], globalScaling=1, flags=p.URDF_USE_IMPLICIT_CYLINDER)
p.changeDynamics(paddle1, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)
p.changeDynamics(paddle2, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)

# Function to move the player-controlled paddle
def move_paddle(paddle_id, target_position):
    current_position, _ = p.getBasePositionAndOrientation(paddle_id)
    p.setCollisionFilterGroupMask(paddle_id, -1, 1, 1)
    p.setCollisionFilterPair(paddle_id, puckId, -1, -1, 1)
    new_position = [target_position[0], target_position[1], target_position[2]]  # Keep the paddle above the surface
    p.resetBasePositionAndOrientation(paddle_id, new_position, [0, 0, 0, 1])

# Simulate and move the paddle in real time
while True:
    keys = p.getKeyboardEvents()  # Read keyboard input
    
    # Paddle 1 (controlled by user, moves with WASD)
    paddle1_pos, _ = p.getBasePositionAndOrientation(paddle1)
    if p.B3G_UP_ARROW in keys:
        paddle1_pos = [paddle1_pos[0], paddle1_pos[1] + 0.01, paddle1_pos[2]]
    if p.B3G_DOWN_ARROW in keys:
        paddle1_pos = [paddle1_pos[0], paddle1_pos[1] - 0.01, paddle1_pos[2]]
    if p.B3G_LEFT_ARROW in keys:
        paddle1_pos = [paddle1_pos[0] - 0.01, paddle1_pos[1], paddle1_pos[2]]
    if p.B3G_RIGHT_ARROW in keys:
        paddle1_pos = [paddle1_pos[0] + 0.01, paddle1_pos[1], paddle1_pos[2]]
    
    move_paddle(paddle1, paddle1_pos)
    
    # Simulate physics step
    p.stepSimulation()
    
    # Add a small delay to avoid overloading the CPU
    time.sleep(1/240)

p.disconnect()  # Disconnect the PyBullet client when done
