import pybullet as p
import pybullet_data
import time

# Set up the physics engine and create the simulation environment
physicsClient = p.connect(p.GUI)  # Connect to GUI for visual simulation
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane.urdf data
p.setGravity(0, 0, -9.8)  # Set gravity, though air hockey doesn't have much

# Load a flat plane to act as the air hockey table surface
planeId = p.loadURDF("plane.urdf")

# Set the simulation time step and disable real-time simulation
p.setTimeStep(1/240)
p.setRealTimeSimulation(0)

# Table dimensions
table_length = 1.8  # Length of the air hockey table (in meters)
table_width = 1.0   # Width of the air hockey table (in meters)
puck_radius = 0.0  # Radius of the puck (5 cm)

# Create table walls (as simple boxes)
def create_wall(position, half_extents):
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=half_extents)
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=half_extents)
    wall_id = p.createMultiBody(
        baseMass=0,  # Static objects should have mass 0
        baseCollisionShapeIndex=collision_shape_id, 
        baseVisualShapeIndex=visual_shape_id, 
        basePosition=position
    )
    return wall_id

# Create four walls for the air hockey table
wall_thickness = 0.05  # Thickness of the walls
wall_height = 0.02  # Height of the walls (small for air hockey)
create_wall([table_length / 2 + wall_thickness, 0, wall_height / 2], [wall_thickness, table_width / 2, wall_height])  # Right wall
create_wall([-table_length / 2 - wall_thickness, 0, wall_height / 2], [wall_thickness, table_width / 2, wall_height])  # Left wall
create_wall([0, table_width / 2 + wall_thickness, wall_height / 2], [table_length / 2, wall_thickness, wall_height])  # Top wall
create_wall([0, -table_width / 2 - wall_thickness, wall_height / 2], [table_length / 2, wall_thickness, wall_height])  # Bottom wall

# Create the puck (as a cylinder)
puckId = p.loadURDF("cylinder.urdf", basePosition=[0, 0, 0.05], globalScaling=1.0)
p.changeDynamics(puckId, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)  # Low friction for sliding

# Create two paddles (as cylinders)
paddle_radius = 0
paddle1 = p.loadURDF("cylinder.urdf", basePosition=[-0.6, 0, 0.08], globalScaling=1)
paddle2 = p.loadURDF("cylinder.urdf", basePosition=[0.6, 0, 0.08], globalScaling=1)
p.changeDynamics(paddle1, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)
p.changeDynamics(paddle2, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)

# Function to move the player-controlled paddle
def move_paddle(paddle_id, target_position):
    current_position, _ = p.getBasePositionAndOrientation(paddle_id)
    p.setCollisionFilterGroupMask(paddle_id, -1, 1, 1)
    p.setCollisionFilterPair(paddle_id, puckId, -1, -1, 1)
    new_position = [target_position[0], target_position[1], paddle_radius]  # Keep the paddle above the surface
    p.resetBasePositionAndOrientation(paddle_id, new_position, [0, 0, 0, 1])

# Simulate and move the paddle in real time
while True:
    keys = p.getKeyboardEvents()  # Read keyboard input
    
    # Paddle 1 (controlled by user, moves with WASD)
    paddle1_pos, _ = p.getBasePositionAndOrientation(paddle1)
    if ord('w') in keys:
        paddle1_pos = [paddle1_pos[0], paddle1_pos[1] + 0.01, paddle1_pos[2]]
    if ord('s') in keys:
        paddle1_pos = [paddle1_pos[0], paddle1_pos[1] - 0.01, paddle1_pos[2]]
    if ord('a') in keys:
        paddle1_pos = [paddle1_pos[0] - 0.01, paddle1_pos[1], paddle1_pos[2]]
    if ord('d') in keys:
        paddle1_pos = [paddle1_pos[0] + 0.01, paddle1_pos[1], paddle1_pos[2]]
    
    move_paddle(paddle1, paddle1_pos)
    
    # Simulate physics step
    p.stepSimulation()
    
    # Add a small delay to avoid overloading the CPU
    time.sleep(1/240)

p.disconnect()  # Disconnect the PyBullet client when done
