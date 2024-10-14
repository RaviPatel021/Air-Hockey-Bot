import pybullet as p
import pybullet_data
import time

class AirHockeySimulation:
    def __init__(self):
        # Set up the physics engine and simulation environment
        self.physicsClient = p.connect(p.GUI)  # Connect to GUI for visual simulation
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane.urdf data
        p.setGravity(0, 0, -9.8)  # Set gravity, though air hockey doesn't have much
        
        # Load the air hockey table
        self.planeId = p.loadURDF("air_hockey_table.urdf")

        # Set simulation time step and disable real-time simulation
        p.setTimeStep(1/240)
        p.setRealTimeSimulation(1)

        # Initialize objects in the simulation
        self.puckId = self.create_puck()
        self.paddle1, self.paddle2 = self.create_paddles()

    def create_puck(self):
        # Define puck dimensions
        puck_radius = 0.08  # 5 cm radius
        puck_height = 0.05  # 1 cm height

        # Create the puck's collision and visual shapes
        puck_collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=puck_radius, height=puck_height)
        puck_visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=puck_radius, length=puck_height, rgbaColor=[1, 0, 0, 1])

        # Add the puck to the world
        puckId = p.createMultiBody(baseMass=0.2,  # Mass of the puck
                                   baseCollisionShapeIndex=puck_collision_shape,
                                   baseVisualShapeIndex=puck_visual_shape,
                                   basePosition=[0, 0, puck_height / 2])

        # Set puck friction properties
        p.changeDynamics(puckId, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)  # Low friction for sliding
        return puckId

    def create_paddles(self):
        # Define paddle dimensions
        paddle_radius = 0.1  # 10 cm radius
        paddle_height = 0.05  # 5 cm height

        # Create paddle collision and visual shapes
        paddle_collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=paddle_radius, height=paddle_height)
        paddle_visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=paddle_radius, length=paddle_height, rgbaColor=[0, 0, 1, 1])

        # Add paddle 1 to the world (left side)
        paddle1 = p.createMultiBody(baseMass=1.0,  # Mass of paddle 1
                                    baseCollisionShapeIndex=paddle_collision_shape,
                                    baseVisualShapeIndex=paddle_visual_shape,
                                    basePosition=[-0.6, 0, paddle_height / 2])

        # Add paddle 2 to the world (right side)
        paddle2 = p.createMultiBody(baseMass=1.0,  # Mass of paddle 2
                                    baseCollisionShapeIndex=paddle_collision_shape,
                                    baseVisualShapeIndex=paddle_visual_shape,
                                    basePosition=[0.6, 0, paddle_height / 2])

        # Set paddles' friction properties
        p.changeDynamics(paddle1, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)
        p.changeDynamics(paddle2, -1, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)

        return paddle1, paddle2

    def move_paddle(self, paddle_id, target_position):
        """Move a paddle to the target position, keeping its Z axis fixed."""
        paddle_height = 0.05  # 5 cm height for keeping the Z coordinate fixed
        new_position = [target_position[0], target_position[1], paddle_height / 2]
        p.resetBasePositionAndOrientation(paddle_id, new_position, [0, 0, 0, 1])

    def apply_action(self, paddle_id, x_velocity, y_velocity):
        """Apply velocity to the paddle in the x and y directions."""
        # Get the current velocity of the paddle
        linear_velocity, angular_velocity = p.getBaseVelocity(paddle_id)

        # Apply new velocities in the x and y directions
        new_velocity = [x_velocity, y_velocity, linear_velocity[2]]  # Keep Z velocity unchanged

        # Set the new linear velocity for the paddle
        p.resetBaseVelocity(paddle_id, new_velocity, angular_velocity)

    def apply_keyboard_controls(self):
        """Control the paddle movement using keyboard input with incremental velocity change."""
        keys = p.getKeyboardEvents()  # Read keyboard input
        
        # Paddle 1 (controlled by arrow keys)
        increment = 0.05  # Velocity increment factor
        
        # Get current velocity of paddle1
        current_velocity, _ = p.getBaseVelocity(self.paddle1)  # Get linear velocity
        
        # Initialize velocity change
        x_velocity, y_velocity = 0, 0
        
        # Increment or decrement the velocity based on the key press
        if p.B3G_UP_ARROW in keys:
            y_velocity += increment
        if p.B3G_DOWN_ARROW in keys:
            y_velocity -= increment
        if p.B3G_LEFT_ARROW in keys:
            x_velocity -= increment
        if p.B3G_RIGHT_ARROW in keys:
            x_velocity += increment

        # Add the current velocity to the new velocity
        new_x_velocity = current_velocity[0] + x_velocity
        new_y_velocity = current_velocity[1] + y_velocity

        max_vel = 1

        new_x_velocity = min(new_x_velocity, max_vel)
        new_y_velocity = min(new_y_velocity, max_vel)
        
        
        # Apply the new velocity to paddle1
        self.apply_action(self.paddle1, new_x_velocity, new_y_velocity)



    def simulate(self):
        """Main loop to run the simulation."""
        while True:
            # Apply keyboard controls for the paddles
            self.apply_keyboard_controls()

            # Step simulation
            p.stepSimulation()

            # Add a small delay to avoid overloading the CPU
            time.sleep(1/240)

    def disconnect(self):
        """Disconnect from the simulation."""
        p.disconnect()

# Example usage
if __name__ == "__main__":
    simulation = AirHockeySimulation()
    try:
        simulation.simulate()
    except KeyboardInterrupt:
        # Gracefully disconnect on exit
        simulation.disconnect()
