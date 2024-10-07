import pybullet as p
import pybullet_utils.bullet_client as bc
import time
import numpy as np
import os
import keyboard  # Install this library using `pip install keyboard`


class AirHockeyEnv:
    def __init__(self):
        # Initialize the PyBullet client
        self.client = bc.BulletClient(connection_mode=p.GUI)  # Change to p.DIRECT for no GUI

        # Load URDF models
        self.puck_id = self.client.loadURDF("puck.urdf", basePosition=[0.0, 0, 0.05])
        self.table_id = self.client.loadURDF("air_hockey_table.urdf", basePosition=[0.0, 0, -0.189], useFixedBase=True)
        
        # Load mallets
        self.mallet1_id = self.client.loadURDF("cylinder.urdf", basePosition=[-0.6, 0, 0.05])
        self.mallet2_id = self.client.loadURDF("cylinder.urdf", basePosition=[0.6, 0, 0.05])

        # Set gravity
        self.client.setGravity(0, 0, -9.81)

        # Set simulation parameters
        self.client.setTimeStep(1 / 240)

    def reset(self):
        # Reset puck position
        self.client.resetBasePositionAndOrientation(self.puck_id, [0.0, 0, 0.05], [0, 0, 0, 1])
        # Reset mallets positions
        self.client.resetBasePositionAndOrientation(self.mallet1_id, [-0.6, 0, 0.05], [0, 0, 0, 1])
        self.client.resetBasePositionAndOrientation(self.mallet2_id, [0.6, 0, 0.05], [0, 0, 0, 1])

    def step(self):
        # Step the simulation
        self.client.stepSimulation()
        time.sleep(1 / 240)  # Control the speed of the simulation

    def apply_action(self, mallet_id, force):
        # Apply a force to the mallet
        self.client.applyExternalForce(mallet_id, -1, force, [0, 0, 0], self.client.WORLD_FRAME)

    def visualize_state(self):
        # Visualize puck position
        puck_pos = self.client.getBasePositionAndOrientation(self.puck_id)[0]
        self.client.addUserDebugLine(puck_pos, [puck_pos[0], puck_pos[1], 0], lineColorRGB=[1, 0, 0], lineWidth=5)
        
        # Visualize mallet positions
        mallet1_pos = self.client.getBasePositionAndOrientation(self.mallet1_id)[0]
        mallet2_pos = self.client.getBasePositionAndOrientation(self.mallet2_id)[0]
        self.client.addUserDebugLine(mallet1_pos, [mallet1_pos[0], mallet1_pos[1], 0], lineColorRGB=[0, 0, 1], lineWidth=5)
        self.client.addUserDebugLine(mallet2_pos, [mallet2_pos[0], mallet2_pos[1], 0], lineColorRGB=[0, 1, 0], lineWidth=5)

    def close(self):
        # Close the PyBullet client
        self.client.disconnect()

    def control_mallets(self):
        # Control mallet1 with W (up) and S (down)
        if keyboard.is_pressed('w'):
            self.apply_action(self.mallet1_id, [5, 0, 0])  # Apply upward force
        elif keyboard.is_pressed('s'):
            self.apply_action(self.mallet1_id, [-5, 0, 0])  # Apply downward force
        
        # Control mallet2 with Up Arrow (up) and Down Arrow (down)
        if keyboard.is_pressed('up'):
            self.apply_action(self.mallet2_id, [5, 0, 0])  # Apply upward force
        elif keyboard.is_pressed('down'):
            self.apply_action(self.mallet2_id, [-5, 0, 0])  # Apply downward force


if __name__ == "__main__":
    env = AirHockeyEnv()  # Initialize the environment
    env.reset()  # Reset the environment

    # Run the simulation for a fixed number of steps
    while True:
        env.step()  # Step the simulation
        
        # Control mallets
        env.control_mallets()

        # Visualize the current state
        env.visualize_state()

    env.close()  # Close the environment