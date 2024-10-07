import gymnasium as gym
import numpy as np

# Create the environment
env = gym.make("CartPole-v1", render_mode="rgb_array_list")
observation, info = env.reset()

def print_observation_space(env):
    print(f"Observation space high: {env.observation_space.high}")
    print(f"Observation space low: {env.observation_space.low}")
    print(f"Number of actions in the action space: {env.action_space.n}")

# Define the size of the discrete observation space and the real observation space we care about
DISCRETE_OS_SIZE = [25, 25]  # Discretization into a 25x25 grid
real_observation_space = np.array([env.observation_space.high[2], 3.5])  # Focusing on pole angle and velocity

# Compute the step size inside the discrete observation space
discrete_os_win_size = real_observation_space * 2 / (DISCRETE_OS_SIZE - np.array([1, 1]))

# Convert continuous state to discrete state
def get_discrete_state(state):
    trimmed_state = np.array([state[2], state[3]])  # We care only about pole angle and velocity
    discrete_state = (trimmed_state + real_observation_space) / discrete_os_win_size
    return tuple(discrete_state.astype(np.int32))

# Initialize Q-table
q_table = np.random.uniform(low=0, high=1, size=(DISCRETE_OS_SIZE + [env.action_space.n]))

# Hyperparameters
LEARNING_RATE = 0.1
DISCOUNT = 0.95
EPISODES = 12000
LOG_FREQUENCY = 2000
epsilon = 0.1
START_DECAY = 1
END_DECAY = EPISODES // 2
epsilon_decay_by = epsilon / (END_DECAY - START_DECAY)

# Main training loop
for episode in range(0,EPISODES,1):
    # Just some logging info
    if episode % LOG_FREQUENCY == 0:
        render = True
        print(f"Episode {episode}")

    else:
        render = False

    # Resetting the environment and getting initial state
    observation, info = env.reset()
    discrete_state = get_discrete_state(observation)
    terminated = False
    truncated = False

    # One iteration of the environment
    while not terminated and not truncated:

        # Epsilon-greedy action selection
        if np.random.random() > epsilon:
            action = np.argmax(q_table[discrete_state])
        else:
            action = np.random.randint(0, env.action_space.n)

        # Step through the environment
        new_state, reward, terminated, truncated, info = env.step(action)
        new_discrete_state = get_discrete_state(new_state)

        if render:
            env.render()

        # Update Q-table using Q-learning formula
        if not terminated and not truncated:
            max_future_q = np.max(q_table[new_discrete_state])
            current_q = q_table[discrete_state + (action,)]
            new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)
            q_table[discrete_state + (action,)] = new_q

        # Move to the next state
        discrete_state = new_discrete_state

    # Decay epsilon
    if END_DECAY >= episode >= START_DECAY:
        epsilon -= epsilon_decay_by
        epsilon = max(epsilon, 0)  # Ensure epsilon does not go negative

# Helper function to get max velocities of the cart and the pole
def get_max_velocity(env):
    max_velo_cart = 0
    max_velo_pole = 0
    observation, info = env.reset()
    terminated = False
    truncated = False
    while not terminated and not truncated:
        new_state, _, terminated, truncated, _ = env.step(env.action_space.sample())  # Random action for exploration
        max_velo_cart = max(max_velo_cart, abs(new_state[1]))
        max_velo_pole = max(max_velo_pole, abs(new_state[3]))
        env.render()

    print(f"Max_velo_cart={max_velo_cart}")
    print(f"Max_velo_pole={max_velo_pole}")

# Example usage of the helper function
get_max_velocity(env)
