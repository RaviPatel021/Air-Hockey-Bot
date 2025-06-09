import numpy as np

dimension_scale = 1

# Enhanced Physics Constants
PHYSICS_CONFIG = {
    "gravity": -9.8,  # Gravity should remain unchanged, though not very relevant in a 2D air hockey simulation
    
    # Walls
    "wall_restitution": 0.9,  # Walls should be nearly elastic but absorb a tiny bit of energy
    
    # Puck Properties
    "puck_restitution": 0.95,  # Some energy loss in collisions to prevent infinite motion
    "puck_linear_damping": 0.01,  # Small air resistance, but keeps the puck moving for a while
    "puck_angular_damping": 0.02,  # Slight damping for spinning motion
    "puck_lateral_friction": 0.02,  # Minimal friction to prevent infinite sliding
    "puck_rolling_friction": 0.0,  # Not relevant since the puck doesn't roll
    "puck_spinning_friction": 0.01,  # Small friction so spin gradually slows

    # Paddle Properties
    "paddle_restitution": 0.9,  # Slightly inelastic to prevent infinite speed growth
    "paddle_linear_damping": 0.02,  # Small damping to match human movement behavior
    "paddle_angular_damping": 0.05,  # Slight angular damping to prevent infinite rotation
    "paddle_lateral_friction": 0.1,  # Some friction to allow controlled movement
    "paddle_rolling_friction": 0.0,  # Not relevant
    "paddle_spinning_friction": 0.02  # Slight spin friction to reduce unnatural effects
}


# Optimized Table Dimensions (cm)
TABLE_CONFIG = {
    "length": 1.3462 * dimension_scale,
    "width": 0.660 * dimension_scale,
    "wall_height": 0.03 * dimension_scale,
    "wall_thickness": 0.05 * dimension_scale,
    "goal_width": 0.169 * dimension_scale,
    "paddle_zone": 0.4132 * dimension_scale,
    "corner": 0.07 * dimension_scale,
    "buffer" : 0.01 * dimension_scale
}

# Paddle and Puck Specs
OBJECT_CONFIG = {
    "puck": {
        "diameter": 0.0581 * dimension_scale,
        "height": 0.01 * dimension_scale,
        "mass": 0.5,
        "max_velocity": 6.00 * dimension_scale,     # cm/s
    },
    "paddle": {
        "diameter": 0.0753 * dimension_scale,
        "height": 0.020 * dimension_scale,
        "mass": 2.5,
        "max_velocity": 1.00 * dimension_scale,     # cm/s
        "max_accel": 5.00 * dimension_scale
    }
}

velocity_sequence_length = 5
# Define crop boundaries
x_start, x_end = 17, 620
y_start, y_end = 68, 366

x_start_paddle, x_end_paddle = 0, 195
y_start_paddle, y_end_paddle = 60, 370

puck_lower_bound = np.array([150, 80, 170])
puck_upper_bound = np.array([179, 255, 255])

paddle_lower_bound = np.array([15, 130, 90])
paddle_upper_bound = np.array([28, 255, 255])

# paddle_lower_bound = np.array([130, 40, 60])
# paddle_upper_bound = np.array([155, 255, 255])

orange_paddle_lower_bound = np.array([5, 100, 160])
orange_paddle_upper_bound = np.array([10, 255, 255])

paddle_x_left = 18
paddle_x_right = 289

real_left = OBJECT_CONFIG["paddle"]["diameter"]/2
real_right = TABLE_CONFIG["width"] - OBJECT_CONFIG["paddle"]["diameter"]/2

paddle_y_up = 13
paddle_y_down = 167

real_up = TABLE_CONFIG["paddle_zone"] - OBJECT_CONFIG["paddle"]["diameter"]/2
real_down = OBJECT_CONFIG["paddle"]["diameter"]/2

m_x_paddle = (real_right - real_left) / (paddle_x_right - paddle_x_left)
b_x_paddle = real_left - m_x_paddle * paddle_x_left

m_y_paddle = (real_down - real_up) / (paddle_y_down - paddle_y_up)
b_y_paddle = real_up - m_y_paddle * paddle_y_up


puck_x_left = 12
puck_x_right = 284

puck_real_left = OBJECT_CONFIG["puck"]["diameter"]/2
puck_real_right = TABLE_CONFIG["width"] - OBJECT_CONFIG["puck"]["diameter"]/2

puck_y_up = 10
puck_y_down = 591

puck_real_up = TABLE_CONFIG["length"] - OBJECT_CONFIG["puck"]["diameter"]/2
puck_real_down = OBJECT_CONFIG["puck"]["diameter"]/2

m_x_puck = (puck_real_right - puck_real_left) / (puck_x_right - puck_x_left)
b_x_puck = puck_real_left - m_x_puck * puck_x_left

m_y_puck = (puck_real_down - puck_real_up) / (puck_y_down - puck_y_up)
b_y_puck = puck_real_up - m_y_puck * puck_y_up

bounces_allowed = 2



FPS = 100
pixel_cm = TABLE_CONFIG["width"]/(y_end-y_start)
cm_pixel = (y_end-y_start)/TABLE_CONFIG["width"]
simulation_time = 2