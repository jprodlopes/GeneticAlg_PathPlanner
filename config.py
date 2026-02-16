"""
Global configuration and constants for the Genetic Algorithm Path Planner
"""

import numpy as np

# Flight and Vehicle Parameters
FLIGHT_SPEED = 5            # Typical flight speeds: 4 m/s < Vflight < 8 m/s
B_MIN = 0.448               # Minimum morphing wing span (meters)
B_MAX = 0.667               # Maximum morphing wing span (meters)


V_MAX = 10                  # Maximum velocity (m/s)

# Physics Constants
ALPHA_MAX = 0.244346        # Maximum angle of attack
RHO = 1.225                 # Air density (kg/m^3)
CD0 = 0.05                  # Zero-lift drag coefficient
M = 0.12                    # Aircraft mass (kg)
G = 9.81                    # Gravitational acceleration (m/s^2)

# Control Parameters
TMAX = 1                    # Maximum thrust
GAMMA_ACT = 1               # Actual flight path angle
R_MIN_THRESHOLD = 1.4       # Minimum radius threshold

# Derived constants
CL = 3.683 * ALPHA_MAX
WING_SURFACE_MIN = 0.0531 + 0.0012 * np.exp(4.6945 * B_MIN)
WING_SURFACE_MAX = 0.0531 + 0.0012 * np.exp(4.6945 * B_MAX)

# Algorithm Parameters
VARIABLE_SPEED = False      # Whether to use variable speed optimization
