"""
Double Pendulum Controller Template

This template provides a clean API for implementing your own controller
for the inverted double pendulum balancing problem.

OBJECTIVE: Keep the double pendulum balanced upright (vertical) by moving
the pivot point horizontally.

STATE VECTOR (what you observe):
    state = [theta1, omega1, theta2, omega2, x_pivot, v_pivot]
    
    theta1   : angle of first pendulum from vertical (radians)
               0 = straight up, positive = clockwise
    omega1   : angular velocity of first pendulum (rad/s)
    
    theta2   : angle of second pendulum from vertical (radians)
               0 = straight up, positive = clockwise
    omega2   : angular velocity of second pendulum (rad/s)
    
    x_pivot  : horizontal position of pivot (meters)
    v_pivot  : horizontal velocity of pivot (m/s)
    
    t        : current time (seconds)

CONTROL OUTPUT:
    Return the desired pivot acceleration (m/s²)
    Positive = accelerate right, Negative = accelerate left

CONSTRAINTS (automatically enforced by simulator):
    Position: -2.0 m ≤ x_pivot ≤ 2.0 m
    Velocity: -1.5 m/s ≤ v_pivot ≤ 1.5 m/s
    Acceleration: -5.0 m/s² ≤ a_pivot ≤ 5.0 m/s²

PHYSICAL PARAMETERS:
    m1 = 1.0 kg    # mass of first pendulum
    m2 = 1.0 kg    # mass of second pendulum
    L1 = 1.0 m     # length of first pendulum
    L2 = 1.0 m     # length of second pendulum
    g = 9.81 m/s²  # gravity

GOAL: Maximize average height of the pendulum's end point
      (Target: -2.0 m when perfectly vertical)
"""

import numpy as np

# Physical parameters (read-only)
m1 = 1.0
m2 = 1.0
L1 = 1.0
L2 = 1.0
g = 9.81

# Constraint limits (read-only)
MAX_PIVOT_VELOCITY = 1.5
MAX_PIVOT_ACCEL = 5.0
PIVOT_LIMIT = 2.0


def controller(state, t):
    """
    Implement your controller here.
    
    Parameters
    ----------
    state : list or array
        [theta1, omega1, theta2, omega2, x_pivot, v_pivot]
    t : float
        Current simulation time in seconds
    
    Returns
    -------
    float
        Desired pivot acceleration in m/s²
        Will be automatically clipped to [-5.0, 5.0]
    
    Example Implementation (PD Controller)
    --------------------------------------
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Proportional-Derivative gains
    Kp1 = 400.0  # Proportional gain for angle 1
    Kd1 = 120.0  # Derivative gain for angle 1
    Kp2 = 220.0  # Proportional gain for angle 2
    Kd2 = 80.0   # Derivative gain for angle 2
    
    # PD control law: drive angles and angular velocities to zero
    control_signal = -(Kp1 * theta1 + Kd1 * omega1 + 
                      Kp2 * theta2 + Kd2 * omega2)
    
    return control_signal
    """
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # TODO: Implement your controller here
    # This is a working PD controller as a starting point
    Kp1 = 400.0
    Kd1 = 120.0
    Kp2 = 220.0
    Kd2 = 80.0
    
    control_signal = -(Kp1 * theta1 + Kd1 * omega1 + 
                      Kp2 * theta2 + Kd2 * omega2)
    
    return control_signal


# Example: Simple PD controller
def pd_controller(state, t):
    """Example PD (Proportional-Derivative) controller."""
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Tuned gains
    Kp1 = 400.0
    Kd1 = 120.0
    Kp2 = 220.0
    Kd2 = 80.0
    
    # Control law
    control_signal = -(Kp1 * theta1 + Kd1 * omega1 + 
                      Kp2 * theta2 + Kd2 * omega2)
    
    return control_signal


# Example: LQR-inspired controller
def lqr_controller(state, t):
    """Example Linear Quadratic Regulator inspired controller."""
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # LQR gains (would normally be computed from Riccati equation)
    K = np.array([450.0, 130.0, 250.0, 90.0, 50.0, 30.0])
    
    # State feedback control
    state_vector = np.array([theta1, omega1, theta2, omega2, x_pivot, v_pivot])
    control_signal = -np.dot(K, state_vector)
    
    return control_signal


# Example: State-dependent controller
def adaptive_controller(state, t):
    """Example controller that adapts based on system state."""
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Base gains
    Kp1 = 400.0
    Kd1 = 120.0
    Kp2 = 220.0
    Kd2 = 80.0
    
    # Increase gains when far from vertical
    angle_magnitude = abs(theta1) + abs(theta2)
    gain_multiplier = 1.0 + 0.5 * angle_magnitude
    
    control_signal = -gain_multiplier * (Kp1 * theta1 + Kd1 * omega1 + 
                                         Kp2 * theta2 + Kd2 * omega2)
    
    # Add position feedback to avoid hitting boundaries
    if abs(x_pivot) > PIVOT_LIMIT * 0.7:
        control_signal -= 30.0 * x_pivot
    
    return control_signal


if __name__ == "__main__":
    print("Controller Template for Double Pendulum Balancing")
    print("=" * 60)
    print("\nTo use this controller:")
    print("1. Implement the controller() function above")
    print("2. Replace the controller import in double_pendulum.py:")
    print("   from controller import controller")
    print("\nExample controllers provided:")
    print("- pd_controller: Simple proportional-derivative control")
    print("- lqr_controller: Linear quadratic regulator approach")
    print("- adaptive_controller: State-dependent gains")
    print("\nTest your controller by running double_pendulum.py")