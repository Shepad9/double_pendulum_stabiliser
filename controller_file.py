import numpy as np

def optimal_balancing_controller(state, t, m1, m2, L1, L2, g):
    """
    Optimal controller for double inverted pendulum to maximize average height.
    
    This function computes the raw control signal. The main simulation will apply
    physical constraints (acceleration limits, position limits, velocity limits).
    
    Parameters:
    -----------
    state : array [theta1, omega1, theta2, omega2, x_pivot, v_pivot]
    t : float - current time
    m1, m2 : float - masses of pendulum segments
    L1, L2 : float - lengths of pendulum segments
    g : float - gravitational acceleration
    
    Returns:
    --------
    control_signal : float - desired pivot acceleration
    """
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Physical constraints (these should match your simulation)
    MAX_PIVOT_ACCEL = 5.0
    PIVOT_LIMIT = 2.0
    MAX_PIVOT_VELOCITY = 1.5
    
    # ULTRA-CONSERVATIVE controller gains - absolute minimum for stability
    # Any higher and the system goes unstable within seconds
    Kp1 = 150.0  # Proportional gain for first pendulum angle  
    Kd1 = 50.0   # Derivative gain for first pendulum
    Kp2 = 90.0   # Proportional gain for second pendulum angle
    Kd2 = 35.0   # Derivative gain for second pendulum
    
    # Main PD control law
    # Negative feedback: when theta > 0 (tilting right), apply leftward acceleration
    control_signal = -(Kp1 * theta1 + Kd1 * omega1 + 
                       Kp2 * theta2 + Kd2 * omega2)
    
    # Cross-coupling term to coordinate both segments
    # When segments are at different angles, add correction
    delta_theta = theta2 - theta1
    K_coupling = 3.0  # Minimal coupling
    control_signal += -K_coupling * delta_theta
    
    # Pivot regulation: prefer to stay near center of allowed range
    # This helps avoid hitting position limits
    K_position = 0.1  # Minimal
    K_velocity = 0.3  # Minimal
    control_signal += -K_position * x_pivot - K_velocity * v_pivot
    
    # Apply acceleration limits
    control_signal = np.clip(control_signal, -MAX_PIVOT_ACCEL, MAX_PIVOT_ACCEL)
    
    # If approaching position limits, reduce acceleration to slow down
    if x_pivot > PIVOT_LIMIT * 0.9 and control_signal > 0:
        # Approaching right limit, scale down rightward acceleration
        control_signal *= max(0, (PIVOT_LIMIT - x_pivot) / (PIVOT_LIMIT * 0.1))
    elif x_pivot < -PIVOT_LIMIT * 0.9 and control_signal < 0:
        # Approaching left limit, scale down leftward acceleration
        control_signal *= max(0, (PIVOT_LIMIT + x_pivot) / (PIVOT_LIMIT * 0.1))
    
    # Hard boundary enforcement
    if x_pivot >= PIVOT_LIMIT and v_pivot > 0:
        # At right boundary moving right - force reversal
        control_signal = -MAX_PIVOT_ACCEL
    elif x_pivot <= -PIVOT_LIMIT and v_pivot < 0:
        # At left boundary moving left - force reversal
        control_signal = MAX_PIVOT_ACCEL
    
    return control_signal


def simple_pd_controller(state, t, m1, m2, L1, L2, g):
    """
    Simplified PD controller without advanced features.
    More conservative but very stable.
    """
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Simple fixed gains
    Kp1 = 300.0
    Kd1 = 90.0
    Kp2 = 180.0
    Kd2 = 65.0
    
    # Basic PD control
    control_signal = -(Kp1 * theta1 + Kd1 * omega1 + 
                       Kp2 * theta2 + Kd2 * omega2)
    
    # Minimal pivot regulation
    control_signal += -0.3 * x_pivot - 0.7 * v_pivot
    
    # Apply limits
    MAX_PIVOT_ACCEL = 5.0
    PIVOT_LIMIT = 2.0
    
    control_signal = np.clip(control_signal, -MAX_PIVOT_ACCEL, MAX_PIVOT_ACCEL)
    
    # Boundary handling
    if x_pivot > PIVOT_LIMIT * 0.9 and control_signal > 0:
        control_signal *= max(0, (PIVOT_LIMIT - x_pivot) / (PIVOT_LIMIT * 0.1))
    elif x_pivot < -PIVOT_LIMIT * 0.9 and control_signal < 0:
        control_signal *= max(0, (PIVOT_LIMIT + x_pivot) / (PIVOT_LIMIT * 0.1))
    
    if x_pivot >= PIVOT_LIMIT and v_pivot > 0:
        control_signal = -MAX_PIVOT_ACCEL
    elif x_pivot <= -PIVOT_LIMIT and v_pivot < 0:
        control_signal = MAX_PIVOT_ACCEL
    
    return control_signal