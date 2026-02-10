"""
Double Pendulum Physics Simulator

This simulator loads a controller from controller.py and runs
the double pendulum balancing simulation with matplotlib animation.

To use:
1. Implement your controller in controller.py
2. Run this file: python physics_sim.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import odeint
import double_pendulum.controller as controller

try:
    from double_pendulum.controller import controller
    print("Successfully loaded controller from controller.py")
except ImportError:
    print("ERROR: Could not import controller from controller.py")
    print("Make sure controller.py is in the same directory.")
    exit(1)

# Physical parameters
m1 = 1.0  # mass of pendulum 1 (kg)
m2 = 1.0  # mass of pendulum 2 (kg)
L1 = 1.0  # length of pendulum 1 (m)
L2 = 1.0  # length of pendulum 2 (m)
g = 9.81  # acceleration due to gravity (m/s^2)

# Physical constraints on pivot movement
MAX_PIVOT_VELOCITY = 1.5  # m/s - maximum speed the pivot can move
MAX_PIVOT_ACCEL = 5.0     # m/s^2 - maximum acceleration
PIVOT_LIMIT = 2.0         # m - pivot can only move within [-2, 2]

# Initial conditions: [theta1, omega1, theta2, omega2, x_pivot, v_pivot]
# Starting with moderate perturbation to show balancing
initial_state = [0.1, 0, -0.08, 0, 0, 0]

# Time parameters
dt = 0.01  # time step (s)
t_max = 30  # total simulation time (s)
t = np.arange(0, t_max, dt)


def apply_constraints_to_controller_output(control_signal, state):
    """
    Apply acceleration limits and boundary enforcement to controller output.
    """
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Apply acceleration limits
    control_signal = np.clip(control_signal, -MAX_PIVOT_ACCEL, MAX_PIVOT_ACCEL)
    
    # If approaching position limits, reduce acceleration to slow down
    if x_pivot > PIVOT_LIMIT * 0.9 and control_signal > 0:
        # Approaching right limit, reduce rightward acceleration
        control_signal *= max(0, (PIVOT_LIMIT - x_pivot) / (PIVOT_LIMIT * 0.1))
    elif x_pivot < -PIVOT_LIMIT * 0.9 and control_signal < 0:
        # Approaching left limit, reduce leftward acceleration
        control_signal *= max(0, (PIVOT_LIMIT + x_pivot) / (PIVOT_LIMIT * 0.1))
    
    # If at position limit, force acceleration to reverse direction
    if x_pivot >= PIVOT_LIMIT and v_pivot > 0:
        control_signal = -MAX_PIVOT_ACCEL
    elif x_pivot <= -PIVOT_LIMIT and v_pivot < 0:
        control_signal = MAX_PIVOT_ACCEL
    
    return control_signal


def derivatives(state, t):
    """
    Compute derivatives of the state vector for the double pendulum with movable pivot.
    state = [theta1, omega1, theta2, omega2, x_pivot, v_pivot]
    """
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Get control acceleration from user's controller
    a_pivot = controller(state, t)
    
    # Apply constraints
    a_pivot = apply_constraints_to_controller_output(a_pivot, state)
    
    # Differences in angles
    delta = theta2 - theta1
    
    # Equations of motion with horizontal pivot acceleration
    # These are derived from the Lagrangian with a moving pivot point
    
    den1 = (m1 + m2) * L1 - m2 * L1 * np.cos(delta)**2
    den2 = (L2 / L1) * den1
    
    # Angular accelerations (modified by pivot acceleration)
    dtheta1_dt = omega1
    dtheta2_dt = omega2
    
    domega1_dt = (m2 * L1 * omega1**2 * np.sin(delta) * np.cos(delta) +
                  m2 * g * np.sin(theta2) * np.cos(delta) +
                  m2 * L2 * omega2**2 * np.sin(delta) -
                  (m1 + m2) * g * np.sin(theta1) -
                  (m1 + m2) * a_pivot * np.cos(theta1)) / den1
    
    domega2_dt = (-m2 * L2 * omega2**2 * np.sin(delta) * np.cos(delta) +
                  (m1 + m2) * g * np.sin(theta1) * np.cos(delta) -
                  (m1 + m2) * L1 * omega1**2 * np.sin(delta) -
                  (m1 + m2) * g * np.sin(theta2) -
                  (m1 + m2) * a_pivot * np.cos(theta2)) / den2
    
    # Pivot dynamics
    dx_pivot_dt = v_pivot
    
    # Strong position enforcement at boundaries
    if x_pivot >= PIVOT_LIMIT:
        # At right boundary
        if v_pivot > 0:
            # Moving further right - stop and reverse
            dx_pivot_dt = 0
            dv_pivot_dt = -MAX_PIVOT_ACCEL  # Maximum deceleration
        else:
            # Moving back left - allow it
            dv_pivot_dt = a_pivot
    elif x_pivot <= -PIVOT_LIMIT:
        # At left boundary
        if v_pivot < 0:
            # Moving further left - stop and reverse
            dx_pivot_dt = 0
            dv_pivot_dt = MAX_PIVOT_ACCEL  # Maximum acceleration
        else:
            # Moving back right - allow it
            dv_pivot_dt = a_pivot
    else:
        # Not at boundary - apply velocity limiting
        if abs(v_pivot) > MAX_PIVOT_VELOCITY:
            # At or above max velocity
            if (v_pivot > 0 and a_pivot > 0) or (v_pivot < 0 and a_pivot < 0):
                dv_pivot_dt = a_pivot * 0.1  # Greatly reduce acceleration in velocity direction
            else:
                dv_pivot_dt = a_pivot  # Allow deceleration
        else:
            dv_pivot_dt = a_pivot
    
    return [dtheta1_dt, domega1_dt, dtheta2_dt, domega2_dt, dx_pivot_dt, dv_pivot_dt]


# Solve the differential equations
print("Computing double pendulum motion with your controller...")
solution = odeint(derivatives, initial_state, t)

# Extract angles and pivot position
theta1 = solution[:, 0]
theta2 = solution[:, 2]
x_pivot = solution[:, 4]

# Convert to Cartesian coordinates (relative to moving pivot)
x1 = x_pivot + L1 * np.sin(theta1)
y1 = -L1 * np.cos(theta1)
x2 = x1 + L2 * np.sin(theta2)
y2 = y1 - L2 * np.cos(theta2)

# Calculate average height of the end of the pendulum
avg_height = np.mean(y2)
print(f"Average height of pendulum end: {avg_height:.3f} m")
print(f"Target height (fully vertical): {-(L1 + L2):.3f} m")
print(f"Performance: {100 * avg_height / -(L1 + L2):.1f}% of target")

# Set up the figure and axis
fig, ax = plt.subplots(figsize=(12, 10))
ax.set_xlim(-3, 3)
ax.set_ylim(-2.2, 2.2)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title('Balanced Double Pendulum with Moving Pivot (Constrained)')

# Add a horizontal line at y=0 to show the pivot level
ax.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)

# Add vertical lines to show pivot boundaries (central 2/3)
ax.axvline(x=-2.0, color='red', linestyle='--', linewidth=2, alpha=0.7, label='Pivot Limits')
ax.axvline(x=2.0, color='red', linestyle='--', linewidth=2, alpha=0.7)

# Add shaded region to show allowed pivot area
ax.axvspan(-2.0, 2.0, alpha=0.1, color='green', label='Allowed Pivot Zone')

ax.legend(loc='upper right', fontsize=10)

# Initialize plot elements
line, = ax.plot([], [], 'o-', lw=2, color='blue', markersize=8)
trace, = ax.plot([], [], '-', lw=1, color='red', alpha=0.3)
pivot_line, = ax.plot([], [], 'v', markersize=12, color='black')  # Pivot marker
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
stats_text = ax.text(0.02, 0.88, '', transform=ax.transAxes, fontsize=10)

plt.tight_layout()

# Storage for trace
trace_x, trace_y = [], []
trace_length = 500  # number of points to keep in trace

# Storage for tracking average height
height_history = []


def init():
    """Initialize animation"""
    line.set_data([], [])
    trace.set_data([], [])
    pivot_line.set_data([], [])
    time_text.set_text('')
    stats_text.set_text('')
    return line, trace, pivot_line, time_text, stats_text


def animate(frame):
    """Animation function called for each frame"""
    # Get current positions (including moving pivot)
    thisx = [x_pivot[frame], x1[frame], x2[frame]]
    thisy = [0, y1[frame], y2[frame]]
    
    # Update pendulum line
    line.set_data(thisx, thisy)
    
    # Update pivot marker
    pivot_line.set_data([x_pivot[frame]], [0])
    
    # Update trace (path of second pendulum)
    trace_x.append(x2[frame])
    trace_y.append(y2[frame])
    
    # Keep only recent trace points
    if len(trace_x) > trace_length:
        trace_x.pop(0)
        trace_y.pop(0)
    
    trace.set_data(trace_x, trace_y)
    
    # Track height
    height_history.append(y2[frame])
    
    # Calculate running average height
    if len(height_history) > 0:
        running_avg_height = np.mean(height_history)
    else:
        running_avg_height = 0
    
    # Update time text
    time_text.set_text(f'Time: {t[frame]:.2f} s')
    
    # Get current velocity
    v_pivot_current = solution[frame, 5]
    
    # Check constraint violations
    pos_violation = "⚠" if abs(x_pivot[frame]) > PIVOT_LIMIT else "✓"
    vel_violation = "⚠" if abs(v_pivot_current) > MAX_PIVOT_VELOCITY else "✓"
    
    # Update statistics
    angle1_deg = np.degrees(theta1[frame])
    angle2_deg = np.degrees(theta2[frame])
    stats_text.set_text(
        f'Pivot X: {x_pivot[frame]:+.2f} m {pos_violation}\n'
        f'Pivot V: {v_pivot_current:+.2f} m/s {vel_violation}\n'
        f'θ₁: {angle1_deg:+.1f}°  θ₂: {angle2_deg:+.1f}°\n'
        f'Avg Height: {running_avg_height:.3f} m'
    )
    
    return line, trace, pivot_line, time_text, stats_text


# Create animation
print("Creating animation...")
anim = FuncAnimation(fig, animate, init_func=init, frames=len(t),
                     interval=dt*1000, blit=False, repeat=True)

print("Displaying animation. Close the window to exit.")
plt.show()