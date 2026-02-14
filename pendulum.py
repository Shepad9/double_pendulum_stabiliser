import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import odeint

import controller_file
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
# Two slightly different initial conditions to show sensitivity
initial_state_1 = [1.01, 1.2, -0.012, 1.1, 1, 0]  # First pendulum
initial_state_2 = [1, 1.2, -0.011, 1.1, 1, 0]  # Second pendulum (slightly different)

# Time parameters
dt = 0.05  # smaller time step for better accuracy (s)
t_max = 100  # total simulation time (s)
t = np.arange(0, t_max, dt)

def controller(state, t):
    control_signal = -5 * state[4]
    
    return control_signal

def derivatives(state, t):
    """
    Compute derivatives of the state vector for the double pendulum with movable pivot.
    """
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Get control acceleration
    a_pivot = controller(state, t)
    
    # Differences in angles
    delta = theta2 - theta1
    
    # Equations of motion
    den1 = (m1 + m2) * L1 - m2 * L1 * np.cos(delta)**2
    den2 = (L2 / L1) * den1
    
    # Angular accelerations
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
    
    # Pivot dynamics with velocity limiting
    dx_pivot_dt = v_pivot
    
    if x_pivot >= PIVOT_LIMIT:
        if v_pivot > 0:
            dx_pivot_dt = 0
            dv_pivot_dt = -MAX_PIVOT_ACCEL
        else:
            dv_pivot_dt = a_pivot
    elif x_pivot <= -PIVOT_LIMIT:
        if v_pivot < 0:
            dx_pivot_dt = 0
            dv_pivot_dt = MAX_PIVOT_ACCEL
        else:
            dv_pivot_dt = a_pivot
    else:
        if abs(v_pivot) > MAX_PIVOT_VELOCITY:
            if (v_pivot > 0 and a_pivot > 0) or (v_pivot < 0 and a_pivot < 0):
                dv_pivot_dt = a_pivot * 0.1
            else:
                dv_pivot_dt = a_pivot
        else:
            dv_pivot_dt = a_pivot
    
    return [dtheta1_dt, domega1_dt, dtheta2_dt, domega2_dt, dx_pivot_dt, dv_pivot_dt]

# Solve for both pendulums
print("Computing motion for pendulum 1...")
solution_1 = odeint(derivatives, initial_state_1, t)

print("Computing motion for pendulum 2...")
solution_2 = odeint(derivatives, initial_state_2, t)

# Extract positions for pendulum 1
theta1_1 = solution_1[:, 0]
theta2_1 = solution_1[:, 2]
x_pivot_1 = solution_1[:, 4]

x1_1 = x_pivot_1 + L1 * np.sin(theta1_1)
y1_1 = -L1 * np.cos(theta1_1)
x2_1 = x1_1 + L2 * np.sin(theta2_1)
y2_1 = y1_1 - L2 * np.cos(theta2_1)

avg_height_1 = np.mean(y2_1)
print(f"Pendulum 1 - Average height: {avg_height_1:.3f} m")

# Extract positions for pendulum 2
theta1_2 = solution_2[:, 0]
theta2_2 = solution_2[:, 2]
x_pivot_2 = solution_2[:, 4]

x1_2 = x_pivot_2 + L1 * np.sin(theta1_2)
y1_2 = -L1 * np.cos(theta1_2)
x2_2 = x1_2 + L2 * np.sin(theta2_2)
y2_2 = y1_2 - L2 * np.cos(theta2_2)

avg_height_2 = np.mean(y2_2)
print(f"Pendulum 2 - Average height: {avg_height_2:.3f} m")

# Set up the figure
fig, ax = plt.subplots(figsize=(12, 10))
ax.set_xlim(-3, 3)
ax.set_ylim(-2.2, 2.2)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title('Two Double Pendulums with Slightly Different Initial Conditions')

# Reference lines
ax.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
ax.axvline(x=-2.0, color='red', linestyle='--', linewidth=2, alpha=0.7, label='Pivot Limits')
ax.axvline(x=2.0, color='red', linestyle='--', linewidth=2, alpha=0.7)
ax.axvspan(-2.0, 2.0, alpha=0.1, color='green')

# Plot elements for pendulum 1 (blue)
line1, = ax.plot([], [], 'o-', lw=2, color='blue', markersize=8, label='Pendulum 1', alpha=0.7)
trace1, = ax.plot([], [], '-', lw=1, color='blue', alpha=0.3)
pivot1, = ax.plot([], [], 'v', markersize=12, color='darkblue', alpha=0.7)

# Plot elements for pendulum 2 (orange)
line2, = ax.plot([], [], 'o-', lw=2, color='orange', markersize=8, label='Pendulum 2', alpha=0.7)
trace2, = ax.plot([], [], '-', lw=1, color='orange', alpha=0.3)
pivot2, = ax.plot([], [], 'v', markersize=12, color='darkorange', alpha=0.7)

time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
stats_text = ax.text(0.02, 0.82, '', transform=ax.transAxes, fontsize=9)

ax.legend(loc='upper right', fontsize=10)
plt.tight_layout()

# Storage for traces
trace_x1, trace_y1 = [], []
trace_x2, trace_y2 = [], []
trace_length = 500

def init():
    """Initialize animation"""
    line1.set_data([], [])
    trace1.set_data([], [])
    pivot1.set_data([], [])
    line2.set_data([], [])
    trace2.set_data([], [])
    pivot2.set_data([], [])
    time_text.set_text('')
    stats_text.set_text('')
    return line1, trace1, pivot1, line2, trace2, pivot2, time_text, stats_text

def animate(frame):
    """Animation function"""
    # Pendulum 1 (blue)
    thisx1 = [x_pivot_1[frame], x1_1[frame], x2_1[frame]]
    thisy1 = [0, y1_1[frame], y2_1[frame]]
    line1.set_data(thisx1, thisy1)
    pivot1.set_data([x_pivot_1[frame]], [0])
    
    trace_x1.append(x2_1[frame])
    trace_y1.append(y2_1[frame])
    if len(trace_x1) > trace_length:
        trace_x1.pop(0)
        trace_y1.pop(0)
    trace1.set_data(trace_x1, trace_y1)
    
    # Pendulum 2 (orange)
    thisx2 = [x_pivot_2[frame], x1_2[frame], x2_2[frame]]
    thisy2 = [0, y1_2[frame], y2_2[frame]]
    line2.set_data(thisx2, thisy2)
    pivot2.set_data([x_pivot_2[frame]], [0])
    
    trace_x2.append(x2_2[frame])
    trace_y2.append(y2_2[frame])
    if len(trace_x2) > trace_length:
        trace_x2.pop(0)
        trace_y2.pop(0)
    trace2.set_data(trace_x2, trace_y2)
    
    # Update text
    time_text.set_text(f'Time: {t[frame]:.2f} s')
    
    # Calculate divergence
    position_diff = np.sqrt((x2_1[frame] - x2_2[frame])**2 + (y2_1[frame] - y2_2[frame])**2)
    angle_diff_1 = np.degrees(abs(theta1_1[frame] - theta1_2[frame]))
    angle_diff_2 = np.degrees(abs(theta2_1[frame] - theta2_2[frame]))
    
    stats_text.set_text(
        f'Pendulum 1: θ₁={np.degrees(theta1_1[frame]):+.1f}° θ₂={np.degrees(theta2_1[frame]):+.1f}°\n'
        f'Pendulum 2: θ₁={np.degrees(theta1_2[frame]):+.1f}° θ₂={np.degrees(theta2_2[frame]):+.1f}°\n'
        f'Position difference: {position_diff:.3f} m\n'
        f'Angle differences: Δθ₁={angle_diff_1:.1f}° Δθ₂={angle_diff_2:.1f}°'
    )
    
    return line1, trace1, pivot1, line2, trace2, pivot2, time_text, stats_text

# Create animation
print("Creating animation...")
anim = FuncAnimation(fig, animate, init_func=init, frames=len(t),
                     interval=dt*1000, blit=False, repeat=True)

print("Displaying animation. Close the window to exit.")
plt.show()