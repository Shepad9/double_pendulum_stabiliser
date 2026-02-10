# Double Pendulum Controller API

A clean interface for implementing and testing controllers for the inverted double pendulum balancing problem.

## Files

- **`controller_template.py`** - Implement your controller here
- **`physics_sim.py`** - Physics simulator that runs your controller

## Quick Start

1. Edit `controller_template.py` and implement the `controller()` function
2. Run the simulator: `python physics_sim.py`
3. Watch your controller balance the pendulum!

## The Problem

**Objective**: Keep a double pendulum balanced upright (vertical) by moving the pivot point horizontally.

**State Vector** (what you observe):
```python
state = [theta1, omega1, theta2, omega2, x_pivot, v_pivot]
```

- `theta1, theta2`: Angles from vertical (radians), 0 = straight up
- `omega1, omega2`: Angular velocities (rad/s)
- `x_pivot, v_pivot`: Pivot position (m) and velocity (m/s)

**Control Output**: Return desired pivot acceleration (m/s²)
- Positive = accelerate right
- Negative = accelerate left

**Constraints** (automatically enforced):
- Position: -2.0 m ≤ x_pivot ≤ 2.0 m
- Velocity: -1.5 m/s ≤ v_pivot ≤ 1.5 m/s  
- Acceleration: -5.0 m/s² ≤ a_pivot ≤ 5.0 m/s²

**Goal**: Maximize average height of pendulum's end point
- Target: -2.0 m (perfectly vertical)
- Your score = average height / -2.0 m

## Example: Simple PD Controller

```python
def controller(state, t):
    theta1, omega1, theta2, omega2, x_pivot, v_pivot = state
    
    # Proportional-Derivative gains
    Kp1 = 400.0  # Proportional gain for angle 1
    Kd1 = 120.0  # Derivative gain for angle 1
    Kp2 = 220.0  # Proportional gain for angle 2
    Kd2 = 80.0   # Derivative gain for angle 2
    
    # Control law: drive angles and velocities to zero
    control_signal = -(Kp1 * theta1 + Kd1 * omega1 + 
                      Kp2 * theta2 + Kd2 * omega2)
    
    return control_signal
```

This achieves ~80% of target height.

## Advanced Controllers

See `controller_template.py` for more examples:
- **`pd_controller`**: Basic proportional-derivative control
- **`lqr_controller`**: Linear quadratic regulator approach
- **`adaptive_controller`**: State-dependent gains with boundary avoidance

## Tips for Better Performance

1. **Tune gains**: Adjust Kp and Kd values for each pendulum
2. **Add position feedback**: Prevent hitting boundaries by considering x_pivot
3. **State-dependent gains**: Increase gains when far from vertical
4. **Velocity feedback**: Use v_pivot to anticipate boundary collisions
5. **Nonlinear control**: Consider sin(theta) vs theta for large angles

## Physical Parameters

```python
m1 = 1.0 kg     # mass of first pendulum
m2 = 1.0 kg     # mass of second pendulum  
L1 = 1.0 m      # length of first pendulum
L2 = 1.0 m      # length of second pendulum
g = 9.81 m/s²   # gravity
```

## Visualization

The simulator shows:
- **Blue line**: The two pendulum segments
- **Black triangle**: Moving pivot point
- **Red trace**: Path of pendulum's end
- **Green zone**: Allowed pivot area (±2m)
- **Red lines**: Position boundaries
- **Stats display**: 
  - Pivot position and velocity (with ✓/⚠ indicators)
  - Pendulum angles
  - Running average height

## Requirements

```bash
pip install numpy scipy matplotlib
```

## Troubleshooting

**Problem**: Pendulum falls over immediately  
**Solution**: Increase proportional gains (Kp1, Kp2)

**Problem**: Oscillates wildly  
**Solution**: Increase derivative gains (Kd1, Kd2)

**Problem**: Hits boundaries repeatedly  
**Solution**: Add position feedback term: `-K_pos * x_pivot`

**Problem**: Simulation becomes unstable (NaN values)  
**Solution**: Reduce gains or initial perturbation in physics_sim.py

## Challenge Ideas

1. Achieve >90% of target height
2. Minimize pivot movement while balancing
3. Swing up from hanging position (theta ≈ π)
4. Balance with time-varying masses or lengths
5. Add robustness to random disturbances

Good luck!