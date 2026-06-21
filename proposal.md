# AutoGlide Controller + Thermal Estimator Implementation Proposal

## 0. Goal

Build a simplified autonomous thermal soaring simulation inspired by `sahil-kale/autoglide`.

The system should simulate a glider that:

1. Flies forward in cruise mode.
2. Uses vertical speed / variometer measurements to detect possible thermal lift.
3. Estimates the thermal core position, strength, and radius from recent flight data.
4. Switches between `Cruise`, `Probe`, and `Thermal` modes.
5. In thermal mode, circles around the estimated thermal core.
6. Chooses a reasonable circling radius and bank angle to maximize net climb.

This proposal should be implemented step by step. Start simple first. Do not over-engineer the first version.

---

## 1. Core Concept

The main physical relationship is:

```text
net climb = thermal lift - glider sink rate
```

In formula form:

```text
h_dot = w - v_s
```

Where:

- `h_dot` is the glider's vertical speed / net climb rate.
- `w` is the vertical air velocity from the thermal updraft.
- `v_s` is the glider sink rate.

The variometer measures approximately `h_dot`. To estimate the thermal lift itself, use:

```text
w_meas = h_dot + v_s
```

The thermal estimator should use `(x, y, w_meas)` data points to fit a simplified thermal model.

---

## 2. Project Structure

Create or update the project using this structure:

```text
autoglide_simplified/
  README.md
  proposal.md
  requirements.txt
  main.py
  config.py

  glider_model/
    __init__.py
    glider.py

  thermal_model/
    __init__.py
    thermal.py

  thermal_estimator/
    __init__.py
    estimator.py

  controller/
    __init__.py
    state_machine.py
    l1_guidance.py
    cruise_control.py
    probe_control.py
    circling_control.py

  simulator/
    __init__.py
    simulation.py
    plotting.py

  tests/
    test_thermal_model.py
    test_estimator.py
    test_controller.py
```

Keep code readable and beginner-friendly. Add comments explaining the math.

---

## 3. Step 1: Implement the Thermal Model

### Requirement

Implement a simplified Gaussian thermal updraft model.

The thermal has four parameters:

```text
x_c: thermal core x-coordinate
y_c: thermal core y-coordinate
W_0: maximum vertical air velocity at the core
R_th: thermal radius / spread parameter
```

For a glider at position `(x, y)`, compute radial distance:

```text
r = sqrt((x - x_c)^2 + (y - y_c)^2)
```

Then compute predicted thermal lift:

```text
w(r) = W_0 * exp(-r^2 / R_th^2)
```

### File

Implement in:

```text
thermal_model/thermal.py
```

### Expected API

```python
class GaussianThermal:
    def __init__(self, x_c: float, y_c: float, W_0: float, R_th: float):
        ...

    def vertical_velocity(self, x: float, y: float) -> float:
        """Return thermal lift w at position (x, y)."""
        ...
```

### Acceptance Criteria

- At the thermal center, `vertical_velocity(x_c, y_c)` should equal approximately `W_0`.
- Far from the center, vertical velocity should be close to zero.
- Add unit tests.

---

## 4. Step 2: Implement a Simple Glider Model

### Requirement

Implement a simple 2D/3D kinematic glider model.

The glider state should include:

```text
x, y: horizontal position
h: altitude
heading: direction of travel in radians
V: airspeed or ground speed, initially treated as the same
bank_angle: roll/bank angle phi
```

For the first version, ignore wind.

### Motion Update

At each timestep `dt`:

```text
x += V * cos(heading) * dt
y += V * sin(heading) * dt
```

Turn rate can be approximated by:

```text
heading_dot = g * tan(bank_angle) / V
heading += heading_dot * dt
```

Altitude update:

```text
h_dot = thermal_lift - sink_rate
h += h_dot * dt
```

### Sink Rate

Start with a simple sink rate model:

```text
sink_rate = base_sink_rate * load_factor
load_factor = 1 / cos(bank_angle)
```

A simple first value:

```text
base_sink_rate = 0.7 m/s
```

This captures the idea that higher bank angle increases sink rate.

### File

Implement in:

```text
glider_model/glider.py
```

### Expected API

```python
class GliderState:
    x: float
    y: float
    h: float
    heading: float
    V: float
    bank_angle: float

class SimpleGlider:
    def __init__(self, state: GliderState):
        ...

    def sink_rate(self) -> float:
        ...

    def step(self, bank_command: float, thermal_lift: float, dt: float) -> float:
        """
        Update the glider state.
        Return h_dot / vertical speed / net climb.
        """
        ...
```

### Acceptance Criteria

- With zero thermal lift, altitude decreases.
- With thermal lift greater than sink rate, altitude increases.
- Higher bank angle should increase sink rate.

---

## 5. Step 3: Implement Measurement Collection

### Requirement

During simulation, collect recent measurements:

```text
x_i, y_i, h_dot_i, sink_rate_i, w_meas_i
```

Use:

```text
w_meas_i = h_dot_i + sink_rate_i
```

Store a rolling window of the most recent N measurements.

Recommended starting value:

```text
N = 50
```

### File

Can be implemented inside:

```text
thermal_estimator/estimator.py
```

or as part of the simulator.

### Acceptance Criteria

- The measurement window should not grow forever.
- It should keep only the most recent N measurements.

---

## 6. Step 4: Implement the Thermal Estimator

### Requirement

Use nonlinear least squares optimization to estimate:

```text
x_c, y_c, W_0, R_th
```

from the recent measurement window.

The predicted vertical air velocity is:

```text
w_pred = W_0 * exp(-r^2 / R_th^2)
```

Minimize:

```text
J = sum((w_pred_i - w_meas_i)^2)
```

Then add regularization to avoid unstable jumps:

```text
J = sum((w_pred_i - w_meas_i)^2)
    + lambda_1 * (W_0 - W_0_prev)^2
    + lambda_2 * (R_th - R_th_prev)^2
    + lambda_3 * ((x_c - x_c_prev)^2 + (y_c - y_c_prev)^2)
```

### File

Implement in:

```text
thermal_estimator/estimator.py
```

### Expected API

```python
@dataclass
class ThermalEstimate:
    x_c: float
    y_c: float
    W_0: float
    R_th: float
    confidence: float

class ThermalEstimator:
    def __init__(self, window_size: int = 50):
        ...

    def add_measurement(self, x: float, y: float, h_dot: float, sink_rate: float):
        """Convert net climb to thermal lift and store the measurement."""
        ...

    def estimate(self) -> ThermalEstimate | None:
        """Fit the thermal model using the measurement window."""
        ...
```

### Implementation Notes

Use `scipy.optimize.minimize` or `scipy.optimize.least_squares`.

Apply reasonable bounds:

```text
W_0 > 0
R_th > 1
x_c, y_c within a reasonable simulation area
```

Use previous estimate as the next initial guess when available.

### Confidence Metric

Implement a simple confidence metric first.

Option A: based on mean squared error:

```text
mse = mean((w_pred - w_meas)^2)
confidence = 1 / (1 + mse)
```

Option B: chi-squared style:

```text
chi2 = mean(((w_pred - w_meas) / sigma)^2)
confidence = 1 / (1 + abs(chi2 - 1))
```

Start with Option A if easier.

### Acceptance Criteria

- If given synthetic data from a known thermal, the estimator should recover approximate `x_c`, `y_c`, `W_0`, and `R_th`.
- Estimates should not jump wildly between timesteps.
- If there are too few measurements, return `None`.

---

## 7. Step 5: Implement the State Machine

### Requirement

Implement three flight modes:

```text
CRUISE
PROBE
THERMAL
```

The state machine uses the thermal estimate and confidence.

Basic logic:

```text
If no estimate exists:
    mode = CRUISE
Else if estimate exists but confidence is below threshold:
    mode = PROBE
Else if confidence is high enough:
    mode = THERMAL
```

Recommended thresholds:

```text
probe_threshold = 0.2
thermal_threshold = 0.5
```

Adjust after testing.

### File

Implement in:

```text
controller/state_machine.py
```

### Expected API

```python
class GuidanceMode(Enum):
    CRUISE = "cruise"
    PROBE = "probe"
    THERMAL = "thermal"

class GuidanceStateMachine:
    def __init__(self, thermal_threshold: float = 0.5):
        ...

    def update(self, estimate: ThermalEstimate | None) -> GuidanceMode:
        ...
```

### Acceptance Criteria

- No estimate should produce `CRUISE`.
- Low confidence estimate should produce `PROBE`.
- High confidence estimate should produce `THERMAL`.

---

## 8. Step 6: Implement L1 Guidance

### Requirement

Implement L1 guidance to convert a desired lookahead point into lateral acceleration.

Core formula:

```text
a_s_cmd = 2 * V^2 / L1 * sin(eta)
```

Where:

```text
V: current speed
L1: lookahead distance
eta: angle between current velocity direction and vector from glider to lookahead point
```

Then convert lateral acceleration to bank angle:

```text
phi_cmd = atan(a_s_cmd / g)
```

Clamp bank angle to a safe range.

Recommended:

```text
max_bank_angle = 45 degrees
```

### File

Implement in:

```text
controller/l1_guidance.py
```

### Expected API

```python
class L1Guidance:
    def __init__(self, L1: float = 30.0, max_bank_deg: float = 45.0):
        ...

    def bank_to_point(self, glider_state: GliderState, target_x: float, target_y: float) -> float:
        """Return bank angle command to steer toward a lookahead/target point."""
        ...
```

### Acceptance Criteria

- If target is to the left of current velocity direction, bank command should turn left.
- If target is to the right, bank command should turn right.
- Bank command should be clamped.

---

## 9. Step 7: Implement Cruise Control

### Requirement

Cruise mode should fly toward a fixed waypoint.

Use L1 guidance to command bank angle toward the waypoint.

### File

Implement in:

```text
controller/cruise_control.py
```

### Expected API

```python
class CruiseControl:
    def __init__(self, waypoint_x: float, waypoint_y: float, l1: L1Guidance):
        ...

    def command(self, glider_state: GliderState) -> float:
        """Return bank angle command."""
        ...
```

### Acceptance Criteria

- Glider should generally turn toward the waypoint.

---

## 10. Step 8: Implement Probe Control

### Requirement

Probe mode should move toward the low-confidence estimated thermal center.

Start with a simple bang-bang controller:

```text
If thermal center is left of current heading:
    command +fixed_bank
Else:
    command -fixed_bank
```

Recommended:

```text
fixed_bank = 25 degrees
```

### File

Implement in:

```text
controller/probe_control.py
```

### Expected API

```python
class ProbeControl:
    def __init__(self, fixed_bank_deg: float = 25.0):
        ...

    def command(self, glider_state: GliderState, estimate: ThermalEstimate) -> float:
        """Return fixed left/right bank toward estimated thermal center."""
        ...
```

### Acceptance Criteria

- If estimated thermal center is left of heading, turn left.
- If estimated thermal center is right of heading, turn right.
- Should produce zig-zag or curved exploratory behavior in simulation.

---

## 11. Step 9: Implement Circling Control

### Requirement

Thermal mode should circle around the estimated thermal center.

For the first version, choose a circling radius based on the estimated thermal radius:

```text
R_circle = 0.5 * R_th
```

Clamp it:

```text
R_min = 10 m
R_max = 2 * R_th
```

Then generate a lookahead point on the desired circle and use L1 guidance to track it.

### Optional Optimization

After the basic version works, implement optimization:

```text
maximize net_climb = w(R) - sink_rate(V, phi)
```

Where:

```text
w(R) = W_0 * exp(-R^2 / R_th^2)
phi = atan(V^2 / (g * R))
sink_rate = base_sink_rate / cos(phi)
```

Search over candidate values of `R` and `V`, then choose the pair with maximum net climb.

Start with grid search instead of complex optimization.

Example:

```text
R candidates: 10 m to 2 * R_th
V candidates: 12 m/s to 30 m/s
```

### File

Implement in:

```text
controller/circling_control.py
```

### Expected API

```python
class CirclingControl:
    def __init__(self, l1: L1Guidance):
        ...

    def choose_radius(self, estimate: ThermalEstimate, V: float) -> float:
        """Return desired circle radius."""
        ...

    def command(self, glider_state: GliderState, estimate: ThermalEstimate) -> float:
        """Return bank angle command to circle around estimated thermal center."""
        ...
```

### Acceptance Criteria

- In thermal mode, the glider should circle around the estimated thermal center.
- The circle should stay inside or near the estimated thermal radius.
- Net climb should improve compared with random circling.

---

## 12. Step 10: Integrate the Simulator

### Requirement

Build a simulation loop.

At each timestep:

```text
1. Get current glider state.
2. Compute true thermal lift from the true thermal model.
3. Compute glider sink rate.
4. Step the glider using previous/selected bank command.
5. Get h_dot / net climb.
6. Add measurement to thermal estimator.
7. Run estimator.
8. Update state machine mode.
9. Choose bank command using cruise/probe/circling controller.
10. Log all states for plotting.
```

### File

Implement in:

```text
simulator/simulation.py
main.py
```

### Logging

Log at least:

```text
time
x, y, h
heading
bank_angle
h_dot
thermal_lift
sink_rate
mode
estimated_x_c, estimated_y_c
estimated_W_0, estimated_R_th
confidence
```

### Acceptance Criteria

- Simulation should run without crashing for at least 300 seconds.
- Plots should show flight path and estimated thermal center.
- Mode switching should be visible in logs.

---

## 13. Step 11: Plotting

### Requirement

Create useful plots:

1. 2D flight path with true thermal center and estimated thermal center.
2. Altitude vs time.
3. Mode vs time.
4. True vs estimated thermal parameters if available.
5. Net climb vs time.

### File

Implement in:

```text
simulator/plotting.py
```

### Acceptance Criteria

- Plots should clearly show whether the controller finds and circles the thermal.

---

## 14. Step 12: Tests

Add basic tests for:

1. Thermal model:
   - center has max lift
   - far away has low lift

2. Glider model:
   - zero thermal causes descent
   - sufficient thermal causes climb
   - higher bank increases sink rate

3. Estimator:
   - synthetic data from known thermal can recover approximate parameters

4. State machine:
   - no estimate -> cruise
   - low confidence -> probe
   - high confidence -> thermal

5. L1 guidance:
   - target left/right gives correct bank sign
   - bank is clamped

---

## 15. Implementation Order for Codex

Please implement in this exact order:

1. Create project structure.
2. Implement `GaussianThermal`.
3. Implement `SimpleGlider` and `GliderState`.
4. Implement measurement window logic.
5. Implement `ThermalEstimator` with basic least squares.
6. Add regularization to `ThermalEstimator`.
7. Implement simple confidence metric.
8. Implement `GuidanceStateMachine`.
9. Implement `L1Guidance`.
10. Implement `CruiseControl`.
11. Implement `ProbeControl`.
12. Implement basic `CirclingControl` using `R_circle = 0.5 * R_th`.
13. Integrate simulation loop.
14. Add plotting.
15. Add unit tests.
16. Run simulation and tune thresholds.
17. Optional: add circling radius optimization.
18. Optional: add wind model.
19. Optional: add Monte Carlo runs.

Do not start with advanced features. First make the simplest end-to-end system work.

---

## 16. Non-Goals for First Version

Do not implement these in the first version:

- Full aircraft dynamics.
- Real actuator dynamics.
- Wind shear.
- Thermal drift with altitude.
- Real sensor delay / variometer lag.
- EKF.
- Reinforcement learning.
- Multi-thermal selection.
- Hardware integration.
- ArduPilot/PX4/MAVLink integration.

These can be future extensions.

---

## 17. Future Extensions

After the first version works, consider:

1. Add wind and thermal drift.
2. Add variometer noise and lag.
3. Add Monte Carlo testing with random initial positions.
4. Add multi-thermal environment.
5. Add smarter probe control using exploration cost maps.
6. Add MacCready speed-to-fly in cruise mode.
7. Add confidence hysteresis to avoid rapid mode switching.
8. Compare estimator performance with and without regularization.
9. Try EKF again only after defining proper thermal dynamics.

---

## 18. Final Expected Result

The final simplified system should demonstrate:

```text
The glider starts in cruise mode.
It detects possible lift from variometer/net climb data.
It estimates a Gaussian thermal center, strength, and radius.
It enters probe mode while confidence is low.
It enters thermal mode when confidence is high.
It circles around the estimated thermal core.
It improves or maintains altitude when the thermal is strong enough.
```

This is mainly a physics simulation + online estimation + nonlinear control project, not a machine learning project.
