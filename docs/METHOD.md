# Method

Everything below is the method as implemented in the models of this repository,
read out of the extracted MATLAB Function blocks (`src/model_code/`) and the
initialisation scripts (`src/matlab/`). Equations use LaTeX notation.

## 1. Platform model

The TurtleBot is a differential-drive robot, modelled at kinematic level as a
unicycle with pose `q = [x, y, θ]ᵀ` and body inputs `u = [v, ω]ᵀ`:

```
q̇ = [v cos θ, v sin θ, ω]ᵀ
```

Wheel angular speeds and body velocities are related by the wheel radius `r`
and wheel base `d`:

```
v = r (ω_R + ω_L) / 2            ω_L = v/r − d ω /(2r)
ω = r (ω_R − ω_L) / d            ω_R = v/r + d ω /(2r)
```

Both conversions are implemented as MATLAB Function blocks
(`kin2omegawheels`, `q_dot`) and used identically in simulation and on the
real robot. The integrator in simulation is the same unicycle equation
evaluated with a fixed step `Ts = 0.04 s`.

## 2. Posture regulation (Task 1)

The regulation law is the **Astolfi polar-coordinate controller**. With
`ρ = ‖q_goal − q̂‖` the distance to the target, `γ` the bearing of the target
in the robot frame and `δ = γ + θ̂ − θ_goal` the orientation mismatch:

```
v     = k₁ ρ cos γ
ω     = k₂ γ + k₁ (sin γ / γ) cos γ (γ + k_θ δ)
```

with `k₁ = 0.5`, `k₂ = 1.2`, `k_θ = 0.3`. When `ρ < 1 mm` the translation is
switched off and the robot rotates in place with `ω = k_θ e_θ`, stopping only
when `|e_θ| < 0.05 rad`. Both inputs are saturated in the block itself:
`|v| ≤ 0.22 m/s`, `|ω| ≤ 0.6 rad/s` — the 10 rad/s figure is the *wheel* limit
and is reached only through the gearing of the kinematic conversion, which the
code comments make explicit.

The same law is what Task 4 uses in `ctrl_mode = 1` and what Task 3 uses to
reach the circle start point (`k_ρ = 0.55`, `k_α = 1.20`, `k_final = 0.95`,
`ρ_switch = 5 mm`, with the linear velocity reduced to zero when the bearing
error exceeds 1.2 rad so that the robot does not cut the corner).

## 3. Trajectory tracking (Tasks 3 and 4)

In robot-frame error coordinates

```
e₁ =  cos θ Δx + sin θ Δy
e₂ = −sin θ Δx + cos θ Δy
e₃ = wrap(θ_d − θ)
```

the tracking law is feedforward plus feedback:

```
v     = v_ref cos e₃ + k₁ e₁
ω     = ω_ref + k₂ v_ref sinc(e₃) e₂ + k₃ e₃
```

Task 3 uses `k₁ = 0.70, k₂ = 2.30, k₃ = 0.70`; the Task 4 variant uses
`k₁ = 0.75, k₂ = 2.00, k₃ = 1.20` and `sinc(e₃)` is replaced by `sin(e₃)` with
`ω_lim = 0.8 rad/s`. Saturation is applied inside the controller, and the
wheel-speed limit is enforced again downstream.

## 4. Reference generation and task sequencing

A **task manager** (a persistent state machine in a MATLAB Function block)
produces the target pose and the feedforward pair `(v_ref, ω_ref)` each step;
the controller has no knowledge of which task is running.

* **Task 2** (`task2_10_fcn.m`): waypoint 1 → waypoint 2 → goal, with
  thresholds `0.10 m`, `0.08 m` + `0.45 rad`, `0.05 m`, plus a 4 s safety
  timeout at waypoint 2 so a small position offset cannot stall the run.
* **Task 3** (`task3_10_Task_Manager.m`): go to the circle start point
  `(1.45, −0.30)` with tangent heading, switch when `ρ < 0.05 m` and
  `|e_θ| < 0.05 rad`, then follow the circle of radius `R = 0.6265 m` around
  the cube at `(2.00, 0.00)` — `ω_circle = 0.30 rad/s`, 1.5 turns,
  `v_ref = R ω_circle = 0.188 m/s` — then hold the final pose.
* **Task 4** (`task4_06_Task_Manager.m`): a Hermite approach trajectory
  (8 s) replacing the pure regulation phase, then the same 1.5-turn circle,
  then hold.

Two deliberate design choices are visible in the code: switching out of a
regulation phase requires **both** position and heading tolerance (position
alone leaves the robot entering the circle with the wrong tangent), and the
thresholds are chosen larger than the controller's own stopping tolerance so
the two cannot deadlock.

## 5. Localization

### 5.1 Odometry / IMU and Vicon fusion (Tasks 1–3)

The localizer predicts with the unicycle model integrated at `Ts` and corrects
with the Vicon measurement when it is valid. The measurement model is the
identity on the pose, so the correction is a linear Kalman update with a
3×3 measurement covariance. Two details matter in practice and are both in the
code:

* **Gating.** A measurement is used only if it is plausible given the
  prediction; during the 90 %-loss phase of Task 3 the raw Vicon stream
  contains zeros, which a naive EKF update would interpret as the robot
  teleporting to the origin.
* **Initialisation.** On hardware the filter does not start until the first
  valid Vicon sample arrives, and it locks onto it (`is_real_robot == 1`
  branch); in simulation the initial pose is taken from the workspace.

### 5.2 LiDAR-EKF without motion capture (Task 4)

`LiDAR_EKF_Localization.m` (383 lines) replaces the mocap correction with a
range measurement to the **known** map, in this activity the cube walls:

* nonlinear range model `h(q) = distance from the predicted pose to the wall`
  along each LiDAR ray, and the measurement Jacobian of that ray-wall
  intersection with respect to `(x, y, θ)`;
* **adaptive gating**: an innovation gate of 0.045 m and at least 4 valid rays
  when the initial pose is trusted, a wider gate (0.120 m) and a lower minimum
  (3 rays) when it is not, so the filter can recover from a deliberately wrong
  initialisation;
* **automatic initial covariance selection** comparing the initial pose with
  the nominal reference (`1e-4` vs `5e-3` on position, `1°` vs `7°` on heading);
* process noise `Q = diag(1.5e-5, 1.5e-5, 6e-5)`;
* scalar, **sequential** updates over a 36-beam scan, with the covariance
  updated in Joseph form for numerical stability — a beam that is non-finite
  (no intersection with the cube) or outside the gate is skipped, and if fewer
  than `min_valid` beams survive, the update is skipped altogether and the
  prediction is kept;
* diagnostic outputs — innovation RMS and the number of valid rays — which are
  what the "LiDAR innovation" figure plots.

**Two variants are archived, and the file names are misleading.** The archive
contains `task4_simulation.slx`, whose EKF evaluates the Jacobian numerically
by finite differences, and a second model whose author-given name is
`task4_simulation_backup_before_analytic_H.slx` but which in fact implements the
**analytical** ray-wall Jacobian (`lidarModelAnalytic`, `rayLineJacobian`). The
submitted report describes, and its Task 4 results use, the analytical version
(report §Task 4); the exported runs `task4_exact_analytic_jacobian.mat` and
`task4_wrong_ekf_analytic_jacobian.mat` are its outputs. Here the two models
are therefore named after what they contain —
`task4_simulation.slx` (numerical) and
`task4_simulation_analytic_jacobian.slx` (analytical, the reported one) — and
both are shipped so nothing is lost.


## 6. Measurement-degradation model

The simulation must reproduce the hardware's failure modes, not an idealised
sensor. The `check` block of the models implements the map regions directly:

| Region | Extent | Model |
|---|---|---|
| noisy Vicon (red) | `x ∈ [−0.60, 0.20]`, `y ∈ [−1.50, 0.10]` | Gaussian noise, `σ = 0.02 m/rad` |
| Vicon loss (orange) | `x ∈ [1.00, 2.90]`, `y ∈ [−1.50, 1.50]` | measurement returned as zero with `p_loss = 0.9` |

The condition index it returns is logged with the run, which is what lets the
analysis script report an RMSE *inside* a region separately from the whole run
(the Task 2 comparison between 0.0168 m overall and 0.0235 m in the red area is
that separation).

## 7. Real-robot integration

On hardware the loop closes through ROS: a generated ROS node (Simulink ROS
Toolbox, ERT target) publishes wheel-speed commands and subscribes to the
encoder/IMU/Vicon topics, while the logged signals are written to a
`Simulink.SimulationOutput` object. The Vicon interface itself is a course
block (`getViconMeasure_*.slx`) and is not redistributed — see
`THIRD_PARTY_NOTICES.md`.

## 8. Where to read the code

| Topic | File |
|---|---|
| Astolfi regulation, saturation | `src/model_code/task1/task1_01_fcn.m` |
| EKF (odometry + Vicon) | `src/model_code/task2/task2_06_fcn.m` |
| Waypoint manager | `src/model_code/task2/task2_10_fcn.m` |
| Tracking + regulation, Task 3 | `src/model_code/task3/task3_01_Controller.m` |
| Circle / task sequencing | `src/model_code/task3/task3_10_Task_Manager.m` |
| LiDAR-EKF | `src/model_code/task4/task4_03_LiDAR_EKF_Localization.m` |
| Hermite + circle reference | `src/model_code/task4/task4_06_Task_Manager.m` |
| Sensor simulation | `src/model_code/task4/task4_07_Sensor_Simulation.m` |
| Workspace initialisation | `src/matlab/fa.m`, `src/matlab/FA_modified.m` |
| Map drawing | `src/matlab/draw_robot.m` |
