# Lesson 2 — Attitude Lab 

**[R]overTech / PoliSpace — Autonomous Drones and Perception**

A small, *out-of-tree* PX4 module that runs **two attitude estimates in parallel**:

1. **Gyro only:** integrate the gyroscope without any measurement correction.
2. **Educational error-state EKF:** use the *same* gyro increments to predict attitude and gyro bias, then correct with the accelerometer and magnetometer.

The module only reads PX4 topics. It **does not replace EKF2, publish `vehicle_attitude`, or control the drone**. This is a teaching experiment for SITL, **not a flight-ready estimator**. It does not spam the terminal: results are printed on demand with `status`.

## 1. Folder layout — do not copy files into PX4-Autopilot

```text
workspace/
├── PX4-Autopilot/             # your existing PX4 v1.18 checkout
└── attitude_lab_out_of_tree/  # extract this ZIP here
    ├── README.md
    └── src/
        ├── CMakeLists.txt
        └── modules/
            └── attitude_lab/
                ├── CMakeLists.txt
                ├── attitude_lab.cpp
                └── attitude_filter.hpp
```

The directory passed to `EXTERNAL_MODULES_LOCATION` is **`attitude_lab_out_of_tree`**, which contains the `src` directory. `src/CMakeLists.txt` declares `config_module_list_external` and the module's CMake file calls `px4_add_module(... EXTERNAL)`. This is the official PX4 out-of-tree mechanism.

## 2. Prerequisite: official PX4 development environment is ALREADY working

Follow the official setup for your OS first. You should already be able to run `make px4_sitl gz_x500` with no external module:

- **Ubuntu 22.04 / 24.04:** https://docs.px4.io/v1.18/en/dev_setup/dev_env_linux_ubuntu
- **Windows 10 / 11 + WSL2:** https://docs.px4.io/v1.18/en/dev_setup/dev_env_windows_wsl
- **macOS (Intel / Apple Silicon):** https://docs.px4.io/v1.18/en/dev_setup/dev_env_mac

Keep all workshop computers on the **same PX4 v1.18 commit or tag**. Check the current one with `git describe --tags --always` inside PX4-Autopilot. This exercise has no additional packages beyond the official PX4 setup.

## 3. Run it — Ubuntu

1. Extract the ZIP so `attitude_lab_out_of_tree/` is **next to** `PX4-Autopilot/` (as in the tree above).
2. Open a terminal in the existing PX4 checkout.
3. Run:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500 EXTERNAL_MODULES_LOCATION="../attitude_lab_out_of_tree"
```

If your PX4 repo is elsewhere, replace `~/PX4-Autopilot` with its real path. The `../attitude_lab_out_of_tree` path is always interpreted **relative to the PX4 repository**.

## 4. Run it — Windows + WSL2

Use an **Ubuntu terminal inside WSL2**, **not PowerShell**. Extract both PX4 and this module into the **WSL Linux filesystem**, not `/mnt/c/...` (performance and permissions can suffer there).

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500 EXTERNAL_MODULES_LOCATION="../attitude_lab_out_of_tree"
```

The PX4 documentation explains WSL2 graphics / WSLg and Gazebo. The module itself needs no Windows-specific code.

## 5. Run it — macOS

Use Terminal or iTerm. After following the **official PX4 macOS setup** (including Gazebo and XQuartz), run the same command:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500 EXTERNAL_MODULES_LOCATION="../attitude_lab_out_of_tree"
```

If your own PX4 installation uses a Python virtual environment, activate it first according to that setup. The module does not require a separate Python environment. If Gazebo graphics do not open, verify the official macOS Gazebo/XQuartz installation **before** debugging `attitude_lab`.

**Important on all platforms:** there is **no space after `EXTERNAL_MODULES_LOCATION=`**. If the module was not picked up because PX4 SITL had already been configured with a different external path, stop SITL, confirm you are inside the correct PX4 checkout using `pwd`, and reconfigure with:

```bash
rm -rf build/px4_sitl_default
make px4_sitl gz_x500 EXTERNAL_MODULES_LOCATION="../attitude_lab_out_of_tree"
```

That last step forces a fresh SITL build (it can take several minutes). Do not delete other build directories.

## 6. Three commands, in the PX4 shell

Wait for PX4 SITL to start. Enter these commands in the **PX4 shell (`pxh>`)**, not in the system shell (`$`). The PX4 shell is also accessible from the QGroundControl MAVLink Console when connected.

```text
pxh> attitude_lab start
pxh> attitude_lab status
pxh> attitude_lab stop
```

Only these **three `attitude_lab` commands** are implemented. PX4's standard `ModuleBase` manages the start/stop/status lifecycle (and automatically provides a usage/help response). `status` shows: gyro-only roll/pitch/yaw, EKF roll/pitch/yaw, estimated bias, accepted/rejected updates, and (when available) PX4 attitude as an *informational reference*. The task does **not** print estimates continuously.

For a visible experiment, start **while the X500 is level on the ground**, wait 5–30 seconds and repeat `status`. To restart both estimators from initial conditions, run `stop` then `start`.

### What should you see?

- **GYRO ONLY**: with a fixed demonstration yaw-gyro bias of **+0.5°/s**, its heading slowly drifts (about **15° after 30 s** if the drone stays still).
- **EKF**: uses accelerometer and magnetometer corrections to oppose the drift and estimate the gyro bias.
- **PX4**: the normal flight attitude estimate; our two algorithms do not affect it.

The demo bias is deliberately added to **both** gyro inputs in `attitude_filter.hpp` (`kDemoYawBiasRadS`). This makes drift visible even with an unrealistically perfect SITL gyro. The bias is **not** a separate command; change that one constant to `0.0` if you want the unmodified SITL measurement.

The EKF learns an initial magnetic field reference from the first valid magnetometer sample: its yaw is thus **relative to initial heading**, not automatically an absolute North-referenced heading. Compare *changes* and drift, not necessarily identical initial yaw angles across the two estimators and PX4. Static/low-acceleration motion is best: accelerometers measure **specific force**, and large dynamic accelerations invalidate the gravity-direction approximation.

## 7. Link the Lesson 2 equations to the code

| Slide idea | Code to inspect | Equation / meaning |
|---|---|---|
| Sensors become software data | `attitude_lab.cpp`: `_imu`, `_mag` | uORB `vehicle_imu`, `vehicle_magnetometer` |
| Gyroscope predicts attitude | `AttitudeFilter::predict()` | `q(k+1) = q(k) * delta_q(dtheta)` |
| Gyro-only integration drifts | `predict()`: `gyro_only` | No measurement correction on this quaternion |
| EKF predicts uncertainty | `predict()`: `F`, `P`, `Q` | `P- = F P F' + Q` |
| Accelerometer corrects tilt | `correct_accel()` | `z_acc ~ -R(q)' * e_D` (quasi-static) |
| Magnetometer aids heading | `correct_mag()` | `z_mag ~ R(q)' * m_N` |
| Innovation and Kalman gain | `update_vector()` | `r=z-h(q)`, `K=P H' (H P H'+R)^-1` |
| Correct quaternion and bias | `update_vector()`: `ekf`, `bias` | Apply local angle error, update gyro bias |

**Conventions:** `q` rotates BODY/FRD into NED (Hamilton quaternion). In NED, gravity acceleration points `[0,0,+g]`; a level accelerometer measures **specific force** `[0,0,-g]` in FRD. `vehicle_imu.delta_angle` is already an *integrated* angular increment in **radians** with `delta_angle_dt` in **microseconds**, while `delta_velocity` is an integrated increment in m/s. The magnetometer field `magnetometer_ga` is given in **Gauss** in body FRD; it is normalized in the estimator.

### Assumptions, deliberately simple

- Nominal quaternion plus a **6-dimensional local error** `[attitude error (3), gyro-bias error (3)]`.
- First-order error transition `F`, illustrative fixed process and measurement covariances (not calibrated from a datasheet).
- Vector innovations for accelerometer and magnetometer; reject accelerometer samples if magnitude differs by more than ±15% from `g` or direction residual is too large.
- No IMU delay compensation, no timing interpolation between magnetometer and IMU, no vibration modeling, no quality-based magnetic disturbance detection, no gravity compensation during aggressive motion.

