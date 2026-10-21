# Lesson 3 — Marker Lab

**[R]overTech / PoliSpace — Autonomous Drones and Perception**

The Lesson 3 exercise: **a marker of known size gives a metric pose from one image** (PnP, slide 26). It has two parts that talk over MAVLink, just like a real drone with a companion computer (slide 32):

1. **Companion (Python + OpenCV):** reads the laptop **webcam**, detects an **ArUco marker**, estimates its pose with `solvePnP` and prints the **distance** from the camera. It can also send the marker position to PX4.
2. **PX4 out-of-tree module `marker_lab`:** receives that position in SITL, rotates it from the **camera frame** into **BODY/FRD** and prints range and bearing on request with `status`.

```text
webcam ──► aruco_companion.py ──MAVLink DEBUG_VECT "ARUCO"──► PX4 SITL ──uORB debug_vect──► marker_lab status
           (host OS)                                          (mavlink receiver)            camera → FRD, range, bearing
           ArUco + solvePnP
```

The module only reads PX4 topics. It **does not publish anything, does not feed EKF2 and does not control the drone**. This is a teaching experiment, **not a precision-landing pipeline**. It does not spam the terminal: results are printed on demand with `status`.

## 1. Folder layout — do not copy files into PX4-Autopilot

```text
workspace/
├── PX4-Autopilot/             # your existing PX4 v1.18 checkout
└── marker_lab_out_of_tree/    # extract this ZIP here
    ├── README.md
    ├── companion/             # Python, runs on the HOST OS (where the webcam is)
    │   ├── requirements.txt
    │   ├── make_marker.py
    │   └── aruco_companion.py
    └── src/                   # PX4 module, compiled by PX4
        ├── CMakeLists.txt
        └── modules/
            └── marker_lab/
                ├── CMakeLists.txt
                └── marker_lab.cpp
```

As in Lesson 2, the directory passed to `EXTERNAL_MODULES_LOCATION` is **`marker_lab_out_of_tree`**, the one that contains `src`. PX4 only looks inside `src/`, so the `companion/` folder does not affect the build.

## 2. Prerequisites

- **PX4:** the same environment as Lessons 1 and 2. `make px4_sitl gz_x500` must already work without external modules.
- **Python ≥ 3.9** on the **host OS**: Windows, macOS or Ubuntu, wherever the webcam is. Under Windows this means **Windows Python, not WSL**: the webcam is usually not visible inside WSL2.

### Install Python (only if you do not have it)

First check: `python --version` (Windows) or `python3 --version` (Ubuntu / macOS). If it prints `Python 3.9` or newer, skip to the venv below.

**Windows**

1. Download the latest **Python 3** "Windows installer (64-bit)" from https://www.python.org/downloads/windows/.
2. In the first installer screen, **tick "Add python.exe to PATH"**, then click *Install Now*.
3. Close and reopen PowerShell, then check with `python --version`.

If `python` opens the Microsoft Store or prints nothing, Windows is using its "App execution alias": turn off the `python.exe` and `python3.exe` aliases in Settings → Apps → Advanced app settings → App execution aliases, then reopen PowerShell. Alternatively, use the launcher `py` in place of `python` (for example `py -m venv .venv`).

**Ubuntu** (Python 3 is already installed; only venv and pip may be missing):

```bash
sudo apt update
sudo apt install python3 python3-venv python3-pip
```

**macOS**

- With [Homebrew](https://brew.sh): `brew install python`, then use `python3`.
- Without Homebrew: download the macOS installer from https://www.python.org/downloads/macos/ and run it.

Check with `python3 --version`.

### Create the venv and install the packages

Install the Python packages in a **virtual environment (venv)**. It keeps the workshop packages separate from the system ones, and on **Ubuntu 23.04+ / 24.04** and **Homebrew Python on macOS** it is mandatory: there, a plain `pip install` fails with `externally-managed-environment`.

**Ubuntu / macOS** (here the command is often `python3`, not `python`):

```bash
cd marker_lab_out_of_tree/companion
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

On Ubuntu, if `venv` is missing: `sudo apt install python3-venv`.

**Windows (PowerShell, Windows Python, not WSL):**

```powershell
cd marker_lab_out_of_tree\companion
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

If PowerShell refuses to run `Activate.ps1`, allow local scripts for your user once with `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, then activate again.

The prompt now starts with `(.venv)`. **Activate the venv in every new terminal** before running `aruco_companion.py`. Type `deactivate` to leave it.

`requirements.txt` installs `opencv-python` (≥ 4.8, which includes `cv2.aruco`), `numpy` and `pymavlink`. Inside a fresh venv there are no conflicts. If you install **without** a venv and `opencv-contrib-python` is already present, do not install `opencv-python` as well: the two packages conflict, and the contrib version already contains everything we need.

### Recommended before the workshop

- [ ] `make px4_sitl gz_x500` works (Lessons 1–2), on the workshop PX4 v1.18 tag.
- [ ] Python ≥ 3.9 on the **host OS**: `python --version` (or `python3 --version`).
- [ ] The venv is created and `pip install -r requirements.txt` finished without errors.
- [ ] **Webcam permission:** on Windows, Settings → Privacy & security → Camera → allow desktop apps. On macOS, allow camera access to Terminal/iTerm the first time. Close Teams, Zoom and any other app that uses the webcam.
- [ ] **Webcam test:** `python aruco_companion.py --no-px4` opens the video window (without a marker it simply shows `marker id 0 not found`).
- [ ] **Windows only:** the WSL ↔ Windows network bridge is configured (section 4, preferably `networkingMode=mirrored`). When the Windows Firewall asks about `python.exe`, allow it.

The printed markers, their measured size and the tape measures are provided by the instructors in class.

## 3. Step A — Distance from the webcam only (no PX4)

With the venv active (`(.venv)` in the prompt):

```bash
python aruco_companion.py --no-px4 --id 0 --dict DICT_4X4_50 --size 0.10 --hfov 70
```

`--id`, `--dict` and `--size` (side of the black square in metres) are placeholders here: use the values given in class with the printed marker.

A window opens with the video. When the marker is visible, you see its outline, its axes (red x, green y, blue z) and a line like this:

```text
dist 0.82 m | Z 0.80 m | pinhole Z 0.81 m
```

The terminal also prints the camera-frame position about twice per second:

```text
t_cam = [+0.052, -0.110, +0.801] m | dist 0.82 m | Z 0.80 m | pinhole Z 0.81 m
```

- `dist` is the Euclidean distance `‖t‖` from the camera centre to the marker centre.
- `Z` is the depth along the optical axis.
- `pinhole Z` is the one-line check from slide 11 (`Z ≈ fx · L / side_px`), valid when the marker faces the camera.

Press **`q`** in the video window to quit. Use `--camera 1` if you have more than one webcam.

**We do not calibrate the webcam.** `K` is computed from the horizontal field of view (slide 13), and the script prints it at startup:

```text
fx = fy = (W/2) / tan(HFOV/2)      cx = W/2      cy = H/2      distortion = 0
```

Everybody starts from the same **`--hfov 70`**: laptop webcams are typically 60–80°, so it is a good first guess. You do not need to look up your exact model. Exercise 1 shows how to tune it on your own webcam.

## 4. Step B — Send the marker to PX4 SITL

### Ubuntu / macOS

Terminal 1 (PX4):

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500 EXTERNAL_MODULES_LOCATION="../marker_lab_out_of_tree"
```

Terminal 2 (companion):

```bash
cd marker_lab_out_of_tree/companion
source .venv/bin/activate
python aruco_companion.py --id 0 --size 0.10 --hfov 70 --px4 udpout:127.0.0.1:14580
```

`14580` is the UDP port where PX4 SITL listens on its **onboard** MAVLink link, the link a companion computer uses.

### Windows + WSL2

PX4 runs in an **Ubuntu terminal inside WSL2**, as in the previous lessons. The companion runs in **PowerShell with Windows Python**, because that is where the webcam is (activate the venv first: `.venv\Scripts\Activate.ps1`). The two must reach each other over the network. Choose one option:

- **Mirrored networking (Windows 11, recommended):** create `%UserProfile%\.wslconfig` containing

  ```ini
  [wsl2]
  networkingMode=mirrored
  ```

  then run `wsl --shutdown`, reopen Ubuntu and start PX4 again. `127.0.0.1` now works from both sides:

  ```powershell
  python aruco_companion.py --id 0 --size 0.10 --px4 udpout:127.0.0.1:14580
  ```

- **WSL IP address:** in Ubuntu, `hostname -I` prints an address such as `172.27.10.5`. Use it from Windows:

  ```powershell
  python aruco_companion.py --id 0 --size 0.10 --px4 udpout:172.27.10.5:14580
  ```

**Important on all platforms:** there is **no space after `EXTERNAL_MODULES_LOCATION=`**. If the module is not picked up because PX4 SITL was already configured with a different external path (for example `attitude_lab`), stop SITL, check with `pwd` that you are in the right PX4 checkout, and reconfigure:

```bash
rm -rf build/px4_sitl_default
make px4_sitl gz_x500 EXTERNAL_MODULES_LOCATION="../marker_lab_out_of_tree"
```

## 5. Commands in the PX4 shell

First check that the data is arriving at all. Enter these commands in the **PX4 shell (`pxh>`)**, not in the system shell (`$`):

```text
pxh> listener debug_vect
```

You should see `name: ARUCO` and the same `x, y, z` that the companion prints. If nothing arrives, the problem is the network (section 4), not the module.

Then:

```text
pxh> marker_lab start
pxh> marker_lab status
pxh> marker_lab stop
```

As in Lesson 2, only these **three `marker_lab` commands** are implemented. PX4's standard `ModuleBase` handles start, stop and status.

### What should you see?

With the marker about 80 cm **in front of the webcam**, slightly to the right and above the centre:

```text
INFO  [marker_lab] RUNNING: ArUco marker from the companion (read-only)
INFO  [marker_lab] p_CAM [x right, y down, z fwd] m: 0.052 / -0.110 / 0.801
INFO  [marker_lab] p_FRD [x fwd, y right, z down] m: 0.801 / 0.052 / -0.110
INFO  [marker_lab] range 0.810 m | azimuth 3.7 deg (+right) | elevation 7.8 deg (+up)
INFO  [marker_lab] samples 412 | rate 29.8 Hz | age 0.03 s
```

- `p_CAM` is exactly what `solvePnP` returned (camera optical frame).
- `p_FRD` is the **same vector in body coordinates**. The camera's z (forward) becomes the body's x (slide 10).
- When the marker is covered, the companion stops sending. After 0.5 s `status` warns `marker lost / companion silent`.

## 6. Exercises

1. **Is the distance right?** Place the marker at 0.5, 1.0, 1.5 and 2.0 m, measured with a tape, and fill in a table with columns *true distance | dist | error %*. If the error is always in the same direction, change `--hfov` until it shrinks. A wrong `K` gives a **bias**, not noise, and no filter can average it away (slides 13 and 17).
2. **Scale comes from the known size.** Pass a `--size` that is twice the real value. The distance doubles and the picture does not change at all. A single camera cannot see scale; the prior "this marker is 10 cm" fixes it (slide 19).
3. **Distortion.** Keep the distance fixed and move the marker from the centre to the corners of the image. Does the distance change? Our model assumes `distortion = 0`, and wide-angle lenses are not distortion-free (slide 15).
4. **Frames.** Move the marker right, left, up and down. Predict the sign of every component of `p_CAM` and `p_FRD`, then check with `marker_lab status` (slide 10).
5. **Noise grows with distance.** Keep the marker still at 0.5 m and then at 2 m, and watch how much `t_cam` fluctuates in the terminal. The same ~0.5 px of corner noise weighs much more when the marker covers few pixels (slides 16 and 21).
6. *(Bonus)* Tilt the marker by about 60°. `Z` from PnP stays correct, but `pinhole Z` does not: the pinhole check assumes the marker faces the camera, while PnP uses all 4 corners.

## 7. Link the Lesson 3 equations to the code

| Slide idea | Code to inspect | Equation / meaning |
|---|---|---|
| Intrinsics from the FOV | `aruco_companion.py`: `intrinsics_from_fov()` | `fx = (W/2) / tan(HFOV/2)`, `cx = W/2`, `cy = H/2` |
| Distortion (ignored) | `intrinsics_from_fov()`: `dist` | `k1 = k2 = p1 = p2 = k3 = 0` |
| Features | `main()`: `detector.detectMarkers()` | 4 corner pixels per marker |
| Known 3D points | `marker_object_points()` | corners `(±L/2, ±L/2, 0)` in the marker frame |
| PnP: metric pose from one image | `main()`: `cv2.solvePnP(..., SOLVEPNP_IPPE_SQUARE)` | `λ·(u,v,1) = K·[R \| t]·(X,Y,Z,1)` → `R`, `t` |
| Pinhole model | `main()`: `z_pinhole` | `Z ≈ fx · L / side_px` |
| Companion → PX4 | `Px4Link.send_marker()` | MAVLink `DEBUG_VECT` → uORB `debug_vect` |
| Extrinsics, camera → body | `marker_lab.cpp`: `camera_to_body()` | `p_B = R_BC · p_C + t_B` |
| Range and bearing | `MarkerLab::print_status()` | `‖p_B‖`, `atan2(y_B, x_B)`, `atan2(−z_B, ‖(x_B, y_B)‖)` |
| Vision can fail | `print_status()`: `kStaleUs` | no sample for 0.5 s → "marker lost" |

**Conventions:** the **camera optical frame** is x right, y down, z forward (OpenCV). `tvec` from `solvePnP` is the **marker centre expressed in the camera frame**, in the same unit as `--size` (metres). **BODY/FRD** is x forward, y right, z down (PX4). For a forward-looking camera, `R_BC` maps `x_B = z_C`, `y_B = x_C`, `z_B = y_C`. The lever arm `t_B` is zero because our "drone" is the webcam. On a real vehicle it is the camera position in the body frame, the same idea as `EKF2_EV_POS_X/Y/Z` (slide 33).

### Assumptions, deliberately simple

- **No calibration:** `K` from a nominal HFOV, square pixels, principal point at the image centre, **zero distortion**.
- **Single marker:** only `--id` is used; other detected markers are only drawn.
- **`DEBUG_VECT` as the transport:** it is a generic debug message, so PX4 does nothing with it. We do not use `ODOMETRY`, because it would be fused by EKF2 (`EKF2_EV_CTRL`) and would disturb the simulated drone. We do not use `LANDING_TARGET` either, because PX4 only accepts it in `MAV_FRAME_LOCAL_NED`, which needs the vehicle pose. A real precision landing chain is `marker + PnP → LANDING_TARGET → PLD_*` (slide 35).
- **Timestamp on receipt:** PX4 stamps `debug_vect` when it **arrives**, not when the image was captured. Camera and network latency are invisible here. In a real system you would timestamp at capture and tune `EKF2_EV_DELAY` (slide 34).
- **No filtering, no covariance, no outlier rejection:** every detection is sent as it is. Exercise 5 shows why a real system needs them.
