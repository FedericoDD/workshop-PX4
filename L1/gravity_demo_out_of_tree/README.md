# gravity_demo — minimal PX4 out-of-tree module

## What it teaches

1. A PX4 module has a lifecycle: `start`, `status`, `stop`.
2. PX4 modules obtain data from other modules through uORB.
3. `vehicle_attitude.q` represents the BODY/FRD -> NED attitude.
4. The same physical gravity vector can be expressed in another reference frame.

The calculation is shown only when
`gravity_demo status` is called.

## Directory structure

```text
gravity_demo_out_of_tree/
└── src/
    ├── CMakeLists.txt
    └── modules/
        └── gravity_demo/
            ├── CMakeLists.txt
            └── gravity_demo.cpp
```

This follows the PX4 external-module convention: the path passed to
`EXTERNAL_MODULES_LOCATION` contains a `src/` directory, and `src/CMakeLists.txt`
exports `config_module_list_external`.

## Build with PX4

Place this folder next to your PX4 checkout, for example:

```text
workspace/
├── PX4-Autopilot/
└── gravity_demo_out_of_tree/
```

Then:

```bash
./QGroundControl-x86_64.AppImage & # ONLY FOR UBUNTU
cd PX4-Autopilot
make px4_sitl EXTERNAL_MODULES_LOCATION="../gravity_demo_out_of_tree/"
```

Important: PX4 documentation notes that the external-module location should be
provided when configuring a fresh build directory. If an old SITL build already
exists and was configured without the external module, clean/reconfigure it.


## SITL example

After building the module, start the X500 simulation as usual for PX4 v1.17:

```bash
make px4_sitl gz_x500
```

If this command uses a different build directory than the one configured with
`EXTERNAL_MODULES_LOCATION`, configure that target with the external-module path too.

Then use the PX4 shell / MAVLink Console:

```text
gravity_demo start
gravity_demo status
gravity_demo stop
gravity_demo status
```


## Run from PX4 shell / QGroundControl MAVLink Console

```text
pxh> gravity_demo start
pxh> gravity_demo status
INFO  [gravity_demo] Running
INFO  [gravity_demo] g_NED  = [0.000, 0.000, 9.807] m/s^2
INFO  [gravity_demo] g_BODY = [...] m/s^2 (FRD)

pxh> gravity_demo stop
pxh> gravity_demo status
INFO  [gravity_demo] not running
```

For a level vehicle the expected result is approximately:

```text
g_BODY = [0, 0, +9.81] m/s^2
```

because both NED and BODY/FRD use a positive Down z-axis when the vehicle is level.

## Why is the rotation written manually?

For this first example the quaternion-to-vector expression
is written explicitly:

```text
reference frames -> quaternion -> rotation -> transformed vector
```

But you could use PX4's matrix library directly.
