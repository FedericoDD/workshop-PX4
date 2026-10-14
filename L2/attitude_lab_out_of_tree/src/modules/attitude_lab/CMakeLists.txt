# A PX4 module, but its source lives OUTSIDE PX4-Autopilot.
px4_add_module(
    MODULE modules__attitude_lab
    MAIN attitude_lab
    STACK_MAIN 8192
    SRCS
        attitude_lab.cpp
    EXTERNAL
)
# Intentionally no DEPENDS platforms__common: that target can be missing
# when external modules are configured in PX4 v1.18 (see README).
