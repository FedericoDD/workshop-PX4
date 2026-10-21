# A PX4 module, but its source lives OUTSIDE PX4-Autopilot.
px4_add_module(
    MODULE modules__marker_lab
    MAIN marker_lab
    STACK_MAIN 4096
    SRCS
        marker_lab.cpp
    EXTERNAL
)
# Intentionally no DEPENDS platforms__common: that target can be missing
# when external modules are configured in PX4 v1.18 (see L2 README).
