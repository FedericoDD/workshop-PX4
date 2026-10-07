/****************************************************************************
 * gravity_demo.cpp
 *
 * Minimal PX4 v1.17 out-of-tree teaching module.
 *
 * Goal:
 *   1. Read the current vehicle attitude from uORB.
 *   2. Express the gravity vector from the NED frame in the BODY/FRD frame.
 *   3. Print the result ONLY when the user runs: gravity_demo status
 *
 * Commands from the PX4 shell / QGroundControl MAVLink Console:
 *   gravity_demo start
 *   gravity_demo status
 *   gravity_demo stop
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>

#include <cerrno>

class GravityDemo : public ModuleBase
{
public:
    static Descriptor desc;

    // Functions required by PX4 ModuleBase.
    static int task_spawn(int argc, char *argv[]);
    static int run_trampoline(int argc, char *argv[]);
    static GravityDemo *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    // Code executed while the module is running.
    void run() override;

    // Called by: gravity_demo status
    int print_status() override;
};


// Connect our module to the standard PX4 commands: start / stop / status.
ModuleBase::Descriptor GravityDemo::desc{task_spawn, custom_command, print_usage};


void GravityDemo::run()
{
    // This teaching module intentionally does not print continuously.
    // It simply stays alive until "gravity_demo stop" is requested.
    // The useful calculation is performed on demand in print_status().
    while (!should_exit()) {
        px4_usleep(200000); // 200 ms
    }
}


int GravityDemo::print_status()
{
    // Subscribe to PX4's estimated attitude.
    uORB::Subscription attitude_sub{ORB_ID(vehicle_attitude)};
    vehicle_attitude_s attitude{};

    if (!attitude_sub.copy(&attitude)) {
        PX4_INFO("Running, but vehicle_attitude is not available yet");
        return 0;
    }

    // PX4 vehicle_attitude.q is a Hamilton quaternion [w, x, y, z]
    // describing the rotation BODY/FRD -> NED.
    const float w = attitude.q[0];
    const float x = attitude.q[1];
    const float y = attitude.q[2];
    const float z = attitude.q[3];

    // Gravity expressed in the NED frame.
    // NED uses +Z downward, so gravity is [0, 0, +g].
    constexpr float g = 9.80665f;

    // We want the SAME physical vector expressed in BODY/FRD coordinates:
    //
    //     g_BODY = R_BODY<-NED * g_NED
    //            = R_NED<-BODY^T * g_NED
    //
    // Because g_NED = [0, 0, g], only the third row of
    // R_NED<-BODY is needed. Writing it directly keeps the example small.
    const float g_body_x = 2.0f * (x * z - w * y) * g;
    const float g_body_y = 2.0f * (y * z + w * x) * g;
    const float g_body_z = (1.0f - 2.0f * (x * x + y * y)) * g;

    PX4_INFO("Running");
    PX4_INFO("g_NED  = [0.000, 0.000, %.3f] m/s^2", (double)g);
    PX4_INFO("g_BODY = [%.3f, %.3f, %.3f] m/s^2 (FRD)",
             (double)g_body_x, (double)g_body_y, (double)g_body_z);

    return 0;
}


int GravityDemo::task_spawn(int argc, char *argv[])
{
    desc.task_id = px4_task_spawn_cmd(
        "gravity_demo",
        SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,
        1200,
        (px4_main_t)&run_trampoline,
        (char *const *)argv);

    if (desc.task_id < 0) {
        desc.task_id = -1;
        return -errno;
    }

    return 0;
}


int GravityDemo::run_trampoline(int argc, char *argv[])
{
    return ModuleBase::run_trampoline_impl(
        desc,
        [](int ac, char *av[]) -> ModuleBase * {
            return GravityDemo::instantiate(ac, av);
        },
        argc,
        argv);
}


GravityDemo *GravityDemo::instantiate(int argc, char *argv[])
{
    // No parameters are needed in this first teaching example.
    (void)argc;
    (void)argv;

    return new GravityDemo();
}


int GravityDemo::custom_command(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    return print_usage("unknown command");
}


int GravityDemo::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Minimal teaching module: rotate the gravity vector from NED into BODY/FRD
using PX4's current vehicle_attitude estimate.

Nothing is printed continuously. Use `gravity_demo status` to see the result.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("gravity_demo", "example");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); // adds stop and status

    return 0;
}


extern "C" __EXPORT int gravity_demo_main(int argc, char *argv[])
{
    return ModuleBase::main(GravityDemo::desc, argc, argv);
}
