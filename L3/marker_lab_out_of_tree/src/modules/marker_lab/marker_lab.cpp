/****************************************************************************
 * [R]overTech -- Lesson 3 -- Minimal PX4 1.18 SITL out-of-tree module.
 *
 * Same order as the slides:
 *   COMPANION -> aruco_companion.py: webcam -> ArUco -> solvePnP (slide 26)
 *   LINK      -> MAVLink DEBUG_VECT "ARUCO" = marker position in the CAMERA
 *                optical frame [m]; PX4 republishes it as uORB debug_vect
 *   FRAMES    -> rotate CAMERA optical -> BODY/FRD with fixed extrinsics
 *                (slides 10 and 12)
 *   OBSERVE   -> range, azimuth, elevation, rate and age on `marker_lab status`
 *
 * Read-only: we NEVER publish anything and never touch EKF2 or control.
 * No continuous terminal output.
 ****************************************************************************/
#include <px4_platform_common/module.h>        // PX4 task lifecycle: start/stop/status
#include <px4_platform_common/log.h>           // PX4_INFO and PX4_WARN
#include <px4_platform_common/posix.h>         // px4_usleep()
#include <px4_platform_common/tasks.h>         // px4_task_spawn_cmd()
#include <drivers/drv_hrt.h>                  // hrt_absolute_time()
#include <uORB/Subscription.hpp>              // read data without changing PX4
#include <uORB/topics/debug_vect.h>           // filled by MAVLink DEBUG_VECT

#include <pthread.h>                          // protects the result snapshot
#include <cerrno>
#include <cmath>
#include <cstring>

namespace lesson3
{
static constexpr char kMarkerName[] = "ARUCO";        // must match the companion
static constexpr uint64_t kStaleUs = 500000;          // 0.5 s without data = lost
static constexpr uint64_t kRateWindowUs = 2000000;    // rate measured over 2 s
static constexpr double kPi = 3.14159265358979323846;

struct V3 { double x, y, z; };

// EXTRINSICS T_BC (slide 12): CAMERA optical -> BODY/FRD.
// Forward-looking camera (slide 10): the camera z axis is the body x axis.
//   x_B (forward) = z_C (forward)
//   y_B (right)   = x_C (right)
//   z_B (down)    = y_C (down)
// The webcam is our "drone", so the lever arm is zero. On a real vehicle it
// is the camera position in BODY/FRD [m], as in EKF2_EV_POS_X/Y/Z (slide 33).
static constexpr double kR_BC[3][3] = {
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
};
static constexpr V3 kLeverArm_B{0.0, 0.0, 0.0};

inline V3 camera_to_body(const V3 &p_C)
{
    return {
        kR_BC[0][0] * p_C.x + kR_BC[0][1] * p_C.y + kR_BC[0][2] * p_C.z + kLeverArm_B.x,
        kR_BC[1][0] * p_C.x + kR_BC[1][1] * p_C.y + kR_BC[1][2] * p_C.z + kLeverArm_B.y,
        kR_BC[2][0] * p_C.x + kR_BC[2][1] * p_C.y + kR_BC[2][2] * p_C.z + kLeverArm_B.z,
    };
}
} // namespace lesson3

class MarkerLab : public ModuleBase
{
public:
    // PX4 ModuleBase pattern (same as PX4's template_module).
    static Descriptor desc;
    static int task_spawn(int argc, char *argv[]);
    static int run_trampoline(int argc, char *argv[]);
    static MarkerLab *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    MarkerLab() { pthread_mutex_init(&_mutex, nullptr); }
    ~MarkerLab() override { pthread_mutex_destroy(&_mutex); }
    void run() override;                    // called by `marker_lab start`
    int print_status() override;            // called by `marker_lab status`

private:
    uORB::Subscription _dbg{ORB_ID(debug_vect)};
    pthread_mutex_t _mutex{};                // status and worker are different tasks

    // Snapshot shared with print_status().
    lesson3::V3 _p_C{0.0, 0.0, 0.0};         // last marker position, CAMERA frame [m]
    uint64_t _last_us{0};                    // receive time of the last sample
    uint32_t _samples{0};                    // total ARUCO samples received
    float _rate_hz{0.f};                     // samples per second

    // Rate estimate: count samples over a fixed window.
    uint64_t _window_start_us{0};
    uint32_t _window_samples{0};
};

ModuleBase::Descriptor MarkerLab::desc{task_spawn, custom_command, print_usage};

// ---------------------------------------------------------------------------
// 1) INPUT: keep only the companion's ARUCO samples. Runs quietly.
// ---------------------------------------------------------------------------
void MarkerLab::run()
{
    while (!should_exit()) {
        debug_vect_s dbg{};

        // debug_vect is shared by every DEBUG_VECT sender: filter by name.
        while (_dbg.update(&dbg)) {
            if (strncmp(dbg.name, lesson3::kMarkerName, sizeof(dbg.name)) != 0) {
                continue;
            }

            // uORB fields are float32, the math below uses double.
            // Convert explicitly: PX4 treats -Wdouble-promotion as an error.
            const lesson3::V3 p_C{static_cast<double>(dbg.x),
                                  static_cast<double>(dbg.y),
                                  static_cast<double>(dbg.z)};

            pthread_mutex_lock(&_mutex);
            _p_C = p_C;
            // PX4 stamps debug_vect on RECEIPT, not at image capture:
            // the camera latency is invisible here (see EKF2_EV_DELAY).
            _last_us = dbg.timestamp;
            _samples++;
            _window_samples++;
            pthread_mutex_unlock(&_mutex);
        }

        const uint64_t now = hrt_absolute_time();
        if (_window_start_us == 0) {
            _window_start_us = now;

        } else if (now - _window_start_us >= lesson3::kRateWindowUs) {
            pthread_mutex_lock(&_mutex);
            _rate_hz = static_cast<float>(_window_samples * 1e6 / static_cast<double>(now - _window_start_us));
            _window_samples = 0;
            _window_start_us = now;
            pthread_mutex_unlock(&_mutex);
        }

        px4_usleep(10000); // 10 ms polling: webcams give ~30 Hz. NO prints here.
    }
}

// ---------------------------------------------------------------------------
// 2) OBSERVE: only this function prints results to the PX4/MAVLink shell.
// ---------------------------------------------------------------------------
int MarkerLab::print_status()
{
    pthread_mutex_lock(&_mutex);
    const lesson3::V3 p_C = _p_C;            // take a consistent snapshot
    const uint64_t last_us = _last_us;
    const uint32_t samples = _samples;
    const float rate_hz = _rate_hz;
    pthread_mutex_unlock(&_mutex);

    if (samples == 0) {
        PX4_WARN("running, waiting for ARUCO debug_vect (is aruco_companion.py sending?)");
        return 0;
    }

    const uint64_t now = hrt_absolute_time();
    const double age_s = (now > last_us) ? (now - last_us) * 1e-6 : 0.0;

    // FRAMES: same physical vector, different coordinates (slide 10).
    const lesson3::V3 p_B = lesson3::camera_to_body(p_C);

    // Range and bearing in BODY/FRD.
    const double degrees = 180.0 / lesson3::kPi;
    const double range = std::sqrt(p_B.x * p_B.x + p_B.y * p_B.y + p_B.z * p_B.z);
    const double azimuth = std::atan2(p_B.y, p_B.x) * degrees;                      // + = right
    const double elevation = std::atan2(-p_B.z, std::sqrt(p_B.x * p_B.x + p_B.y * p_B.y)) * degrees; // + = up

    PX4_INFO("RUNNING: ArUco marker from the companion (read-only)");
    PX4_INFO("p_CAM [x right, y down, z fwd] m: %.3f / %.3f / %.3f", p_C.x, p_C.y, p_C.z);
    PX4_INFO("p_FRD [x fwd, y right, z down] m: %.3f / %.3f / %.3f", p_B.x, p_B.y, p_B.z);
    PX4_INFO("range %.3f m | azimuth %.1f deg (+right) | elevation %.1f deg (+up)",
             range, azimuth, elevation);
    PX4_INFO("samples %u | rate %.1f Hz | age %.2f s",
             (unsigned)samples, (double)rate_hz, age_s);

    if (now > last_us && now - last_us > lesson3::kStaleUs) {
        PX4_WARN("marker lost / companion silent: last sample is %.2f s old", age_s);
    }

    return 0;
}

// ---------------------------------------------------------------------------
// 3) PX4 standard ModuleBase boilerplate. Students do NOT need to edit this.
//    PX4 itself handles start, stop and status; no other commands are added.
// ---------------------------------------------------------------------------
int MarkerLab::task_spawn(int /*argc*/, char *argv[])
{
    desc.task_id = px4_task_spawn_cmd("marker_lab", SCHED_DEFAULT,
                   SCHED_PRIORITY_DEFAULT, 4096,
                   (px4_main_t)&run_trampoline, (char *const *)argv);
    if (desc.task_id < 0) { desc.task_id = -1; return -errno; }
    return 0;
}

int MarkerLab::run_trampoline(int argc, char *argv[])
{
    return ModuleBase::run_trampoline_impl(desc,
        [](int ac, char *av[]) -> ModuleBase * { return MarkerLab::instantiate(ac, av); },
        argc, argv);
}

MarkerLab *MarkerLab::instantiate(int, char *[]) { return new MarkerLab(); }

int MarkerLab::custom_command(int, char *[])
{
    return print_usage("only start, status, stop are supported");
}

int MarkerLab::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s", reason);
    PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
# Lesson 3: observe an ArUco marker detected by a companion computer.

Read debug_vect "ARUCO" (marker position in the camera optical frame),
rotate it into BODY/FRD and print range and bearing ONLY on `marker_lab status`.
No effect on flight control.
)DESCR_STR");
    PRINT_MODULE_USAGE_NAME("marker_lab", "example");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); // PX4-provided stop and status
    return 0;
}

extern "C" __EXPORT int marker_lab_main(int argc, char *argv[])
{
    return ModuleBase::main(MarkerLab::desc, argc, argv);
}
