/****************************************************************************
 * [R]overTech -- Lesson 2 -- Minimal PX4 1.18 SITL out-of-tree module.
 *
 * Same order as the slides:
 *   INPUT     -> read vehicle_imu and vehicle_magnetometer from uORB
 *   PREDICT   -> integrate gyro into TWO attitudes (gyro-only and EKF)
 *   CORRECT   -> accelerometer corrects tilt; magnetometer aids heading
 *   OBSERVE   -> inspect BOTH estimates using `attitude_lab status`
 *
 * PX4 EKF2 keeps controlling the vehicle. We NEVER publish vehicle_attitude
 * or actuator commands. No continuous terminal output.
 ****************************************************************************/
#include <px4_platform_common/module.h>        // PX4 task lifecycle: start/stop/status
#include <px4_platform_common/log.h>           // PX4_INFO and PX4_WARN
#include <px4_platform_common/posix.h>         // px4_usleep()
#include <px4_platform_common/tasks.h>         // px4_task_spawn_cmd()
#include <drivers/drv_hrt.h>                  // hrt_absolute_time()
#include <uORB/Subscription.hpp>              // read data without changing PX4
#include <uORB/topics/vehicle_imu.h>          // integrated delta angle/velocity
#include <uORB/topics/vehicle_magnetometer.h> // magnetic field in FRD [Gauss]
#include <uORB/topics/vehicle_attitude.h>     // optional PX4 reference for status

#include <pthread.h>                          // protects the result snapshot
#include <cerrno>
#include "attitude_filter.hpp"                // pure C++ estimation equations

class AttitudeLab : public ModuleBase
{
public:
    // PX4 ModuleBase pattern (same as PX4's template_module).
    static Descriptor desc;
    static int task_spawn(int argc, char *argv[]);
    static int run_trampoline(int argc, char *argv[]);
    static AttitudeLab *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    AttitudeLab() { pthread_mutex_init(&_mutex, nullptr); }
    ~AttitudeLab() override { pthread_mutex_destroy(&_mutex); }
    void run() override;                    // called by `attitude_lab start`
    int print_status() override;            // called by `attitude_lab status`

private:
    uORB::Subscription _imu{ORB_ID(vehicle_imu)};
    uORB::Subscription _mag{ORB_ID(vehicle_magnetometer)};
    pthread_mutex_t _mutex{};                // status and worker are different tasks
    lesson2::AttitudeFilter _filter;         // the TWO estimators live in one object
    uint64_t _last_imu_us{0};
};

ModuleBase::Descriptor AttitudeLab::desc{task_spawn, custom_command, print_usage};

// ---------------------------------------------------------------------------
// 1) INPUT -> PREDICT -> CORRECT. This task runs quietly in the background.
// ---------------------------------------------------------------------------
void AttitudeLab::run()
{
    while (!should_exit()) {
        vehicle_imu_s imu{};

        if (_imu.update(&imu) && imu.delta_angle_dt > 0) {
            // PX4 gives angular INCREMENTS [rad], not gyro rates [rad/s].
            // delta_angle_dt is microseconds. We do not integrate twice.
            const double dt = imu.delta_angle_dt * 1.e-6;
            const lesson2::V3 dangle{imu.delta_angle[0], imu.delta_angle[1], imu.delta_angle[2]};

            pthread_mutex_lock(&_mutex);
            _filter.predict(dangle, dt);             // gyro-only AND EKF prediction
            _last_imu_us = imu.timestamp_sample;

            if (imu.delta_velocity_dt > 0) {
                // Delta velocity divided by time -> specific force [m/s^2].
                // Level and nearly static in BODY/FRD: approx [0, 0, -g].
                const double dt_acc = imu.delta_velocity_dt * 1.e-6;
                // uORB delta_velocity is float32, whereas our EKF uses double.
                // Convert explicitly: PX4 treats -Wdouble-promotion as an error.
                const lesson2::V3 specific_force{
                    static_cast<double>(imu.delta_velocity[0]) / dt_acc,
                    static_cast<double>(imu.delta_velocity[1]) / dt_acc,
                    static_cast<double>(imu.delta_velocity[2]) / dt_acc
                };
                _filter.correct_accel(specific_force); // EKF correction: roll/pitch
            }
            pthread_mutex_unlock(&_mutex);
        }

        vehicle_magnetometer_s mag{};
        if (_mag.update(&mag)) {
            pthread_mutex_lock(&_mutex);
            _filter.correct_mag({mag.magnetometer_ga[0], mag.magnetometer_ga[1],
                                 mag.magnetometer_ga[2]}); // EKF correction: yaw aid
            pthread_mutex_unlock(&_mutex);
        }

        px4_usleep(2000); // 2 ms polling. NO prints inside the loop.
    }
}

// ---------------------------------------------------------------------------
// 2) OBSERVE: only this function prints estimates to the PX4/MAVLink shell.
// ---------------------------------------------------------------------------
int AttitudeLab::print_status()
{
    pthread_mutex_lock(&_mutex);
    const lesson2::AttitudeFilter f = _filter; // take consistent snapshot
    const uint64_t sample_time = _last_imu_us;
    pthread_mutex_unlock(&_mutex);

    if (sample_time == 0) {
        PX4_WARN("running, waiting for vehicle_imu data");
        return 0;
    }
    const uint64_t now = hrt_absolute_time();
    if (now > sample_time && now - sample_time > 1000000) {
        PX4_WARN("IMU data is more than 1 second old");
    }

    const double degrees = 180.0 / lesson2::kPi;
    const auto gyro = f.gyro_only.rpy();
    const auto ekf = f.ekf.rpy();

    PX4_INFO("RUNNING: gyro-only and EKF in parallel (read-only)");
    PX4_INFO("GYRO roll/pitch/yaw deg: %.2f / %.2f / %.2f",
             gyro.x*degrees, gyro.y*degrees, gyro.z*degrees);
    PX4_INFO("EKF  roll/pitch/yaw deg: %.2f / %.2f / %.2f",
             ekf.x*degrees, ekf.y*degrees, ekf.z*degrees);
    PX4_INFO("EKF gyro bias [deg/s]: %.3f / %.3f / %.3f",
             f.bias.x*degrees, f.bias.y*degrees, f.bias.z*degrees);
    PX4_INFO("Demo yaw gyro bias: +0.500 deg/s applied to BOTH");
    PX4_INFO("gyro samples %u | accel accepted %u/rejected %u | mag accepted %u/rejected %u",
             (unsigned)f.predictions, (unsigned)f.accel_updates,
             (unsigned)f.accel_rejected, (unsigned)f.mag_updates,
             (unsigned)f.mag_rejected);
    PX4_INFO("Magnetic reference: %s", f.mag_reference_ready ? "ready" : "waiting");

    // An informational reference only: NEVER read by the educational EKF.
    uORB::Subscription reference{ORB_ID(vehicle_attitude)};
    vehicle_attitude_s px4_att{};
    if (reference.copy(&px4_att)) {
        const lesson2::Q4 q{px4_att.q[0], px4_att.q[1], px4_att.q[2], px4_att.q[3]};
        const auto rpy = q.rpy();
        PX4_INFO("PX4  roll/pitch/yaw deg: %.2f / %.2f / %.2f (reference only)",
                 rpy.x*degrees, rpy.y*degrees, rpy.z*degrees);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// 3) PX4 standard ModuleBase boilerplate. Students do NOT need to edit this.
//    PX4 itself handles start, stop and status; no other commands are added.
// ---------------------------------------------------------------------------
int AttitudeLab::task_spawn(int /*argc*/, char *argv[])
{
    desc.task_id = px4_task_spawn_cmd("attitude_lab", SCHED_DEFAULT,
                   SCHED_PRIORITY_DEFAULT, 8192,
                   (px4_main_t)&run_trampoline, (char *const *)argv);
    if (desc.task_id < 0) { desc.task_id = -1; return -errno; }
    return 0;
}

int AttitudeLab::run_trampoline(int argc, char *argv[])
{
    return ModuleBase::run_trampoline_impl(desc,
        [](int ac, char *av[]) -> ModuleBase * { return AttitudeLab::instantiate(ac, av); },
        argc, argv);
}

AttitudeLab *AttitudeLab::instantiate(int, char *[]) { return new AttitudeLab(); }

int AttitudeLab::custom_command(int, char *[])
{
    return print_usage("only start, status, stop are supported");
}

int AttitudeLab::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s", reason);
    PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
# Lesson 2: observe gyro integration and a tiny attitude EKF in parallel.

Read vehicle_imu and vehicle_magnetometer and compute in the background.
Print estimates ONLY on `attitude_lab status`. No effect on flight control.
)DESCR_STR");
    PRINT_MODULE_USAGE_NAME("attitude_lab", "example");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); // PX4-provided stop and status
    return 0;
}

extern "C" __EXPORT int attitude_lab_main(int argc, char *argv[])
{
    return ModuleBase::main(AttitudeLab::desc, argc, argv);
}
