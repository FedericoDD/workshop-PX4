#pragma once
/*
 * LESSON 2: pure C++ mathematics (no PX4 dependencies).
 *
 * Two estimators see EXACTLY the same gyro:
 *   1. gyro_only: q = q * delta_q(gyro * dt); no corrections, no bias estimate.
 *   2. EKF: gyro prediction + accelerometer and magnetic vector corrections.
 *
 * EKF STATE: nominal quaternion q (BODY/FRD -> NED), gyro bias b [rad/s].
 * EKF ERROR: dx = [small body-frame attitude error (3), bias error (3)].
 * COVARIANCE: P (6x6). This is an error-state EKF, not
 * a complementary filter called an EKF. See README.md for equations.
 *
 * Simplifications for class: small error linearization, diagonal Q and R,
 * no time-delay compensation; magnetic reference is initialized at start.
 * IMPORTANT: this is an educational SITL demo, not flight-ready software.
 */

#include <cmath>
#include <cstdint>

namespace lesson2 {

constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979323846;

struct V3 {
    double x{0}, y{0}, z{0};
    V3() = default;
    V3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
    V3 operator+(V3 a) const { return {x+a.x, y+a.y, z+a.z}; }
    V3 operator-(V3 a) const { return {x-a.x, y-a.y, z-a.z}; }
    V3 operator*(double a) const { return {x*a, y*a, z*a}; }
    V3 operator/(double a) const { return {x/a, y/a, z/a}; }
    double norm() const { return std::sqrt(x*x+y*y+z*z); }
    V3 unit() const { const double n = norm(); return n > 1.e-10 ? *this/n : V3{}; }
};
inline V3 cross(V3 a, V3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
inline bool finite(V3 a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }

struct Q4 {
    // Hamilton quaternion; q rotates BODY vectors into NED.
    double w{1}, x{0}, y{0}, z{0};
    Q4 operator*(Q4 b) const {
        return {w*b.w-x*b.x-y*b.y-z*b.z,
                w*b.x+x*b.w+y*b.z-z*b.y,
                w*b.y-x*b.z+y*b.w+z*b.x,
                w*b.z+x*b.y-y*b.x+z*b.w};
    }
    void normalize() {
        const double n = std::sqrt(w*w+x*x+y*y+z*z);
        if (n > 1.e-12) { w/=n; x/=n; y/=n; z/=n; }
        else { *this = {}; }
    }
    // Small gyro increment, exact Rodrigues/exponential quaternion.
    static Q4 delta(V3 dtheta) {
        const double angle = dtheta.norm();
        if (angle < 1.e-9) return {1, dtheta.x/2, dtheta.y/2, dtheta.z/2};
        const double s = std::sin(angle/2)/angle;
        return {std::cos(angle/2), s*dtheta.x, s*dtheta.y, s*dtheta.z};
    }
    V3 rotate(V3 v) const {
        V3 t = cross({x,y,z},v)*2.0;
        return v + t*w + cross({x,y,z}, t);
    }
    V3 inverse_rotate(V3 v) const {
        V3 t = cross({-x,-y,-z},v)*2.0;
        return v + t*w + cross({-x,-y,-z},t);
    }
    // Roll-pitch-yaw (ZYX, radians). Only used for printing status.
    V3 rpy() const {
        return {std::atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                std::asin(std::fmax(-1.,std::fmin(1.,2*(w*y-z*x)))),
                std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))};
    }
};

class AttitudeFilter {
public:
    Q4 gyro_only;          // A: prediction only, always running
    Q4 ekf;                // B: prediction + selected aiding
    V3 bias;              // b_hat: estimated gyro bias [rad/s]
    // One visible, fixed demo disturbance. Both estimators see the SAME biased
    // gyro. Without this, an ideal SITL gyro may show almost no drift.
    // Set to 0.0 to disable the class experiment.
    static constexpr double kDemoYawBiasRadS = 0.5*kPi/180.0; // 0.5 deg/s
    uint32_t predictions{0}, accel_updates{0}, mag_updates{0};
    uint32_t accel_rejected{0}, mag_rejected{0};
    bool mag_reference_ready{false};
    double last_accel_innovation{0}, last_mag_innovation{0};

    AttitudeFilter() {
        // Starting attitude: level, pointing along the arbitrary NED X of SITL.
        // Start BEFORE takeoff. A production filter would initialize from sensors.
        for (int i=0; i<6; ++i) P[i][i] = (i<3 ? 0.15*0.15 : 0.04*0.04);
    }

    // PREDICT: one IMU sample gives an integrated delta-angle (rad) and dt (s).
    void predict(V3 dangle, double dt) {
        if (!finite(dangle) || !std::isfinite(dt) || dt <= 0 || dt > 0.05) return;
        dangle.z += kDemoYawBiasRadS*dt; // known injected yaw bias: same for both
        gyro_only = gyro_only * Q4::delta(dangle);
        gyro_only.normalize();

        const V3 rate = dangle/dt - bias;   // omega_hat = omega_gyro - b_hat
        ekf = ekf * Q4::delta(rate*dt);    // quaternion kinematics
        ekf.normalize();

        // P^- = F P F' + Q. Right-multiplicative attitude error:
        // F = [ I - skew(omega)*dt   -I*dt ]
        //     [          0             I  ]
        double F[6][6]{};
        for (int i=0; i<6; ++i) F[i][i]=1;
        const double s[3][3]={{0,-rate.z,rate.y},{rate.z,0,-rate.x},{-rate.y,rate.x,0}};
        for (int i=0; i<3; ++i) {
            for (int j=0; j<3; ++j) F[i][j]-=s[i][j]*dt;
            F[i][i+3]=-dt;
        }
        double A[6][6]{}, N[6][6]{};
        for (int i=0; i<6; ++i) for (int j=0; j<6; ++j)
            for (int k=0; k<6; ++k) A[i][j]+=F[i][k]*P[k][j];
        for (int i=0; i<6; ++i) for (int j=0; j<6; ++j)
            for (int k=0; k<6; ++k) N[i][j]+=A[i][k]*F[j][k];
        // Educational illustrative process-noise spectral densities.
        for (int i=0; i<3; ++i) {
            N[i][i] += (0.02*0.02)*dt;     // gyro angle random walk
            N[i+3][i+3] += (0.002*0.002)*dt; // slowly changing bias
        }
        copy6(P,N);
        ++predictions;
    }

    // CORRECT TILT: an accelerometer measures SPECIFIC FORCE, not gravity.
    // On ground / quasi-static flight: f_B ~ - R(q)' * [0,0,+g].
    // We normalize, so the expected measurement is q^-1*[0,0,-1].
    void correct_accel(V3 specific_force) {
        const double n=specific_force.norm();
        // Reject strong non-gravitational accelerations; a first, imperfect gate.
        if (!finite(specific_force) || n<0.85*kG || n>1.15*kG) { ++accel_rejected; return; }
        const V3 z=specific_force/n;
        const V3 h=ekf.inverse_rotate({0,0,-1});
        last_accel_innovation=(z-h).norm();
        if (last_accel_innovation>0.7) { ++accel_rejected; return; }
        update_vector(z,h,0.07);      // R = (normalized accel std)^2 I
        ++accel_updates;
    }

    // CORRECT HEADING: compare magnetic field in BODY with one reference in NED.
    // The reference is LEARNED from the first measurement, assuming we start
    // near level, before takeoff. Hence yaw is RELATIVE to the initial heading.
    // This does not claim geographic north or magnetic declination correction.
    void correct_mag(V3 magnetic_field) {
        const double n=magnetic_field.norm();
        if (!finite(magnetic_field) || n<0.05 || n>2.0) { ++mag_rejected; return; }
        const V3 z=magnetic_field/n;
        if (!mag_reference_ready) {
            mag_nav_ref=ekf.rotate(z).unit();
            // A near-vertical magnetic field cannot constrain yaw well.
            if (std::sqrt(mag_nav_ref.x*mag_nav_ref.x+mag_nav_ref.y*mag_nav_ref.y)<0.2) {
                ++mag_rejected; return;
            }
            mag_reference_ready=true;
        }
        const V3 h=ekf.inverse_rotate(mag_nav_ref);
        last_mag_innovation=(z-h).norm();
        if (last_mag_innovation>0.7) { ++mag_rejected; return; }
        update_vector(z,h,0.08);       // R = (normalized mag std)^2 I
        ++mag_updates;
    }

    double attitude_variance() const { return P[0][0]+P[1][1]+P[2][2]; }
    double bias_z_std() const { return std::sqrt(std::fmax(0.0,P[5][5])); }

private:
    double P[6][6]{};
    V3 mag_nav_ref{};
    static void copy6(double dst[6][6], const double src[6][6]) {
        for (int i=0;i<6;++i) for (int j=0;j<6;++j) dst[i][j]=src[i][j];
    }

    // A REAL EKF CORRECTION:
    // r=z-h(q); H=[skew(h)  0]; S=H P H' + R;
    // K=P H' S^-1; dx=K r; inject into quaternion and bias.
    void update_vector(V3 z, V3 h, double sigma) {
        const double hv[3]={h.x,h.y,h.z};
        double H[3][6]{};
        const double sk[3][3]={{0,-hv[2],hv[1]}, {hv[2],0,-hv[0]}, {-hv[1],hv[0],0}};
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) H[i][j]=sk[i][j];
        double PHt[6][3]{}, S[3][3]{};
        for (int i=0;i<6;++i) for (int j=0;j<3;++j)
            for (int k=0;k<6;++k) PHt[i][j]+=P[i][k]*H[j][k];
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
            for (int k=0;k<6;++k) S[i][j]+=H[i][k]*PHt[k][j];
            if (i==j) S[i][j]+=sigma*sigma;
        }
        // Gauss-Jordan 3x3 inversion: pedagogical alternative to matrix library.
        double aug[3][6]{};
        for (int i=0;i<3;++i) {
            for (int j=0;j<3;++j) aug[i][j]=S[i][j];
            aug[i][i+3]=1;
        }
        for (int i=0;i<3;++i) {
            int pivot=i;
            for (int r=i+1;r<3;++r) if (std::fabs(aug[r][i])>std::fabs(aug[pivot][i])) pivot=r;
            if (std::fabs(aug[pivot][i])<1.e-12) return;
            for (int j=0;j<6;++j) { const double a=aug[i][j]; aug[i][j]=aug[pivot][j]; aug[pivot][j]=a; }
            const double d=aug[i][i];
            for (int j=0;j<6;++j) aug[i][j]/=d;
            for (int r=0;r<3;++r) if (r!=i) {
                const double a=aug[r][i];
                for (int j=0;j<6;++j) aug[r][j]-=a*aug[i][j];
            }
        }
        double K[6][3]{};
        for (int i=0;i<6;++i) for (int j=0;j<3;++j)
            for (int k=0;k<3;++k) K[i][j]+=PHt[i][k]*aug[k][j+3];
        const double r[3]={z.x-h.x,z.y-h.y,z.z-h.z};
        double dx[6]{};
        for (int i=0;i<6;++i) for (int j=0;j<3;++j) dx[i]+=K[i][j]*r[j];

        // Joseph covariance update keeps P symmetric/positive in finite precision.
        double M[6][6]{}, MP[6][6]{}, N[6][6]{};
        for (int i=0;i<6;++i) {
            M[i][i]=1;
            for (int j=0;j<6;++j) for (int k=0;k<3;++k) M[i][j]-=K[i][k]*H[k][j];
        }
        for (int i=0;i<6;++i) for (int j=0;j<6;++j)
            for (int k=0;k<6;++k) MP[i][j]+=M[i][k]*P[k][j];
        for (int i=0;i<6;++i) for (int j=0;j<6;++j) {
            for (int k=0;k<6;++k) N[i][j]+=MP[i][k]*M[j][k];
            for (int k=0;k<3;++k) N[i][j]+=sigma*sigma*K[i][k]*K[j][k];
        }

        // Inject error into nominal quaternion and bias; error mean resets to 0.
        V3 dtheta{dx[0],dx[1],dx[2]};
        const double angle=dtheta.norm();
        if (angle>0.25) dtheta=dtheta*(0.25/angle); // keep local approx valid
        ekf=ekf*Q4::delta(dtheta);
        ekf.normalize();
        bias=bias+V3{dx[3],dx[4],dx[5]};

        // First-order attitude-error reset Jacobian: G=I-0.5*skew(dtheta).
        const double v[3]={dtheta.x,dtheta.y,dtheta.z};
        const double reset_sk[3][3]={{0,-v[2],v[1]}, {v[2],0,-v[0]}, {-v[1],v[0],0}};
        double G[6][6]{}, GN[6][6]{}, Rst[6][6]{};
        for (int i=0;i<6;++i) G[i][i]=1;
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) G[i][j]-=0.5*reset_sk[i][j];
        for (int i=0;i<6;++i) for (int j=0;j<6;++j)
            for (int k=0;k<6;++k) GN[i][j]+=G[i][k]*N[k][j];
        for (int i=0;i<6;++i) for (int j=0;j<6;++j)
            for (int k=0;k<6;++k) Rst[i][j]+=GN[i][k]*G[j][k];
        for (int i=0;i<6;++i) for (int j=0;j<6;++j)
            P[i][j]=0.5*(Rst[i][j]+Rst[j][i]);
    }
};

} // namespace lesson2
