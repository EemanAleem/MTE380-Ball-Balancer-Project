#include "ik.hpp"

/* All Measurements [cm unless otherwise specified] */
static constexpr Vec3 T = {0.0, 0.0, 11.0}; // z height from motor shaft to plate
static constexpr double L1 = 8;             // length of first link
static constexpr double L2 = 11;            // length of second link
static constexpr double R_L = 14.5;         // distance from center of plate to center of end effector joint

/* Vectors from center of plate to center of end effector joint for legs A, B & C. */
static Vec3 PA_D = {0.0, R_L, 0.0};
static Vec3 PB_D = {R_L * cos(210*DEG_TO_RAD), R_L * sin(210*DEG_TO_RAD), 0.0};
static Vec3 PC_D = {R_L * cos(330*DEG_TO_RAD), R_L * sin(330*DEG_TO_RAD), 0.0};

/* Vectors from center of base to center of motors A, B & C. */
static Vec3 BASE_A = {0.0, 6.0, 0.0};
static Vec3 BASE_B = {5.5*cos(210*DEG_TO_RAD), 5.5*sin(210*DEG_TO_RAD), 0.0};
static Vec3 BASE_C = {6.7*cos(330*DEG_TO_RAD), 6.7*sin(330*DEG_TO_RAD), 0.0};

/* Flips Axes Rotation Signs depending on Camera View */
static constexpr double SIGN_X = 1.0;  // Flip X axis rotation sign. Default =  1.0.
static constexpr double SIGN_Y = -1.0; // Flip Y axis rotation sign  Default = -1.0.

/* Angle Limits */
static constexpr double MIN_MOTOR_ANGLE = 0.0;  // Minimum angle for the legs
static constexpr double MAX_MOTOR_ANGLE = 50.0; // Maximum angle for the legs
static constexpr double MAX_TILT_ANGLE = 10.0; // Maximum tilt angle for the plate

Vec3 calculateNormalFromDirVec(const Vec3 &uVec) {
    // 1.0 Guards against control signal vector, uVec
    if (uVec[0] == 0.0 && uVec[1] == 0.0) return UNIT_K;

    // 1.1 Calculate magnitude of direction vector, if 0, plate should point straight up (UNIT_K)
    double mag = std::hypot(uVec[0], uVec[1]);
    mag *= DEG_TO_RAD; // Convert to radians
    if (mag == 0.0) return UNIT_K;
    if (mag > MAX_TILT_ANGLE) mag = MAX_TILT_ANGLE * DEG_TO_RAD;

    // 1.2 Calculate the direction vector in the x-y plane, scaled to unit length
    const Vec3 dirVec = {uVec[0]/mag, uVec[1]/mag, 0.0};

    // 2. Axis of rotation: orthogonal to dirVec, in the x-y plane
    const Vec3 rotAxis = { (SIGN_Y*dirVec[1]), (SIGN_X*dirVec[0]), 0.0 };
    const Vec3 rotAxisNorm = scale(rotAxis, 1.0 / std::hypot(rotAxis[0], rotAxis[1]));

    // 3. Extend the direction vector to 3D
    const Vec3 dir3 = {dirVec[0], dirVec[1], 0.0};

    // 4. Rodrigues rotation of the direction vector about the axis by `mag`
    const Vec3 dirRot = add(
        add(scale(dir3, std::cos(mag)),
            scale(cross(rotAxisNorm, dir3), std::sin(mag))),
        scale(rotAxisNorm, dot(rotAxisNorm, dir3) * (1.0 - std::cos(mag))));

    // 5. Normalize the rotated direction vector
    const Vec3 dirRotNorm = scale(dirRot, 1.0 / std::hypot(dirRot[0], dirRot[1], dirRot[2]));

    Vec3 N = cross(rotAxisNorm, dirRotNorm);

    // 6. Always point in +Z
    if (N[2] < 0.0) {
        N = scale(N, -1.0);
    }

    return N;
}

Vec3 calculateMotorAngles(const Vec3 &uVec) {
    // 1. Calculate the normal vector from the direction vector.
    const Vec3 N = calculateNormalFromDirVec(uVec);

    // 2. Calculate the rotation matrix from the normal vector.
    const Mat3 R = RYRX(N);

    // 3. Calculate the desired rotated body vectors of Legs A, B & C.
    Vec3 pa = matVec(R, PA_D);
    Vec3 pb = matVec(R, PB_D);
    Vec3 pc = matVec(R, PC_D);

    // 4. Calculate the overall vector and magnitudes from motor shaft
    //    centre to end effector joint centre for Legs A, B & C.
    Vec3 la = {T[0] + pa[0] - BASE_A[0],   T[1] + pa[1] - BASE_A[1],   T[2] + pa[2] - BASE_A[2]};
    Vec3 lb = {T[0] + pb[0] - BASE_B[0],   T[1] + pb[1] - BASE_B[1],   T[2] + pb[2] - BASE_B[2]};
    Vec3 lc = {T[0] + pc[0] - BASE_C[0],   T[1] + pc[1] - BASE_C[1],   T[2] + pc[2] - BASE_C[2]};
    double la_mag = std::hypot(la[0], la[1], la[2]);
    double lb_mag = std::hypot(lb[0], lb[1], lb[2]);
    double lc_mag = std::hypot(lc[0], lc[1], lc[2]);

    // 5. Calculate the alpha angles for Legs A, B & C.
    double alpha_a = safeAcos( (la_mag*la_mag + L1*L1 - L2*L2)/(2.0*la_mag*L1) );
    double alpha_b = safeAcos( (lb_mag*lb_mag + L1*L1 - L2*L2)/(2.0*lb_mag*L1) );
    double alpha_c = safeAcos( (lc_mag*lc_mag + L1*L1 - L2*L2)/(2.0*lc_mag*L1) );

    // 6. Calculate the beta angles for Legs A, B & C.
    double beta_a = safeAcos(dot(UNIT_K, la)/la_mag);
    double beta_b = safeAcos(dot(UNIT_K, lb)/lb_mag);
    double beta_c = safeAcos(dot(UNIT_K, lc)/lc_mag);

    // 7. Calculate the final motor angles for Legs A, B & C.
    double gamma_a = (PI/2 - alpha_a - beta_a) * RAD_TO_DEG;
    double gamma_b = (PI/2 - alpha_b - beta_b) * RAD_TO_DEG;
    double gamma_c = (PI/2 - alpha_c - beta_c) * RAD_TO_DEG;

    // 8. Clamp gamma angles to the range [MIN_MOTOR_ANGLE and MAX_MOTOR_ANGLE] 
    //    to prevent overextension of the legs.
    gamma_a = std::clamp(gamma_a, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
    gamma_b = std::clamp(gamma_b, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
    gamma_c = std::clamp(gamma_c, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);

    return {gamma_a, gamma_b, gamma_c};
}