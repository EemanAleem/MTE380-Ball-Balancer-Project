#include "math.hpp"

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

inline double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline double norm(const Vec3& a) {
    return sqrt(dot(a, a));
}

inline Vec3 scale(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}

inline Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline Vec3 matVec(const Mat3& m, const Vec3& v) {
    return {m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
}

inline Mat3 RYRX(const Vec3 &N) {
    double nx = N[0];
    double ny = N[1];

    double theta = asin(nx/(sqrt(1.0 - ny*ny)));
    double phi = asin(-1*ny);

    double st = sin(theta);
    double ct = cos(theta);
    double sp = sin(phi);
    double cp = cos(phi);

    Mat3 R = {{
        {ct, st*sp, st*cp},
        {0.0, cp, -1.0*sp},
        {-1.0*st, ct*sp, ct*cp}
    }};

    return R;
}
