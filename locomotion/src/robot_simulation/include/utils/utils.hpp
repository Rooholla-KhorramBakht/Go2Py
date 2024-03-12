#pragma once

#include <memory_types.hpp>
#include <iostream>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace pinocchio {
    inline mat3x3 skew_symm(const vec3 &v) {
        mat3x3 vx;
        vx << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;

        return vx;
    }

    inline mat3x3 Rx(float x)
    {
        mat3x3 R = mat3x3::Zero();

        R << 1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x);

        return R;
    }
    inline mat3x3 Ry(float x)
    {
        mat3x3 R = mat3x3::Zero();

        R << cos(x), 0, sin(x),
            0, 1, 0,
            -sin(x), 0, cos(x);

        return R;
    }
    inline mat3x3 Rz(float x)
    {
        mat3x3 R = mat3x3::Zero();

        R << cos(x), -sin(x), 0,
            sin(x), cos(x), 0,
            0, 0, 1;

        return R;
    }

    // Rewrite directly the analytical expression
    inline mat3x3 EulXYZ2Rot(const vec3 &eul)
    {
        // XYZ intrinsic euler angles, converts from base to global
        return Rx(eul(0)) * Ry(eul(1)) * Rz(eul(2));
    }

    inline mat3x3 quat2rot(const vec4 &q) {
        // pinocchio convention of {x, y, z, w}
        float q0 = q(3);
        vec3 qv = q.block<3, 1>(0, 0);
        mat3x3 Rot = (2 * q0 * q0 - 1) * mat3x3::Identity() + 2 * q0 * skew_symm(qv) + 2 * qv * qv.transpose();

        return Rot;
    }

    inline vec4 zeta(const vec3 &v) {
        float v_norm = v.norm();
        vec4 q = vec4(0, 0, 0, 1);
        q(3) = cos(v_norm / 2);
        if (v_norm == 0) {
            q.block<3, 1>(0, 0) = vec3::Zero(3);
        } else {
            q.block<3, 1>(0, 0) = v * sin(v_norm / 2) / v_norm;
        }
        return q;
    }

    inline vec4 Rot2AxisAngle(const mat3x3 &R) {
        vec4 axang = vec4::Zero();

        double tmp = (R.trace() - 1) / 2;
        axang.block<3, 1>(1, 0) = vec3(0, 0, 0);
        if (tmp >= 1.) {
            axang(0) = 0;
        } else if (tmp <= -1.) {
            axang(0) = M_PI;
        } else {
            axang(0) = std::acos(tmp);
            axang.block<3, 1>(1, 0) = (0.5 / sin(axang(0))) * vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        }

        return axang;
    }

    inline vec3 Rot2Phi(const mat3x3 &R) {
        vec4 axang = Rot2AxisAngle(R);

        return axang(0) * axang.block<3, 1>(1, 0);
    }

    inline vec4 EulXYZ2quat(const vec3& eul) {
        return zeta(Rot2Phi(EulXYZ2Rot(eul)));
    }

    inline vec3 Rot2EulXYZ(const mat3x3& R) {
        // reference: https://www.geometrictools.com/Documentation/EulerAngles.pdf : Page 4-5
        vec3 eul(0, 0, 0);
        if (R(0, 2) < 1) {
            if (R(0, 2) > -1) {
                eul(0) = atan2(-R(1, 2), R(2, 2));
                eul(1) = asin(R(0, 2));
                eul(2) = atan2(-R(0, 1), R(0, 0));
            } else {
                eul(0) = -atan2(R(1, 0), R(1, 1));
                eul(1) = -M_PI / 2;
                eul(2) = 0;
            }
        } else {
            eul(0) = atan2(R(1, 0), R(1, 1));
            eul(1) = M_PI / 2;
            eul(2) = 0;
        }

        return eul;
    }
    inline vec3 matrixLogRot(const mat3x3 &R) {
        double theta;

        double tmp = (R.trace() - 1) / 2;
        if (tmp >= 1.) {
            theta = 0;
        } else if (tmp <= -1.) {
            theta = M_PI;
        } else {
            theta = std::acos(tmp);
        }

        vec3 omega = vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

        if (theta > 1e-4) {
            omega *= theta / (2 * sin(theta));
        }
        else {
            omega /= 2;
        }

        return omega;
    }
}