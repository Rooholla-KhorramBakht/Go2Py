#pragma once

#include <memory_types.hpp>

namespace pinocchio {
    inline mat3x3 skew_symm(const vec3 &v) {
        mat3x3 vx;
        vx << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;

        return vx;
    }

    inline mat3x3 quat2rot(const vec4 &q) {
        // pinocchio convention of {x, y, z, w}
        float q0 = q(3);
        vec3 qv = q.block<3, 1>(0, 0);
        mat3x3 Rot = (2 * q0 * q0 - 1) * mat3x3::Identity() + 2 * q0 * skew_symm(qv) + 2 * qv * qv.transpose();

        return Rot;
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