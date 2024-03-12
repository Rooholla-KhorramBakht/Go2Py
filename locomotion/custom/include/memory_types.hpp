#pragma once

#include <eigen3/Eigen/Dense>

enum SOLVER_TYPE {
    ITERATIVE,
    ANALYTICAL
};

typedef Eigen::Matrix<int, 4, 1> int4;
typedef Eigen::Matrix<double, 3, 1> vec3;
typedef Eigen::Matrix<double, 4, 1> vec4;
typedef Eigen::Matrix<double, 6, 1> vec6;
typedef Eigen::Matrix<double, 7, 1> vec7;
typedef Eigen::Matrix<double, 12, 1> vec12;
typedef Eigen::Matrix<double, 18, 1> vec18;
typedef Eigen::Matrix<double, 19, 1> vec19;
typedef Eigen::Matrix<double, 3, 3> mat3x3;
typedef Eigen::Matrix<double, 4, 4> mat4x4;
typedef Eigen::Matrix<double, 6, 6> mat6x6;
typedef Eigen::Matrix<double, 8, 8> mat8x8;
typedef Eigen::Matrix<double, 12, 12> mat12x12;
typedef Eigen::Matrix<double, 18, 18> mat18x18;
typedef Eigen::Matrix<double, 6, 12> mat6x12;
typedef Eigen::Matrix<double, 12, 6> mat12x6;
typedef Eigen::Matrix<double, 6, 18> mat6x18;
typedef Eigen::Matrix<double, 18, 6> mat18x6;

typedef struct QuadrupedSensorData {
    vec12 q, qd;
    vec12 tau;
    vec4 quat;
    vec3 a_B, w_B;
    vec3 a_W, w_W;
    QuadrupedSensorData() {
        q = vec12::Zero();
        qd = vec12::Zero();
        tau = vec12::Zero();
        quat = vec4(0, 0, 0, 1);
        a_B = vec3::Zero();
        w_B = vec3::Zero();
        a_W = vec3::Zero();
        w_W = vec3::Zero();
    }

    void copy(const QuadrupedSensorData& sd) {
        this->q = sd.q;
        this->qd = sd.qd;
        this->tau = sd.tau;
        this->quat = sd.quat;
        this->a_B = sd.a_B;
        this->w_B = sd.w_B;
        this->a_W = sd.a_W;
        this->w_W = sd.w_W;
    }
} QuadrupedSensorData;

typedef struct QuadrupedEstimationData {
    vec3 rB;
    vec3 vB;
    vec12 rP;
    int4 cs;
    vec4 pc;
    vec19 js;
    vec18 jv;
    vec18 ja;
    QuadrupedEstimationData() {
        rB = vec3::Zero();
        vB = vec3::Zero();
        rP = vec12::Zero();
        cs = int4::Zero();
        pc = vec4::Zero();
        js = vec19::Zero();
        jv = vec18::Zero();
        ja = vec18::Zero();
    }

    void copy(const QuadrupedEstimationData& est) {
        this->rB = est.rB;
        this->vB = est.vB;
        this->rP = est.rP;
        this->cs = est.cs;
        this->pc = est.pc;
        this->js = est.js;
        this->jv = est.jv;
        this->ja = est.ja;
    }
} QuadrupedEstimationData;

typedef struct QuadrupedCommandData {
    vec12 q, qd, tau, kp, kd;
    QuadrupedCommandData() {
        q = vec12::Zero();
        qd = vec12::Zero();
        tau = vec12::Zero();
        kp = vec12::Zero();
        kd = vec12::Zero();
    }
    void copy(const QuadrupedCommandData& cmd) {
        this->q = cmd.q;
        this->qd = cmd.qd;
        this->tau = cmd.tau;
        this->kp = cmd.kp;
        this->kd = cmd.kd;
    }
} QuadrupedCommandData;

typedef struct QuadrupedPlannerData {
    vec19 x;
    vec18 xd, xdd;
    int4 cs_ref;
    vec4 pc_ref;
    QuadrupedPlannerData() {
        x = vec19::Zero();
        xd = vec18::Zero();
        xdd = vec18::Zero();
        cs_ref = int4::Zero();
        pc_ref = vec4::Zero();
    }
} QuadrupedPlannerData;

typedef struct QuadrupedMeasurementData {
    vec3 base_position;
    // A better name might be PluckerVector?
    struct TotalVelocity {
        vec3 linear;
        vec3 angular;
        TotalVelocity() {
            linear = vec3::Zero();
            angular = vec3::Zero();
        }
    };
    TotalVelocity base_velocity;
    TotalVelocity base_acceleration;
    vec12 contact_force;
    vec12 estimated_contact_force;
    QuadrupedMeasurementData() {
        base_position = vec3::Zero();
        base_velocity = TotalVelocity();
        base_acceleration = TotalVelocity();
        contact_force = vec12::Zero();
        estimated_contact_force = vec12::Zero();
    }
    void copy(const QuadrupedMeasurementData& m) {
        this->base_position = m.base_position;
        this->base_velocity.linear = m.base_velocity.linear;
        this->base_velocity.angular = m.base_velocity.angular;
        this->base_acceleration.linear = m.base_acceleration.linear;
        this->base_acceleration.angular = m.base_acceleration.angular;
        this->contact_force = m.contact_force;
        this->estimated_contact_force = m.estimated_contact_force;
    }
} QuadrupedMeasurementData;

typedef struct QuadrupedJoystickData {
    double vel_x = 0;
    double vel_y = 0;
    double vel_yaw = 0;
    int mode = 0; // 0: sleepToStance, 1: stanceToSleep(), 2: move
    int gait = 0; // 0: stance, 1: trot
    void copy(const QuadrupedJoystickData& jd) {
        this->vel_x = jd.vel_x;
        this->vel_y = jd.vel_y;
        this->vel_yaw = jd.vel_yaw;
        this->mode = jd.mode;
        this->gait = jd.gait;
    }
} QuadrupedJoystickData;