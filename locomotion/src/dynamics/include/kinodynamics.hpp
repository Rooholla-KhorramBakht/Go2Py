#pragma once

#include <string>

#include <utils.hpp>
#include "timer.hpp"


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/algorithm/model.hpp>
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"



class Quadruped
{
public:
    Quadruped();
    Quadruped(std::string urdf_path);

    ~Quadruped() {}

    vec12 forwardKinematics(const vec19 & q);

    vec12 inverseKinematics(const vec19 & x, SOLVER_TYPE solver=SOLVER_TYPE::ANALYTICAL);

    vec18 getJointVelocity(const vec19 & q, const vec18 & xd);

    mat18x18 feetJacobian(const vec19 & q);

    mat18x18 feetJacobianDot(const vec19 & q, const vec18 & v);

    mat18x18 massMatrix(const vec19 & q);

    mat18x18 massMatrixInverse(const vec18 & q);

    vec18 coriolisVector(const vec19 & q, const vec18 & v);

    vec18 nonLinearEffects(const vec19 & q, const vec18 & v);

    vec18 gravitationalTerms(const vec19 & q);

    mat18x18 coriolisMatrix(const vec19 & q, const vec18 & v);

    void setStanceHeight(const float& h);

    vec19 getSleepStates();

    vec19 getStanceStates();

    vec19 getNeutralJointStates();

    vec12 getHAAPosition(const vec19 & q);

    vec12 getHFEPosition(const vec19& q);

    // vec18 forwardDynamics(const vec19 & q, const vec18 & v, const vec12 & tau);

    // Eigen::VectorXd inverseDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & a);

    // void updatePinocchioModel(pinocchio::Model& model,
    //                         pinocchio::Data& data,
    //                         const vec19& q, 
    //                         const vec18& dq) {
    //     // update pinocchio robot model
    //     pinocchio::forwardKinematics(model, data, q, dq);
    //     pinocchio::computeJointJacobians(model, data, q); 
    //     pinocchio::updateFramePlacements(model, data);   
    //     pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
    // }

    void showParams() {
        std::cout << "h: " << h << "\n";
        std::cout << "b: " << b << "\n";
        std::cout << "l1: " << l1 << "\n";
        std::cout << "l2: " << l2 << "\n";
        std::cout << "l3: " << l3 << "\n";
        return;
    }

    std::string getURDFFilepath() {
        return urdf_filepath;
    }

private:
    void InitClass();

    std::string urdf_filepath;

    pinocchio::Model model;
    pinocchio::Data data;

    vec12 inverseKinematicsIterative(const vec19& x);
    vec12 inverseKinematicsAnalytical(const vec19& x);

    vec19 q_current, q_neutral, q_sleep, q_stance, x_sleep, x_stance;

    int ID_FOOT_FRAME[4], ID_FOOT_JOINT[4], ID_BASE;
    int ID_HAA[4], ID_FR_HAA, ID_FL_HAA, ID_RL_HAA, ID_RR_HAA;
    int ID_HFE[4], ID_FR_HFE, ID_FL_HFE, ID_RL_HFE, ID_RR_HFE;
    int ID_KFE[4], ID_FR_KFE, ID_FL_KFE, ID_RL_KFE, ID_RR_KFE;

    // meta-vars for iterative IK
    double IK_ERR_TOL = 1e-3;
    double DAMP = 1e-5;
    int MAX_ITERS_IK = 100;
    int iters = 0;

    double h = 0.4, b = 0.12;
    double l1 = 0.065, l2 = 0.20, l3 = 0.20;

    int sx[4] = {1, 1, -1, -1};
    int sy[4] = {1, -1, 1, -1};
};