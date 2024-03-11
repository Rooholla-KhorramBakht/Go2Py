//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

namespace pinocchio
{
  namespace python
  {
    static void computeAllTerms_proxy(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q,
                                      const Eigen::VectorXd & v)
    {
      data.M.fill(0);
      computeAllTerms(model,data,q,v);
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    
    void exposeCAT()
    {
      bp::def("computeAllTerms",computeAllTerms_proxy,
              bp::args("model","data","q","v"),
              "Compute all the terms M, non linear effects, center of mass quantities, centroidal quantities and Jacobians in"
              "in the same loop and store the results in data.\n"
              "This algorithm is equivalent to calling:\n"
              "\t- forwardKinematics\n"
              "\t- crba\n"
              "\t- nonLinearEffects\n"
              "\t- computeJointJacobians\n"
              "\t- centerOfMass\n"
              "\t- jacobianCenterOfMass\n"
              "\t- ccrba\n"
              "\t- computeKineticEnergy\n"
              "\t- computePotentialEnergy\n"
              "\t- computeGeneralizedGravity\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n"
              );
    }
  } // namespace python
} // namespace pinocchio
