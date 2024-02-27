//
// Copyright (c) 2021 INRIA
//

#include <omp.h>

#include "pinocchio/bindings/python/fwd.hpp"

namespace pinocchio
{
  namespace python
  {
  
    void exposeParallelRNEA();
    void exposeParallelABA();
    void exposeParallelGeometry();
    
    void exposeParallelAlgorithms()
    {
      namespace bp = boost::python;
      
      exposeParallelRNEA();
      exposeParallelABA();
      
#ifdef PINOCCHIO_WITH_HPP_FCL
      exposeParallelGeometry();
#endif
      
      bp::def("omp_get_max_threads",&omp_get_max_threads,
              "Returns an upper bound on the number of threads that could be used.");
    }
    
  } // namespace python
} // namespace pinocchio
