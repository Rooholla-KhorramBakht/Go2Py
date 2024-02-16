#
#   Copyright 2020 CNRS INRIA
#
#   Author: Guilhem Saurel
#

# Try to find cppadcg
# in standard prefixes and in ${cppadcg_PREFIX}
# Once done this will define
#  cppadcg_FOUND - System has cppadcg
#  cppadcg_INCLUDE_DIR - The cppadcg include directories
#  cppadcg_VERSION - Version of cppadcg found

FIND_PATH(cppadcg_INCLUDE_DIR
  NAMES cppad/cg.hpp
  PATHS ${cppadcg_PREFIX}
  PATH_SUFFIXES include
  )

IF(cppadcg_INCLUDE_DIR AND EXISTS "${cppadcg_INCLUDE_DIR}/cppad/cg/configure.hpp")
  file(STRINGS "${cppadcg_INCLUDE_DIR}/cppad/cg/configure.hpp" cppadcg_version_str
    REGEX "^#define[\t ]+CPPAD_CG_VERSION[\t ]+\"cppadcg-.*\"")
  string(REGEX REPLACE "^#define[\t ]+CPPAD_CG_VERSION[\t ]+\"cppadcg-([^\"]*)\".*" "\\1"
    cppadcg_VERSION "${cppadcg_version_str}")
ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(cppadcg REQUIRED_VARS cppadcg_INCLUDE_DIR VERSION_VAR cppadcg_VERSION)
mark_as_advanced(cppadcg_INCLUDE_DIR)
