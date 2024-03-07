#
# Copyright 2019 CNRS
#
# Author: Guilhem Saurel
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License along
# with this program.  If not, see <https://www.gnu.org/licenses/>.
#

# Try to find CLP in standard prefixes and in ${CLP_PREFIX} Once done this will
# define CLP_FOUND - System has CLP CLP_INCLUDE_DIRS - The CLP include
# directories CLP_LIBRARIES - The libraries needed to use CLP CLP_DEFINITIONS -
# Compiler switches required for using CLP

find_path(
  CLP_INCLUDE_DIR
  NAMES coin/ClpSimplex.hpp
  PATHS ${CLP_PREFIX})
find_library(
  CLP_LIBRARY
  NAMES libclp.so libClp.so
  PATHS ${CLP_PREFIX})

set(CLP_LIBRARIES ${CLP_LIBRARY})
set(CLP_INCLUDE_DIRS ${CLP_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CLP DEFAULT_MSG CLP_LIBRARY CLP_INCLUDE_DIR)
mark_as_advanced(CLP_INCLUDE_DIR CLP_LIBRARY)

if(CLP_FOUND AND NOT TARGET CLP::CLP)
  add_library(CLP::CLP SHARED IMPORTED)
  set_target_properties(CLP::CLP PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                            "${CLP_INCLUDE_DIR}")
  set_target_properties(CLP::CLP PROPERTIES IMPORTED_LOCATION_RELEASE
                                            "${CLP_LIBRARY}")
  set_property(
    TARGET CLP::CLP
    APPEND
    PROPERTY IMPORTED_CONFIGURATIONS "RELEASE")
  message(STATUS "CLP found (include: ${CLP_INCLUDE_DIR}, lib: ${CLP_LIBRARY})")
endif()
