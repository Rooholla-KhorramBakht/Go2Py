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

# Try to find libcdd in standard prefixes and in ${CDD_PREFIX} Once done this
# will define CDD_FOUND - System has CDD CDD_INCLUDE_DIRS - The CDD include
# directories CDD_LIBRARIES - The libraries needed to use CDD CDD_DEFINITIONS -
# Compiler switches required for using CDD

find_path(
  CDD_INCLUDE_DIR
  NAMES cdd.h cddmp.h
  PATHS ${CDD_PREFIX}
  PATH_SUFFIXES include/cdd include/cddlib)
find_library(
  CDD_LIBRARY
  NAMES libcdd.so
  PATHS ${CDD_PREFIX})

set(CDD_LIBRARIES ${CDD_LIBRARY})
set(CDD_INCLUDE_DIRS ${CDD_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CDD DEFAULT_MSG CDD_LIBRARY CDD_INCLUDE_DIR)
mark_as_advanced(CDD_INCLUDE_DIR CDD_LIBRARY)

if(CDD_FOUND AND NOT TARGET CDD::CDD)
  add_library(CDD::CDD SHARED IMPORTED)
  set_target_properties(CDD::CDD PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                            "${CDD_INCLUDE_DIR}")
  set_target_properties(CDD::CDD PROPERTIES IMPORTED_LOCATION_RELEASE
                                            "${CDD_LIBRARY}")
  set_property(
    TARGET CDD::CDD
    APPEND
    PROPERTY IMPORTED_CONFIGURATIONS "RELEASE")
  message(STATUS "CDD found (include: ${CDD_INCLUDE_DIR}, lib: ${CDD_LIBRARY})")
endif()
