# Copyright (C) 2016-2022  LAAS-CNRS, JRL AIST-CNRS and others.
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.

# _SETUP_PROJECT_PACKAGE_INIT
# -------------
#
# Initialize the PackageConfig installation configuration.
# https://github.com/forexample/package-example
#
# This function does not take any argument but check that some variables are
# defined (see documentation at the beginning of this file).
#
macro(_SETUP_PROJECT_PACKAGE_INIT)
  #
  # Installation (https://github.com/forexample/package-example)

  # Layout. This works for all platforms: * <prefix>/lib/cmake/<PROJECT-NAME> *
  # <prefix>/lib/ * <prefix>/include/
  set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
  set(INCLUDE_INSTALL_DIR "include")
  set(INCLUDE_INSTALL_DESTINATION "${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}")

  set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

  # Configuration
  set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
  set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
  set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
  set(namespace "${PROJECT_NAME}::")

  set(_PACKAGE_CONFIG_DEPENDENCIES_PROJECTS
      ""
      CACHE INTERNAL "")
  set(_PACKAGE_CONFIG_DEPENDENCIES_FIND_PACKAGE
      ""
      CACHE INTERNAL "")
  set(_PACKAGE_CONFIG_DEPENDENCIES_FIND_DEPENDENCY
      ""
      CACHE INTERNAL "")
  set(_PACKAGE_CONFIG_DEPENDENCIES_FIND_EXTERNAL
      ""
      CACHE INTERNAL "")
  set(PACKAGE_EXTRA_MACROS
      ""
      CACHE INTERNAL "")
endmacro(_SETUP_PROJECT_PACKAGE_INIT)

# .rst: .. command:: ADD_PROJECT_DEPENDENCY(ARGS [PKG_CONFIG_REQUIRES pkg]
# [FOR_COMPONENT component] [FIND_EXTERNAL pkg])
#
# This is a wrapper around find_package to add correct find_dependency calls in
# the generated config script. All arguments are passed to find_package.
#
# In cases where find_package is not supported by a project, or only in recent
# versions, one should provide a custom <PackageName>Config.cmake or use a more
# traditional way to get a dependency.
#
# If PKG_CONFIG_REQUIRES is provided, it will also add pkg to the Requires
# section of the generated .pc file
#
# If FIND_EXTERNAL is provided, a custom dependency finder will be added in
# $CMAKE_MODULE_PATH and installed with the project
#
macro(ADD_PROJECT_DEPENDENCY)
  # add dependency to the generated .pc ref
  # https://github.com/jrl-umi3218/jrl-cmakemodules/pull/335
  cmake_parse_arguments(
    PARSED_ARGN "" "PKG_CONFIG_REQUIRES;FOR_COMPONENT;FIND_EXTERNAL" "" ${ARGN})
  if(PARSED_ARGN_PKG_CONFIG_REQUIRES)
    _add_to_list_if_not_present(_PKG_CONFIG_REQUIRES
                                "${PARSED_ARGN_PKG_CONFIG_REQUIRES}")
    _add_to_list_if_not_present(_PKG_CONFIG_DEP_NOT_FOR_CONFIG_CMAKE
                                "${PARSED_ARGN_PKG_CONFIG_REQUIRES}")
  endif()
  if(PARSED_ARGN_FOR_COMPONENT)
    set(component "_${PARSED_ARGN_FOR_COMPONENT}")
  endif(PARSED_ARGN_FOR_COMPONENT)
  if(PARSED_ARGN_FIND_EXTERNAL)
    set(_ext "find-external/${PARSED_ARGN_FIND_EXTERNAL}")
    set(CMAKE_MODULE_PATH "${PROJECT_JRL_CMAKE_MODULE_DIR}/${_ext}"
                          ${CMAKE_MODULE_PATH})
    set(_PACKAGE_CONFIG_DEPENDENCIES_FIND_EXTERNAL
        "${_PACKAGE_CONFIG_DEPENDENCIES_FIND_EXTERNAL}\n \${PACKAGE_PREFIX_DIR}/${CONFIG_INSTALL_DIR}/${_ext}"
    )
    install(DIRECTORY "${PROJECT_JRL_CMAKE_MODULE_DIR}/${_ext}"
            DESTINATION "${CONFIG_INSTALL_DIR}/find-external")
  endif()
  _add_to_list_if_not_present(_PACKAGE_CONFIG${component}_DEPENDENCIES_PROJECTS
                              "${ARGV0}")

  string(REPLACE ";" " " PACKAGE_ARGS "${PARSED_ARGN_UNPARSED_ARGUMENTS}")
  _add_to_list_if_not_present(
    _PACKAGE_CONFIG${component}_DEPENDENCIES_FIND_PACKAGE
    "find_package(${PACKAGE_ARGS})")
  _add_to_list_if_not_present(
    _PACKAGE_CONFIG${component}_DEPENDENCIES_FIND_DEPENDENCY
    "find_dependency(${PACKAGE_ARGS})")
  find_package(${PARSED_ARGN_UNPARSED_ARGUMENTS})

  # Propagate variables changes to the cached values
  set(_PACKAGE_CONFIG${component}_DEPENDENCIES_PROJECTS
      "${_PACKAGE_CONFIG${component}_DEPENDENCIES_PROJECTS}"
      CACHE INTERNAL "")
  set(_PACKAGE_CONFIG${component}_DEPENDENCIES_FIND_PACKAGE
      "${_PACKAGE_CONFIG${component}_DEPENDENCIES_FIND_PACKAGE}"
      CACHE INTERNAL "")
  set(_PACKAGE_CONFIG${component}_DEPENDENCIES_FIND_DEPENDENCY
      "${_PACKAGE_CONFIG${component}_DEPENDENCIES_FIND_DEPENDENCY}"
      CACHE INTERNAL "")
endmacro()

# SETUP_PROJECT_PACKAGE_FINALIZE
# -------------
#
# Generates CMake PackageConfig.cmake, Targets, and Version files so users can
# call:
#
# find_package(MyPackage)
#
# Initialize the PackageConfig installation configuration.
# https://github.com/forexample/package-example
#
# This function does not take any argument but check that some variables are
# defined (see documentation at the beginning of this file).
#
# assumes SETUP_PROJECT() was called internally the real requirement is that
# _SETUP_PROJECT_PACKAGE_INIT() was called
macro(SETUP_PROJECT_PACKAGE_FINALIZE)
  #
  # Installation (https://github.com/forexample/package-example)

  # Layout. This works for all platforms: * <prefix>/lib/cmake/<PROJECT-NAME> *
  # <prefix>/lib/ * <prefix>/include/
  set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
  set(INCLUDE_INSTALL_DIR "include")
  set(INCLUDE_INSTALL_DESTINATION "${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}")

  set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

  # Configuration
  set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
  set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
  set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
  set(namespace "${PROJECT_NAME}::")
  string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
  string(REGEX REPLACE "[^a-zA-Z0-9]" "_" PROJECT_NAME_UPPER
                       "${PROJECT_NAME_UPPER}")

  # Include module with fuction 'write_basic_package_version_file'
  include(CMakePackageConfigHelpers)

  string(REPLACE ";" "\n  " PACKAGE_DEPENDENCIES_FIND_PACKAGE
                 "${_PACKAGE_CONFIG_DEPENDENCIES_FIND_PACKAGE}")
  string(REPLACE ";" "\n  " PACKAGE_DEPENDENCIES_FIND_DEPENDENCY
                 "${_PACKAGE_CONFIG_DEPENDENCIES_FIND_DEPENDENCY}")

  if(DEFINED _MINIMAL_CXX_STANDARD)
    install_jrl_cmakemodules_file("cxx-standard.cmake")

    # Add check for standard - enforce if required
    if(${MINIMAL_CXX_STANDARD_ENFORCE})
      set(PACKAGE_EXTRA_MACROS
          "${PACKAGE_EXTRA_MACROS}\nCHECK_MINIMAL_CXX_STANDARD(${_MINIMAL_CXX_STANDARD} ENFORCE)"
      )
    else()
      set(PACKAGE_EXTRA_MACROS
          "${PACKAGE_EXTRA_MACROS}\nCHECK_MINIMAL_CXX_STANDARD(${_MINIMAL_CXX_STANDARD})"
      )
    endif()
  endif()

  # Configure '<PROJECT-NAME>ConfigVersion.cmake' Note: PROJECT_VERSION is used
  # as a VERSION
  if(NOT PROJECT_COMPATIBILITY_VERSION)
    set(PROJECT_COMPATIBILITY_VERSION SameMajorVersion)
  endif()
  write_basic_package_version_file(
    "${VERSION_CONFIG}"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY ${PROJECT_COMPATIBILITY_VERSION})

  # Configure '<PROJECT-NAME>Config.cmake' Use variables: * TARGETS_EXPORT_NAME
  # * PROJECT_NAME * _PKG_CONFIG_REQUIRES
  unset(_PKG_CONFIG_REQUIRES_LIST)
  if(_PKG_CONFIG_REQUIRES)
    set(_PKG_CONFIG_REQUIRES_LIST "${_PKG_CONFIG_REQUIRES}")
    foreach(_pkg ${_PKG_CONFIG_DEP_NOT_FOR_CONFIG_CMAKE})
      list(REMOVE_ITEM _PKG_CONFIG_REQUIRES_LIST ${_pkg})
    endforeach()
    list(REMOVE_DUPLICATES _PKG_CONFIG_REQUIRES_LIST)
  endif(_PKG_CONFIG_REQUIRES)

  if(NOT PROJECT_EXPORT_NO_TARGET)
    set(INCLUDE_TARGETS_FILE
        "include(\"\${CMAKE_CURRENT_LIST_DIR}/${TARGETS_EXPORT_NAME}.cmake\")")
  else()
    set(INCLUDE_TARGETS_FILE "# Package with no targets")
  endif()

  set(INSTALL_FULL_INCLUDEDIR ${CMAKE_INSTALL_FULL_INCLUDEDIR})
  configure_package_config_file(
    "${PROJECT_JRL_CMAKE_MODULE_DIR}/Config.cmake.in" "${PROJECT_CONFIG}"
    INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
    PATH_VARS INSTALL_FULL_INCLUDEDIR)
  unset(INSTALL_FULL_INCLUDEDIR)

  # Config * <prefix>/lib/cmake/Foo/FooConfig.cmake *
  # <prefix>/lib/cmake/Foo/FooConfigVersion.cmake
  install(FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
          DESTINATION "${CONFIG_INSTALL_DIR}")

  # Config * <prefix>/lib/cmake/Foo/FooTargets.cmake
  if(NOT PROJECT_EXPORT_NO_TARGET)
    install(
      EXPORT "${TARGETS_EXPORT_NAME}"
      NAMESPACE "${namespace}"
      DESTINATION "${CONFIG_INSTALL_DIR}")
  endif()

endmacro(SETUP_PROJECT_PACKAGE_FINALIZE)

# .rst: .. command:: PROJECT_INSTALL_COMPONENT(COMPONENT [EXTRA_MACRO
# cmake_code] [NAMESPACE namespace])
#
# Generates CMake componentConfig.cmake and Targets files so users can call::
#
# find_package(MyPackage COMPONENT component)
#
# :param EXTRA_MACRO: optional argument. `cmake_code` will be appended to the
# generated *Config.cmake*. :param NAMESPACE: optional argument. Defaults to
# `${PROJECT_NAME}::`.
#
macro(PROJECT_INSTALL_COMPONENT COMPONENT)
  cmake_parse_arguments(PARSED_ARGN "" "NAMESPACE;EXTRA_MACRO" "" ${ARGN})
  if(PARSED_ARGN_NAMESPACE)
    set(namespace "${PARSED_ARGN_NAMESPACE}")
  else()
    set(namespace "${PROJECT_NAME}::")
  endif()

  install(
    EXPORT ${COMPONENT}Targets
    NAMESPACE "${namespace}"
    DESTINATION "${CONFIG_INSTALL_DIR}")

  set(COMPONENT ${COMPONENT})
  set(_PACKAGE_CONFIG_COMPONENT_DEPENDENCIES_PROJECTS
      "${_PACKAGE_CONFIG_${COMPONENT}_DEPENDENCIES_PROJECTS}")
  string(REPLACE ";" "\n  " COMPONENT_FIND_PACKAGE
                 "${_PACKAGE_CONFIG_${COMPONENT}_DEPENDENCIES_FIND_PACKAGE}")
  string(REPLACE ";" "\n  " COMPONENT_FIND_DEPENDENCY
                 "${_PACKAGE_CONFIG_${COMPONENT}_DEPENDENCIES_FIND_DEPENDENCY}")
  set(COMPONENT_CONFIG "${CMAKE_BINARY_DIR}/generated/${COMPONENT}Config.cmake")
  set(COMPONENT_EXTRA_MACRO "${PARSED_ARGN_EXTRA_MACRO}")
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    "${PROJECT_JRL_CMAKE_MODULE_DIR}/componentConfig.cmake.in"
    "${COMPONENT_CONFIG}"
    INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
    NO_CHECK_REQUIRED_COMPONENTS_MACRO NO_SET_AND_CHECK_MACRO)
  install(FILES "${COMPONENT_CONFIG}" DESTINATION "${CONFIG_INSTALL_DIR}")
endmacro()

# .rst: .. command:: INSTALL_JRL_CMAKEMODULES_FILE (filename)
#
# install jrl-cmakemodules/$filename along CMake package exports
#
macro(INSTALL_JRL_CMAKEMODULES_FILE filename)
  install(FILES "${PROJECT_JRL_CMAKE_MODULE_DIR}/${filename}"
          DESTINATION "${CONFIG_INSTALL_DIR}")
  set(INCLUDE_INSTALLED_JRL_FILES
      "${INCLUDE_INSTALLED_JRL_FILES}\ninclude(\"\${CMAKE_CURRENT_LIST_DIR}/${filename}\")"
  )
endmacro()

# .rst: .. command:: INSTALL_JRL_CMAKEMODULES_DIR (dirname)
#
# install jrl-cmakemodules/$dirname along CMake package exports
#
macro(INSTALL_JRL_CMAKEMODULES_DIR dirname)
  install(DIRECTORY "${PROJECT_JRL_CMAKE_MODULE_DIR}/${dirname}"
          DESTINATION "${CONFIG_INSTALL_DIR}")
endmacro()
