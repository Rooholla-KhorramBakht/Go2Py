// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from unitree_api:msg/RequestHeader.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "unitree_api/msg/detail/request_header__struct.h"
#include "unitree_api/msg/detail/request_header__functions.h"

bool unitree_api__msg__request_identity__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * unitree_api__msg__request_identity__convert_to_py(void * raw_ros_message);
bool unitree_api__msg__request_lease__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * unitree_api__msg__request_lease__convert_to_py(void * raw_ros_message);
bool unitree_api__msg__request_policy__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * unitree_api__msg__request_policy__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool unitree_api__msg__request_header__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("unitree_api.msg._request_header.RequestHeader", full_classname_dest, 45) == 0);
  }
  unitree_api__msg__RequestHeader * ros_message = _ros_message;
  {  // identity
    PyObject * field = PyObject_GetAttrString(_pymsg, "identity");
    if (!field) {
      return false;
    }
    if (!unitree_api__msg__request_identity__convert_from_py(field, &ros_message->identity)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // lease
    PyObject * field = PyObject_GetAttrString(_pymsg, "lease");
    if (!field) {
      return false;
    }
    if (!unitree_api__msg__request_lease__convert_from_py(field, &ros_message->lease)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // policy
    PyObject * field = PyObject_GetAttrString(_pymsg, "policy");
    if (!field) {
      return false;
    }
    if (!unitree_api__msg__request_policy__convert_from_py(field, &ros_message->policy)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * unitree_api__msg__request_header__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RequestHeader */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("unitree_api.msg._request_header");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RequestHeader");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  unitree_api__msg__RequestHeader * ros_message = (unitree_api__msg__RequestHeader *)raw_ros_message;
  {  // identity
    PyObject * field = NULL;
    field = unitree_api__msg__request_identity__convert_to_py(&ros_message->identity);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "identity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lease
    PyObject * field = NULL;
    field = unitree_api__msg__request_lease__convert_to_py(&ros_message->lease);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "lease", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // policy
    PyObject * field = NULL;
    field = unitree_api__msg__request_policy__convert_to_py(&ros_message->policy);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "policy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
