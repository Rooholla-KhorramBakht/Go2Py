// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from unitree_api:msg/RequestPolicy.idl
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
#include "unitree_api/msg/detail/request_policy__struct.h"
#include "unitree_api/msg/detail/request_policy__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool unitree_api__msg__request_policy__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("unitree_api.msg._request_policy.RequestPolicy", full_classname_dest, 45) == 0);
  }
  unitree_api__msg__RequestPolicy * ros_message = _ros_message;
  {  // priority
    PyObject * field = PyObject_GetAttrString(_pymsg, "priority");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->priority = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // noreply
    PyObject * field = PyObject_GetAttrString(_pymsg, "noreply");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->noreply = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * unitree_api__msg__request_policy__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RequestPolicy */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("unitree_api.msg._request_policy");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RequestPolicy");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  unitree_api__msg__RequestPolicy * ros_message = (unitree_api__msg__RequestPolicy *)raw_ros_message;
  {  // priority
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->priority);
    {
      int rc = PyObject_SetAttrString(_pymessage, "priority", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // noreply
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->noreply ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "noreply", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
