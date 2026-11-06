// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rescue_interfaces:msg/GimbalCommand.idl
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
#include "rescue_interfaces/msg/detail/gimbal_command__struct.h"
#include "rescue_interfaces/msg/detail/gimbal_command__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rescue_interfaces__msg__gimbal_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
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
    assert(strncmp("rescue_interfaces.msg._gimbal_command.GimbalCommand", full_classname_dest, 51) == 0);
  }
  rescue_interfaces__msg__GimbalCommand * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // owner
    PyObject * field = PyObject_GetAttrString(_pymsg, "owner");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->owner, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // pan_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "pan_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pan_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tilt_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "tilt_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tilt_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed_deg_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_deg_s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_deg_s = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lease_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "lease_ms");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->lease_ms = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rescue_interfaces__msg__gimbal_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GimbalCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rescue_interfaces.msg._gimbal_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GimbalCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rescue_interfaces__msg__GimbalCommand * ros_message = (rescue_interfaces__msg__GimbalCommand *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // owner
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->owner.data,
      strlen(ros_message->owner.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "owner", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pan_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pan_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pan_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tilt_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tilt_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tilt_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_deg_s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_deg_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_deg_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lease_ms
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->lease_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lease_ms", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
