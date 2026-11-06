// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rescue_interfaces:msg/ManeuverCommand.idl
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
#include "rescue_interfaces/msg/detail/maneuver_command__struct.h"
#include "rescue_interfaces/msg/detail/maneuver_command__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rescue_interfaces__msg__maneuver_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("rescue_interfaces.msg._maneuver_command.ManeuverCommand", full_classname_dest, 55) == 0);
  }
  rescue_interfaces__msg__ManeuverCommand * ros_message = _ros_message;
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
  {  // command_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "command_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->command_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // edge_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "edge_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->edge_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // route_revision
    PyObject * field = PyObject_GetAttrString(_pymsg, "route_revision");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->route_revision = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // turn
    PyObject * field = PyObject_GetAttrString(_pymsg, "turn");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->turn = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // stop_at_end
    PyObject * field = PyObject_GetAttrString(_pymsg, "stop_at_end");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stop_at_end = (Py_True == field);
    Py_DECREF(field);
  }
  {  // speed_limit_m_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_limit_m_s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_limit_m_s = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // distance_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance_m = (float)PyFloat_AS_DOUBLE(field);
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
PyObject * rescue_interfaces__msg__maneuver_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ManeuverCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rescue_interfaces.msg._maneuver_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ManeuverCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rescue_interfaces__msg__ManeuverCommand * ros_message = (rescue_interfaces__msg__ManeuverCommand *)raw_ros_message;
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
  {  // command_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->command_id.data,
      strlen(ros_message->command_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "command_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // edge_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->edge_id.data,
      strlen(ros_message->edge_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "edge_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // route_revision
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->route_revision);
    {
      int rc = PyObject_SetAttrString(_pymessage, "route_revision", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // turn
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->turn);
    {
      int rc = PyObject_SetAttrString(_pymessage, "turn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stop_at_end
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stop_at_end ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stop_at_end", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_limit_m_s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_limit_m_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_limit_m_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_m", field);
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
