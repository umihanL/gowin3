// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rescue_interfaces:msg/FpgaEvent.idl
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
#include "rescue_interfaces/msg/detail/fpga_event__struct.h"
#include "rescue_interfaces/msg/detail/fpga_event__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rescue_interfaces__msg__fpga_event__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("rescue_interfaces.msg._fpga_event.FpgaEvent", full_classname_dest, 43) == 0);
  }
  rescue_interfaces__msg__FpgaEvent * ros_message = _ros_message;
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
  {  // event_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "event_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->event_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // wire_sequence
    PyObject * field = PyObject_GetAttrString(_pymsg, "wire_sequence");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->wire_sequence = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // event_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "event_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->event_id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // vehicle_stopped
    PyObject * field = PyObject_GetAttrString(_pymsg, "vehicle_stopped");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->vehicle_stopped = (Py_True == field);
    Py_DECREF(field);
  }
  {  // gimbal_settled
    PyObject * field = PyObject_GetAttrString(_pymsg, "gimbal_settled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->gimbal_settled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // measured_angles_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "measured_angles_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->measured_angles_valid = (Py_True == field);
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
  {  // acknowledged_sequence
    PyObject * field = PyObject_GetAttrString(_pymsg, "acknowledged_sequence");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->acknowledged_sequence = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // acknowledgement_result
    PyObject * field = PyObject_GetAttrString(_pymsg, "acknowledgement_result");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->acknowledgement_result = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rescue_interfaces__msg__fpga_event__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of FpgaEvent */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rescue_interfaces.msg._fpga_event");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "FpgaEvent");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rescue_interfaces__msg__FpgaEvent * ros_message = (rescue_interfaces__msg__FpgaEvent *)raw_ros_message;
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
  {  // event_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->event_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "event_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wire_sequence
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->wire_sequence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wire_sequence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // event_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->event_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "event_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vehicle_stopped
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->vehicle_stopped ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vehicle_stopped", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gimbal_settled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->gimbal_settled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gimbal_settled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // measured_angles_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->measured_angles_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "measured_angles_valid", field);
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
  {  // acknowledged_sequence
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->acknowledged_sequence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acknowledged_sequence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acknowledgement_result
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->acknowledgement_result);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acknowledgement_result", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
