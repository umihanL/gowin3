# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rescue_interfaces:msg/GimbalCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GimbalCommand(type):
    """Metaclass of message 'GimbalCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rescue_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rescue_interfaces.msg.GimbalCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gimbal_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gimbal_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gimbal_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gimbal_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gimbal_command

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GimbalCommand(metaclass=Metaclass_GimbalCommand):
    """Message class 'GimbalCommand'."""

    __slots__ = [
        '_header',
        '_owner',
        '_pan_deg',
        '_tilt_deg',
        '_speed_deg_s',
        '_lease_ms',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'owner': 'string',
        'pan_deg': 'float',
        'tilt_deg': 'float',
        'speed_deg_s': 'float',
        'lease_ms': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.owner = kwargs.get('owner', str())
        self.pan_deg = kwargs.get('pan_deg', float())
        self.tilt_deg = kwargs.get('tilt_deg', float())
        self.speed_deg_s = kwargs.get('speed_deg_s', float())
        self.lease_ms = kwargs.get('lease_ms', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.owner != other.owner:
            return False
        if self.pan_deg != other.pan_deg:
            return False
        if self.tilt_deg != other.tilt_deg:
            return False
        if self.speed_deg_s != other.speed_deg_s:
            return False
        if self.lease_ms != other.lease_ms:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def owner(self):
        """Message field 'owner'."""
        return self._owner

    @owner.setter
    def owner(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'owner' field must be of type 'str'"
        self._owner = value

    @builtins.property
    def pan_deg(self):
        """Message field 'pan_deg'."""
        return self._pan_deg

    @pan_deg.setter
    def pan_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pan_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pan_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pan_deg = value

    @builtins.property
    def tilt_deg(self):
        """Message field 'tilt_deg'."""
        return self._tilt_deg

    @tilt_deg.setter
    def tilt_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tilt_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tilt_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tilt_deg = value

    @builtins.property
    def speed_deg_s(self):
        """Message field 'speed_deg_s'."""
        return self._speed_deg_s

    @speed_deg_s.setter
    def speed_deg_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_deg_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_deg_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_deg_s = value

    @builtins.property
    def lease_ms(self):
        """Message field 'lease_ms'."""
        return self._lease_ms

    @lease_ms.setter
    def lease_ms(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lease_ms' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'lease_ms' field must be an unsigned integer in [0, 65535]"
        self._lease_ms = value
