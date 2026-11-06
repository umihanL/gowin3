# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rescue_interfaces:msg/Detection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'bbox_xyxy'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Detection(type):
    """Metaclass of message 'Detection'."""

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
                'rescue_interfaces.msg.Detection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detection

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Detection(metaclass=Metaclass_Detection):
    """Message class 'Detection'."""

    __slots__ = [
        '_class_id',
        '_class_name',
        '_confidence',
        '_bbox_xyxy',
        '_depth_valid',
        '_position_m',
        '_depth_m',
        '_depth_reason',
    ]

    _fields_and_field_types = {
        'class_id': 'int32',
        'class_name': 'string',
        'confidence': 'float',
        'bbox_xyxy': 'float[4]',
        'depth_valid': 'boolean',
        'position_m': 'geometry_msgs/Point',
        'depth_m': 'float',
        'depth_reason': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.class_id = kwargs.get('class_id', int())
        self.class_name = kwargs.get('class_name', str())
        self.confidence = kwargs.get('confidence', float())
        if 'bbox_xyxy' not in kwargs:
            self.bbox_xyxy = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.bbox_xyxy = numpy.array(kwargs.get('bbox_xyxy'), dtype=numpy.float32)
            assert self.bbox_xyxy.shape == (4, )
        self.depth_valid = kwargs.get('depth_valid', bool())
        from geometry_msgs.msg import Point
        self.position_m = kwargs.get('position_m', Point())
        self.depth_m = kwargs.get('depth_m', float())
        self.depth_reason = kwargs.get('depth_reason', str())

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
        if self.class_id != other.class_id:
            return False
        if self.class_name != other.class_name:
            return False
        if self.confidence != other.confidence:
            return False
        if any(self.bbox_xyxy != other.bbox_xyxy):
            return False
        if self.depth_valid != other.depth_valid:
            return False
        if self.position_m != other.position_m:
            return False
        if self.depth_m != other.depth_m:
            return False
        if self.depth_reason != other.depth_reason:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def class_id(self):
        """Message field 'class_id'."""
        return self._class_id

    @class_id.setter
    def class_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'class_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'class_id' field must be an integer in [-2147483648, 2147483647]"
        self._class_id = value

    @builtins.property
    def class_name(self):
        """Message field 'class_name'."""
        return self._class_name

    @class_name.setter
    def class_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'class_name' field must be of type 'str'"
        self._class_name = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value

    @builtins.property
    def bbox_xyxy(self):
        """Message field 'bbox_xyxy'."""
        return self._bbox_xyxy

    @bbox_xyxy.setter
    def bbox_xyxy(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'bbox_xyxy' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'bbox_xyxy' numpy.ndarray() must have a size of 4"
            self._bbox_xyxy = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'bbox_xyxy' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._bbox_xyxy = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def depth_valid(self):
        """Message field 'depth_valid'."""
        return self._depth_valid

    @depth_valid.setter
    def depth_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'depth_valid' field must be of type 'bool'"
        self._depth_valid = value

    @builtins.property
    def position_m(self):
        """Message field 'position_m'."""
        return self._position_m

    @position_m.setter
    def position_m(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position_m' field must be a sub message of type 'Point'"
        self._position_m = value

    @builtins.property
    def depth_m(self):
        """Message field 'depth_m'."""
        return self._depth_m

    @depth_m.setter
    def depth_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'depth_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._depth_m = value

    @builtins.property
    def depth_reason(self):
        """Message field 'depth_reason'."""
        return self._depth_reason

    @depth_reason.setter
    def depth_reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'depth_reason' field must be of type 'str'"
        self._depth_reason = value
