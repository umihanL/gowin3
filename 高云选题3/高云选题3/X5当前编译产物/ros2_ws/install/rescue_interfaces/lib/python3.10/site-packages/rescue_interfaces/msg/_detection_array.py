# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rescue_interfaces:msg/DetectionArray.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DetectionArray(type):
    """Metaclass of message 'DetectionArray'."""

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
                'rescue_interfaces.msg.DetectionArray')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detection_array
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detection_array
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detection_array
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detection_array
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detection_array

            from rescue_interfaces.msg import Detection
            if Detection.__class__._TYPE_SUPPORT is None:
                Detection.__class__.__import_type_support__()

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


class DetectionArray(metaclass=Metaclass_DetectionArray):
    """Message class 'DetectionArray'."""

    __slots__ = [
        '_header',
        '_sequence',
        '_width',
        '_height',
        '_rectified',
        '_detector_available',
        '_status',
        '_stereo_calibration_id',
        '_detections',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'sequence': 'uint64',
        'width': 'uint32',
        'height': 'uint32',
        'rectified': 'boolean',
        'detector_available': 'boolean',
        'status': 'string',
        'stereo_calibration_id': 'string',
        'detections': 'sequence<rescue_interfaces/Detection>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rescue_interfaces', 'msg'], 'Detection')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.sequence = kwargs.get('sequence', int())
        self.width = kwargs.get('width', int())
        self.height = kwargs.get('height', int())
        self.rectified = kwargs.get('rectified', bool())
        self.detector_available = kwargs.get('detector_available', bool())
        self.status = kwargs.get('status', str())
        self.stereo_calibration_id = kwargs.get('stereo_calibration_id', str())
        self.detections = kwargs.get('detections', [])

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
        if self.sequence != other.sequence:
            return False
        if self.width != other.width:
            return False
        if self.height != other.height:
            return False
        if self.rectified != other.rectified:
            return False
        if self.detector_available != other.detector_available:
            return False
        if self.status != other.status:
            return False
        if self.stereo_calibration_id != other.stereo_calibration_id:
            return False
        if self.detections != other.detections:
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
    def sequence(self):
        """Message field 'sequence'."""
        return self._sequence

    @sequence.setter
    def sequence(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sequence' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'sequence' field must be an unsigned integer in [0, 18446744073709551615]"
        self._sequence = value

    @builtins.property
    def width(self):
        """Message field 'width'."""
        return self._width

    @width.setter
    def width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'width' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'width' field must be an unsigned integer in [0, 4294967295]"
        self._width = value

    @builtins.property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'height' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'height' field must be an unsigned integer in [0, 4294967295]"
        self._height = value

    @builtins.property
    def rectified(self):
        """Message field 'rectified'."""
        return self._rectified

    @rectified.setter
    def rectified(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rectified' field must be of type 'bool'"
        self._rectified = value

    @builtins.property
    def detector_available(self):
        """Message field 'detector_available'."""
        return self._detector_available

    @detector_available.setter
    def detector_available(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'detector_available' field must be of type 'bool'"
        self._detector_available = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status' field must be of type 'str'"
        self._status = value

    @builtins.property
    def stereo_calibration_id(self):
        """Message field 'stereo_calibration_id'."""
        return self._stereo_calibration_id

    @stereo_calibration_id.setter
    def stereo_calibration_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'stereo_calibration_id' field must be of type 'str'"
        self._stereo_calibration_id = value

    @builtins.property
    def detections(self):
        """Message field 'detections'."""
        return self._detections

    @detections.setter
    def detections(self, value):
        if __debug__:
            from rescue_interfaces.msg import Detection
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
                 all(isinstance(v, Detection) for v in value) and
                 True), \
                "The 'detections' field must be a set or sequence and each value of type 'Detection'"
        self._detections = value
