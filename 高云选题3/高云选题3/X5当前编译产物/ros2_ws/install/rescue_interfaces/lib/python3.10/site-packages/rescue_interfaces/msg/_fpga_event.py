# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rescue_interfaces:msg/FpgaEvent.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FpgaEvent(type):
    """Metaclass of message 'FpgaEvent'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CROSSWALK': 16,
        'CROSSING_COMPLETE': 17,
        'STATUS': 18,
        'ACK': 19,
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
                'rescue_interfaces.msg.FpgaEvent')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__fpga_event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__fpga_event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__fpga_event
            cls._TYPE_SUPPORT = module.type_support_msg__msg__fpga_event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__fpga_event

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CROSSWALK': cls.__constants['CROSSWALK'],
            'CROSSING_COMPLETE': cls.__constants['CROSSING_COMPLETE'],
            'STATUS': cls.__constants['STATUS'],
            'ACK': cls.__constants['ACK'],
        }

    @property
    def CROSSWALK(self):
        """Message constant 'CROSSWALK'."""
        return Metaclass_FpgaEvent.__constants['CROSSWALK']

    @property
    def CROSSING_COMPLETE(self):
        """Message constant 'CROSSING_COMPLETE'."""
        return Metaclass_FpgaEvent.__constants['CROSSING_COMPLETE']

    @property
    def STATUS(self):
        """Message constant 'STATUS'."""
        return Metaclass_FpgaEvent.__constants['STATUS']

    @property
    def ACK(self):
        """Message constant 'ACK'."""
        return Metaclass_FpgaEvent.__constants['ACK']


class FpgaEvent(metaclass=Metaclass_FpgaEvent):
    """
    Message class 'FpgaEvent'.

    Constants:
      CROSSWALK
      CROSSING_COMPLETE
      STATUS
      ACK
    """

    __slots__ = [
        '_header',
        '_event_type',
        '_wire_sequence',
        '_event_id',
        '_vehicle_stopped',
        '_gimbal_settled',
        '_measured_angles_valid',
        '_pan_deg',
        '_tilt_deg',
        '_acknowledged_sequence',
        '_acknowledgement_result',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'event_type': 'uint8',
        'wire_sequence': 'uint16',
        'event_id': 'uint32',
        'vehicle_stopped': 'boolean',
        'gimbal_settled': 'boolean',
        'measured_angles_valid': 'boolean',
        'pan_deg': 'float',
        'tilt_deg': 'float',
        'acknowledged_sequence': 'uint16',
        'acknowledgement_result': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.event_type = kwargs.get('event_type', int())
        self.wire_sequence = kwargs.get('wire_sequence', int())
        self.event_id = kwargs.get('event_id', int())
        self.vehicle_stopped = kwargs.get('vehicle_stopped', bool())
        self.gimbal_settled = kwargs.get('gimbal_settled', bool())
        self.measured_angles_valid = kwargs.get('measured_angles_valid', bool())
        self.pan_deg = kwargs.get('pan_deg', float())
        self.tilt_deg = kwargs.get('tilt_deg', float())
        self.acknowledged_sequence = kwargs.get('acknowledged_sequence', int())
        self.acknowledgement_result = kwargs.get('acknowledgement_result', int())

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
        if self.event_type != other.event_type:
            return False
        if self.wire_sequence != other.wire_sequence:
            return False
        if self.event_id != other.event_id:
            return False
        if self.vehicle_stopped != other.vehicle_stopped:
            return False
        if self.gimbal_settled != other.gimbal_settled:
            return False
        if self.measured_angles_valid != other.measured_angles_valid:
            return False
        if self.pan_deg != other.pan_deg:
            return False
        if self.tilt_deg != other.tilt_deg:
            return False
        if self.acknowledged_sequence != other.acknowledged_sequence:
            return False
        if self.acknowledgement_result != other.acknowledgement_result:
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
    def event_type(self):
        """Message field 'event_type'."""
        return self._event_type

    @event_type.setter
    def event_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'event_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'event_type' field must be an unsigned integer in [0, 255]"
        self._event_type = value

    @builtins.property
    def wire_sequence(self):
        """Message field 'wire_sequence'."""
        return self._wire_sequence

    @wire_sequence.setter
    def wire_sequence(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'wire_sequence' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'wire_sequence' field must be an unsigned integer in [0, 65535]"
        self._wire_sequence = value

    @builtins.property
    def event_id(self):
        """Message field 'event_id'."""
        return self._event_id

    @event_id.setter
    def event_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'event_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'event_id' field must be an unsigned integer in [0, 4294967295]"
        self._event_id = value

    @builtins.property
    def vehicle_stopped(self):
        """Message field 'vehicle_stopped'."""
        return self._vehicle_stopped

    @vehicle_stopped.setter
    def vehicle_stopped(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'vehicle_stopped' field must be of type 'bool'"
        self._vehicle_stopped = value

    @builtins.property
    def gimbal_settled(self):
        """Message field 'gimbal_settled'."""
        return self._gimbal_settled

    @gimbal_settled.setter
    def gimbal_settled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'gimbal_settled' field must be of type 'bool'"
        self._gimbal_settled = value

    @builtins.property
    def measured_angles_valid(self):
        """Message field 'measured_angles_valid'."""
        return self._measured_angles_valid

    @measured_angles_valid.setter
    def measured_angles_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'measured_angles_valid' field must be of type 'bool'"
        self._measured_angles_valid = value

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
    def acknowledged_sequence(self):
        """Message field 'acknowledged_sequence'."""
        return self._acknowledged_sequence

    @acknowledged_sequence.setter
    def acknowledged_sequence(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'acknowledged_sequence' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'acknowledged_sequence' field must be an unsigned integer in [0, 65535]"
        self._acknowledged_sequence = value

    @builtins.property
    def acknowledgement_result(self):
        """Message field 'acknowledgement_result'."""
        return self._acknowledgement_result

    @acknowledgement_result.setter
    def acknowledgement_result(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'acknowledgement_result' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'acknowledgement_result' field must be an unsigned integer in [0, 255]"
        self._acknowledgement_result = value
