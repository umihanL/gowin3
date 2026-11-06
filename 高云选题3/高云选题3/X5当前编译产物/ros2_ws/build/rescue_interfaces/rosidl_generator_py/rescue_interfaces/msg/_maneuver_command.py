# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rescue_interfaces:msg/ManeuverCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ManeuverCommand(type):
    """Metaclass of message 'ManeuverCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FOLLOW_LANE': 0,
        'TURN_LEFT': 1,
        'TURN_RIGHT': 2,
        'STRAIGHT': 3,
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
                'rescue_interfaces.msg.ManeuverCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__maneuver_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__maneuver_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__maneuver_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__maneuver_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__maneuver_command

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FOLLOW_LANE': cls.__constants['FOLLOW_LANE'],
            'TURN_LEFT': cls.__constants['TURN_LEFT'],
            'TURN_RIGHT': cls.__constants['TURN_RIGHT'],
            'STRAIGHT': cls.__constants['STRAIGHT'],
        }

    @property
    def FOLLOW_LANE(self):
        """Message constant 'FOLLOW_LANE'."""
        return Metaclass_ManeuverCommand.__constants['FOLLOW_LANE']

    @property
    def TURN_LEFT(self):
        """Message constant 'TURN_LEFT'."""
        return Metaclass_ManeuverCommand.__constants['TURN_LEFT']

    @property
    def TURN_RIGHT(self):
        """Message constant 'TURN_RIGHT'."""
        return Metaclass_ManeuverCommand.__constants['TURN_RIGHT']

    @property
    def STRAIGHT(self):
        """Message constant 'STRAIGHT'."""
        return Metaclass_ManeuverCommand.__constants['STRAIGHT']


class ManeuverCommand(metaclass=Metaclass_ManeuverCommand):
    """
    Message class 'ManeuverCommand'.

    Constants:
      FOLLOW_LANE
      TURN_LEFT
      TURN_RIGHT
      STRAIGHT
    """

    __slots__ = [
        '_header',
        '_command_id',
        '_edge_id',
        '_route_revision',
        '_turn',
        '_stop_at_end',
        '_speed_limit_m_s',
        '_distance_m',
        '_lease_ms',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'command_id': 'string',
        'edge_id': 'string',
        'route_revision': 'uint32',
        'turn': 'uint8',
        'stop_at_end': 'boolean',
        'speed_limit_m_s': 'float',
        'distance_m': 'float',
        'lease_ms': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.command_id = kwargs.get('command_id', str())
        self.edge_id = kwargs.get('edge_id', str())
        self.route_revision = kwargs.get('route_revision', int())
        self.turn = kwargs.get('turn', int())
        self.stop_at_end = kwargs.get('stop_at_end', bool())
        self.speed_limit_m_s = kwargs.get('speed_limit_m_s', float())
        self.distance_m = kwargs.get('distance_m', float())
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
        if self.command_id != other.command_id:
            return False
        if self.edge_id != other.edge_id:
            return False
        if self.route_revision != other.route_revision:
            return False
        if self.turn != other.turn:
            return False
        if self.stop_at_end != other.stop_at_end:
            return False
        if self.speed_limit_m_s != other.speed_limit_m_s:
            return False
        if self.distance_m != other.distance_m:
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
    def command_id(self):
        """Message field 'command_id'."""
        return self._command_id

    @command_id.setter
    def command_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'command_id' field must be of type 'str'"
        self._command_id = value

    @builtins.property
    def edge_id(self):
        """Message field 'edge_id'."""
        return self._edge_id

    @edge_id.setter
    def edge_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'edge_id' field must be of type 'str'"
        self._edge_id = value

    @builtins.property
    def route_revision(self):
        """Message field 'route_revision'."""
        return self._route_revision

    @route_revision.setter
    def route_revision(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'route_revision' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'route_revision' field must be an unsigned integer in [0, 4294967295]"
        self._route_revision = value

    @builtins.property
    def turn(self):
        """Message field 'turn'."""
        return self._turn

    @turn.setter
    def turn(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'turn' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'turn' field must be an unsigned integer in [0, 255]"
        self._turn = value

    @builtins.property
    def stop_at_end(self):
        """Message field 'stop_at_end'."""
        return self._stop_at_end

    @stop_at_end.setter
    def stop_at_end(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stop_at_end' field must be of type 'bool'"
        self._stop_at_end = value

    @builtins.property
    def speed_limit_m_s(self):
        """Message field 'speed_limit_m_s'."""
        return self._speed_limit_m_s

    @speed_limit_m_s.setter
    def speed_limit_m_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_limit_m_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_limit_m_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_limit_m_s = value

    @builtins.property
    def distance_m(self):
        """Message field 'distance_m'."""
        return self._distance_m

    @distance_m.setter
    def distance_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance_m = value

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
