# generated from rosidl_generator_py/resource/_idl.py.em
# with input from aic_task_interfaces:msg/Task.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Task(type):
    """Metaclass of message 'Task'."""

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
            module = import_type_support('aic_task_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'aic_task_interfaces.msg.Task')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__task
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__task
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__task
            cls._TYPE_SUPPORT = module.type_support_msg__msg__task
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__task

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Task(metaclass=Metaclass_Task):
    """Message class 'Task'."""

    __slots__ = [
        '_id',
        '_cable_type',
        '_cable_name',
        '_plug_type',
        '_plug_name',
        '_port_type',
        '_port_name',
        '_target_module_name',
        '_time_limit',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'id': 'string',
        'cable_type': 'string',
        'cable_name': 'string',
        'plug_type': 'string',
        'plug_name': 'string',
        'port_type': 'string',
        'port_name': 'string',
        'target_module_name': 'string',
        'time_limit': 'uint64',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', str())
        self.cable_type = kwargs.get('cable_type', str())
        self.cable_name = kwargs.get('cable_name', str())
        self.plug_type = kwargs.get('plug_type', str())
        self.plug_name = kwargs.get('plug_name', str())
        self.port_type = kwargs.get('port_type', str())
        self.port_name = kwargs.get('port_name', str())
        self.target_module_name = kwargs.get('target_module_name', str())
        self.time_limit = kwargs.get('time_limit', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id != other.id:
            return False
        if self.cable_type != other.cable_type:
            return False
        if self.cable_name != other.cable_name:
            return False
        if self.plug_type != other.plug_type:
            return False
        if self.plug_name != other.plug_name:
            return False
        if self.port_type != other.port_type:
            return False
        if self.port_name != other.port_name:
            return False
        if self.target_module_name != other.target_module_name:
            return False
        if self.time_limit != other.time_limit:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'id' field must be of type 'str'"
        self._id = value

    @builtins.property
    def cable_type(self):
        """Message field 'cable_type'."""
        return self._cable_type

    @cable_type.setter
    def cable_type(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'cable_type' field must be of type 'str'"
        self._cable_type = value

    @builtins.property
    def cable_name(self):
        """Message field 'cable_name'."""
        return self._cable_name

    @cable_name.setter
    def cable_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'cable_name' field must be of type 'str'"
        self._cable_name = value

    @builtins.property
    def plug_type(self):
        """Message field 'plug_type'."""
        return self._plug_type

    @plug_type.setter
    def plug_type(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'plug_type' field must be of type 'str'"
        self._plug_type = value

    @builtins.property
    def plug_name(self):
        """Message field 'plug_name'."""
        return self._plug_name

    @plug_name.setter
    def plug_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'plug_name' field must be of type 'str'"
        self._plug_name = value

    @builtins.property
    def port_type(self):
        """Message field 'port_type'."""
        return self._port_type

    @port_type.setter
    def port_type(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'port_type' field must be of type 'str'"
        self._port_type = value

    @builtins.property
    def port_name(self):
        """Message field 'port_name'."""
        return self._port_name

    @port_name.setter
    def port_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'port_name' field must be of type 'str'"
        self._port_name = value

    @builtins.property
    def target_module_name(self):
        """Message field 'target_module_name'."""
        return self._target_module_name

    @target_module_name.setter
    def target_module_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'target_module_name' field must be of type 'str'"
        self._target_module_name = value

    @builtins.property
    def time_limit(self):
        """Message field 'time_limit'."""
        return self._time_limit

    @time_limit.setter
    def time_limit(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'time_limit' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'time_limit' field must be an unsigned integer in [0, 18446744073709551615]"
        self._time_limit = value
