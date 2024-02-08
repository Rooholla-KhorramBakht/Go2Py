# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_api:msg/ResponseHeader.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ResponseHeader(type):
    """Metaclass of message 'ResponseHeader'."""

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
            module = import_type_support('unitree_api')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'unitree_api.msg.ResponseHeader')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__response_header
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__response_header
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__response_header
            cls._TYPE_SUPPORT = module.type_support_msg__msg__response_header
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__response_header

            from unitree_api.msg import RequestIdentity
            if RequestIdentity.__class__._TYPE_SUPPORT is None:
                RequestIdentity.__class__.__import_type_support__()

            from unitree_api.msg import ResponseStatus
            if ResponseStatus.__class__._TYPE_SUPPORT is None:
                ResponseStatus.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ResponseHeader(metaclass=Metaclass_ResponseHeader):
    """Message class 'ResponseHeader'."""

    __slots__ = [
        '_identity',
        '_status',
    ]

    _fields_and_field_types = {
        'identity': 'unitree_api/RequestIdentity',
        'status': 'unitree_api/ResponseStatus',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unitree_api', 'msg'], 'RequestIdentity'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['unitree_api', 'msg'], 'ResponseStatus'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unitree_api.msg import RequestIdentity
        self.identity = kwargs.get('identity', RequestIdentity())
        from unitree_api.msg import ResponseStatus
        self.status = kwargs.get('status', ResponseStatus())

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
        if self.identity != other.identity:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def identity(self):
        """Message field 'identity'."""
        return self._identity

    @identity.setter
    def identity(self, value):
        if __debug__:
            from unitree_api.msg import RequestIdentity
            assert \
                isinstance(value, RequestIdentity), \
                "The 'identity' field must be a sub message of type 'RequestIdentity'"
        self._identity = value

    @property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            from unitree_api.msg import ResponseStatus
            assert \
                isinstance(value, ResponseStatus), \
                "The 'status' field must be a sub message of type 'ResponseStatus'"
        self._status = value
