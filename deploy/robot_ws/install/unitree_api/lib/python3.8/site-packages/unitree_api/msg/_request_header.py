# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_api:msg/RequestHeader.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RequestHeader(type):
    """Metaclass of message 'RequestHeader'."""

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
                'unitree_api.msg.RequestHeader')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__request_header
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__request_header
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__request_header
            cls._TYPE_SUPPORT = module.type_support_msg__msg__request_header
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__request_header

            from unitree_api.msg import RequestIdentity
            if RequestIdentity.__class__._TYPE_SUPPORT is None:
                RequestIdentity.__class__.__import_type_support__()

            from unitree_api.msg import RequestLease
            if RequestLease.__class__._TYPE_SUPPORT is None:
                RequestLease.__class__.__import_type_support__()

            from unitree_api.msg import RequestPolicy
            if RequestPolicy.__class__._TYPE_SUPPORT is None:
                RequestPolicy.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RequestHeader(metaclass=Metaclass_RequestHeader):
    """Message class 'RequestHeader'."""

    __slots__ = [
        '_identity',
        '_lease',
        '_policy',
    ]

    _fields_and_field_types = {
        'identity': 'unitree_api/RequestIdentity',
        'lease': 'unitree_api/RequestLease',
        'policy': 'unitree_api/RequestPolicy',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unitree_api', 'msg'], 'RequestIdentity'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['unitree_api', 'msg'], 'RequestLease'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['unitree_api', 'msg'], 'RequestPolicy'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unitree_api.msg import RequestIdentity
        self.identity = kwargs.get('identity', RequestIdentity())
        from unitree_api.msg import RequestLease
        self.lease = kwargs.get('lease', RequestLease())
        from unitree_api.msg import RequestPolicy
        self.policy = kwargs.get('policy', RequestPolicy())

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
        if self.lease != other.lease:
            return False
        if self.policy != other.policy:
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
    def lease(self):
        """Message field 'lease'."""
        return self._lease

    @lease.setter
    def lease(self, value):
        if __debug__:
            from unitree_api.msg import RequestLease
            assert \
                isinstance(value, RequestLease), \
                "The 'lease' field must be a sub message of type 'RequestLease'"
        self._lease = value

    @property
    def policy(self):
        """Message field 'policy'."""
        return self._policy

    @policy.setter
    def policy(self, value):
        if __debug__:
            from unitree_api.msg import RequestPolicy
            assert \
                isinstance(value, RequestPolicy), \
                "The 'policy' field must be a sub message of type 'RequestPolicy'"
        self._policy = value
