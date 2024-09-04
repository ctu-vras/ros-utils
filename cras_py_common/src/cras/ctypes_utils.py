# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with the ctypes library."""

from ctypes import CDLL, CFUNCTYPE, c_bool, c_char_p, c_void_p, c_size_t, cast, create_string_buffer, c_uint8, \
    get_errno, POINTER, set_errno, sizeof, string_at
from ctypes.util import find_library
from logging import LogRecord
import os
import sys

from rosgraph.roslogging import RosStreamHandler
import rospy

RTLD_LAZY = 1
"""Relocate dynamic symbols only when they are used."""

RTLD_NOW = 2
"""Relocate dynamic symbols as soon as the library is loaded."""


def load_library(library_name, mode=RTLD_LAZY, use_errno=False):
    """Load a C library (.so file) located on LD_LIBRARY_PATH.

    All dependencies of this library have to be loaded before this one with :obj:`mode` =  :const:`ctypes.RTLD_GLOBAL`.

    :param str library_name: Name of the library (without the "lib" prefix and ".so" suffix).
    :param int mode: Symbol loading scope and when they should be loaded. Either :const:`ctypes.RTLD_LOCAL`,
                     :const:`ctypes.RTLD_GLOBAL`, :const:`RTLD_NOW` or :const:`RTLD_LAZY`.
    :param bool use_errno: Whether called functions should use a ctypes' thread-local errno copy.
    :return: A handle to the loaded library.
    :rtype: CDLL
    """
    # fix find_library on Python 2.7
    os.environ['LIBRARY_PATH'] = os.getenv('LD_LIBRARY_PATH', '')

    library = find_library(library_name)
    if library_name is not None and library is None:
        rospy.logfatal("Could not find shared library " + library_name)
        return None

    return CDLL(library, mode=mode, use_errno=use_errno)


def get_ro_c_buffer(buf, buf_len=None):
    """Return a read-only ctypes byte buffer object that points to the given buffer's memory or its copy.

    :param [list|tuple|BufferStringIO|bytes] buf: The source buffer.
    :param int buf_len: Length of the buffer (if `None`, `len(buf)` is used if it exists, or stream length).
    :return: The ctypes c_uint8 array.
    """
    if isinstance(buf, list) or isinstance(buf, tuple):
        if buf_len is None:
            buf_len = len(buf)
        return (c_uint8 * buf_len).from_buffer(bytearray(buf))
    elif hasattr(buf, "getvalue"):  # check for BufferStringIO instance cannot be done directly
        if buf_len is None:
            pos = buf.tell()
            buf.seek(0, os.SEEK_END)
            buf_len = buf.tell()
            buf.seek(pos)
        return (c_uint8 * buf_len).from_buffer_copy(buf.getvalue())
    else:
        return cast(buf, POINTER(c_uint8))


def c_array(data, c_type):
    """Convert the given Python iterable to a C array with zero element after the last one.

    :param list data: The data to convert.
    :param type c_type: The ctypes type of the individual elements.
    :return: The ctypes array (it is one element longer than the input).
    """
    c_data = (c_type * (len(data) + 1))()
    c_data[:-1] = data
    c_data[-1] = 0
    return c_data


__libc = None


def get_libc():
    global __libc
    if __libc is None:
        __libc = load_library("c", use_errno=True)
    return __libc


__getenv = None
__setenv = None
__unsetenv = None
__setlocale = None


def libc_getenv(name, default_value=None):
    """Call libc function `getenv()` to get a C program environment variable value.

    :param str name: Name of the variable.
    :param default_value: Default value to return if the variable is not found.
    :type default_value: str, optional
    :return: The variable (as Python string) or the default value if the variable is not set.
    :rtype: str, optional
    """
    global __getenv
    if __getenv is None:
        libc = get_libc()
        __getenv = libc.getenv
        __getenv.argtypes = [c_char_p]
        __getenv.restype = c_char_p
    fsencode = getattr(os, 'fsencode', lambda x: x)  # Py2 doesn't have fsencode
    fsdecode = getattr(os, 'fsdecode', lambda x: x)  # Py2 doesn't have fsdecode
    set_errno(0)
    res = __getenv(fsencode(name))
    if res is None:
        return default_value
    return fsdecode(res)


def libc_setenv(name, value, overwrite=True):
    """Call libc function `setenv()` to set the C program environment.

    :param str name: Name of the variable.
    :param str value: Value of the variable.
    :param bool overwrite: Whether to overwrite an existing variable or return True without overwriting it.
    :return: True on success.
    :rtype: bool
    :raises OSError: If there is not enough memory or `name` contains '='.
    """
    global __setenv
    if __setenv is None:
        libc = get_libc()
        __setenv = libc.setenv
        __setenv.argtypes = [c_char_p, c_char_p, c_bool]
        __setenv.restype = int
    fsencode = getattr(os, 'fsencode', lambda x: x)  # Py2 doesn't have fsencode
    set_errno(0)
    res = __setenv(fsencode(name), fsencode(value), overwrite)
    if res != 0:
        raise OSError(get_errno(), os.strerror(get_errno()))
    return True


def libc_unsetenv(name):
    """Call libc function `unsetenv()` to unset the C program environment.

    :param str name: Name of the variable.
    :return: True on success.
    :rtype: bool
    :raises OSError: If `name` contains '='.
    """
    global __unsetenv
    if __unsetenv is None:
        libc = get_libc()
        __unsetenv = libc.unsetenv
        __unsetenv.argtypes = [c_char_p]
        __unsetenv.restype = int
    fsencode = getattr(os, 'fsencode', lambda x: x)  # Py2 doesn't have fsencode
    set_errno(0)
    res = __unsetenv(fsencode(name))
    if res != 0:
        raise OSError(get_errno(), os.strerror(get_errno()))
    return True


class Allocator(object):
    """An allocator that can be passed as a callback to a C function that should return dynamically allocated data.

    This way, Python can still manage the lifetime of the buffer, and it is not needed to know the size of the buffer
    prior to calling the C function.

    :ivar List[c_void_p] allocated: The allocated buffers.
    :ivar List[int] allocated_sizes: Sizes of the allocated buffer in bytes.
    """

    ALLOCATOR = CFUNCTYPE(c_void_p, c_size_t)
    """The ctypes signature of the allocator callback. The corresponding C signature is
       `typedef void*(*allocator_t)(size_t);`"""

    def __init__(self):
        self.allocated = []
        self.allocated_sizes = []

    def __call__(self, size):
        self.allocated.append(self._alloc(size))
        self.allocated_sizes.append(size)
        return cast(self.allocated[-1], c_void_p).value

    def _alloc(self, size):
        """Actually allocate a buffer of the given size and return it.

        Descendant classes have to implement this method.

        :param int size: Size of the buffer in bytes.
        :return: The buffer.
        :rtype: c_void_p
        """
        raise NotImplementedError()

    def get_cfunc(self):
        """Get the ctypes object referring to this allocator.

        :return: The ctypes-compatible allocator object (that should be passed as function argument).
        :rtype: Allocator.ALLOCATOR
        """
        return Allocator.ALLOCATOR(self)

    @property
    def value(self):
        """Get the first allocated value converted to a Python object.

        :return: The allocated value.
        """
        if len(self.allocated) == 0:
            return None
        return self.allocated[0].value

    @property
    def values(self):
        """Get the allocated values converted to Python objects.

        :return: The allocated values.
        """
        return [a.value for a in self.allocated]


class StringAllocator(Allocator):
    """ctypes allocator suitable for allocating strings. The returned value is of `str` type."""

    def _alloc(self, size):
        return create_string_buffer(size)

    @property
    def value(self):
        if len(self.allocated) == 0:
            return None
        if sys.version_info[0] == 2:
            return self.allocated[0].value
        else:
            return self.allocated[0].value.decode('utf-8')

    @property
    def values(self):
        return [(a.value if sys.version_info[0] == 2 else a.value.decode('utf-8')) for a in self.allocated]


class BytesAllocator(Allocator):
    """ctypes allocator suitable for allocating byte arrays. The returned value is a bytes object."""

    def _alloc(self, size):
        return (c_uint8 * size)()

    @property
    def value(self):
        if len(self.allocated) == 0:
            return None
        return string_at(self.allocated[0], self.allocated_sizes[0])

    @property
    def values(self):
        return [string_at(a, s) for a, s in zip(self.allocated, self.allocated_sizes)]


class ScalarAllocator(Allocator):
    """ctypes allocator suitable for allocating scalar values (e.g. numbers). The returned value is a scalar value.

    This means the size of each allocation needs to exactly match the number of bytes the given `c_type` has. If other
    sizes are requested, :class:`RuntimeError` is thrown.
    """

    def __init__(self, c_type):
        super(ScalarAllocator, self).__init__()
        self._c_type = c_type

    def _alloc(self, size):
        if size != sizeof(self._c_type):
            raise RuntimeError("ScalarAllocator can only handle size 1 allocations.")
        return (self._c_type * 1)()

    @property
    def value(self):
        if len(self.allocated) == 0:
            return None
        return self.allocated[0][0]

    @property
    def values(self):
        return [a[0] for a in self.allocated]


class RosMessagesAllocator(BytesAllocator):
    """ctypes allocator suitable for allocating byte buffers for serialized ROS messages."""

    def __init__(self, msg_type):
        """Create the message allocator.

        :param Type msg_type: Type of the messages.
        """
        super(RosMessagesAllocator, self).__init__()
        self.msg_type = msg_type

    @property
    def message(self):
        """Return the first ROS message allocated by this allocator.

        :return: The log message.
        :rtype: genpy.Message
        """
        if len(self.allocated) == 0:
            return None
        return self.messages[0]

    @property
    def messages(self):
        """Return the ROS messages allocated by this allocator.

        :return: The log messages.
        :rtype: collections.Iterable[genpy.Message]
        """
        for msg in self.values:
            m = self.msg_type()
            m.deserialize(msg)
            yield m


class LogMessagesAllocator(RosMessagesAllocator):
    """ctypes allocator suitable for allocating byte buffers for serialized :rosgraph_msgs:`Log` messages and printing
    them via standard `rospy` logging mechanisms."""

    def __init__(self):
        from rosgraph_msgs.msg import Log
        super(LogMessagesAllocator, self).__init__(Log)
        try:
            from cStringIO import StringIO
        except ImportError:
            from io import StringIO
        self.stream = StringIO()
        self.stream_handler = RosStreamHandler(colorize=False, stdout=self.stream, stderr=self.stream)

    def format_log_message(self, msg):
        """Convert the log message to a string to be logged.

        :param rosgraph_msgs.msg.Log msg: The log message.
        :return: The text to be logged.
        :rtype: str
        """
        from cras.log_utils import log_level_ros_to_py, log_level_names
        record = LogRecord(msg.name, log_level_ros_to_py[msg.level], msg.file, msg.line, msg.msg, None, None,
                           func=msg.function)
        self.stream.truncate(0)
        self.stream.seek(0)
        self.stream_handler.emit(record)
        message = self.stream.getvalue().rstrip("\n")
        # get rid of the [INFO] part of the message as it will be duplicated by the rospy logging call
        message = message.replace(log_level_names[msg.level], "")
        message = message.replace("[]: ", "").replace("[] ", "").replace("[]", "")
        return message

    def get_formatted_log_messages(self):
        """Get all messages allocated by this allocator converted to strings to be printed.

        :return: Tuples of (loglevel, message string).
        :rtype: collections.Iterable[Tuple[int, str]]
        """
        for msg in self.messages:
            yield msg.level, self.format_log_message(msg)

    def print_log_messages(self):
        """Print all messages allocated by this allocator using the standard rospy logger."""
        from cras.log_utils import log_level_ros_to_py_name
        for level, msg in self.get_formatted_log_messages():
            # Call _base_logger directly so that the stack unwinding correctly points to the line that called this
            # function.
            rospy.core._base_logger(msg, (), {}, level=log_level_ros_to_py_name[level].lower())
