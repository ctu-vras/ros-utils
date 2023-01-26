# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with the ctypes library."""

from ctypes import CDLL, CFUNCTYPE, c_void_p, c_size_t, cast, create_string_buffer, c_uint8, POINTER, string_at
from ctypes.util import find_library
import os
import sys

import rospy


RTLD_LAZY = 1
"""Relocate dynamic symbols only when they are used."""

RTLD_NOW = 2
"""Relocate dynamic symbols as soon as the library is loaded."""


def load_library(library_name, mode=RTLD_LAZY):
    """Load a C library (.so file) located on LD_LIBRARY_PATH.

    All dependencies of this library have to be loaded before this one with :obj:`mode` =  :const:`ctypes.RTLD_GLOBAL`.

    :param str library_name: Name of the library (without the "lib" prefix and ".so" suffix).
    :param int mode: Symbol loading scope and when they should be loaded. Either :const:`ctypes.RTLD_LOCAL`,
                     :const:`ctypes.RTLD_GLOBAL`, :const:`RTLD_NOW` or :const:`RTLD_LAZY`.
    :return: A handle to the loaded library.
    :rtype: CDLL
    """
    # fix find_library on Python 2.7
    os.environ['LIBRARY_PATH'] = os.getenv('LD_LIBRARY_PATH', '')

    library = find_library(library_name)
    if library is None:
        rospy.logfatal("Could not find shared library " + library_name)
        return None

    return CDLL(library, mode=mode)


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


class Allocator(object):
    """An allocator that can be passed as a callback to a C function that should return dynamically allocated data.

    This way, Python can still manage the lifetime of the buffer, and it is not needed to know the size of the buffer
    prior to calling the C function.

    :ivar c_void_p|None allocated: The allocated buffer.
    :ivar int allocated_size: Size of the allocated buffer in bytes.
    """

    ALLOCATOR = CFUNCTYPE(c_void_p, c_size_t)
    """The ctypes signature of the allocator callback. The corresponding C signature is
       `typedef void*(*allocator_t)(size_t);`"""

    def __init__(self):
        self.allocated = None
        self.allocated_size = 0

    def __call__(self, size):
        self.allocated = self._alloc(size)
        self.allocated_size = size
        return cast(self.allocated, c_void_p).value

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
        """Get the allocated value converted to a Python object.

        :return: The allocated value.
        :rtype: Iterable.
        """
        if self.allocated is None:
            return None
        return self.allocated.value


class StringAllocator(Allocator):
    """ctypes allocator suitable for allocating strings. The returned value is of `str` type."""

    def _alloc(self, size):
        return create_string_buffer(size)

    @property
    def value(self):
        if self.allocated is None:
            return None
        if sys.version_info[0] == 2:
            return self.allocated.value
        else:
            return self.allocated.value.decode('utf-8')


class BytesAllocator(Allocator):
    """ctypes allocator suitable for allocating byte arrays. The returned value is a bytes object."""
    def _alloc(self, size):
        return (c_uint8 * size)()

    @property
    def value(self):
        if self.allocated is None:
            return None
        return string_at(self.allocated, self.allocated_size)
