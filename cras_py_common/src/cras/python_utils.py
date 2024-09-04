# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""General Python utilities."""

import contextlib
import locale
import os

from .ctypes_utils import libc_setenv, libc_unsetenv


@contextlib.contextmanager
def temp_environ(unset=None, **update):
    """Context manager that temporarily updates the `os.environ` dictionary in-place.

    The implementation is based on https://stackoverflow.com/a/34333710/1076564 (MIT license).

    :param unset: Environment variables to temporarily unset.
    :type unset: list of str or None
    :param dict update: Dictionary of environment variables and values to temporarily set/update.
    """
    env = os.environ
    update = update or {}
    unset = unset or []

    # List of environment variables being updated or removed.
    stomped = (set(update.keys()) | set(unset)) & set(env.keys())
    # Environment variables and values to restore on exit.
    update_after = {k: env[k] for k in stomped}
    # Environment variables and values to unset on exit.
    unset_after = frozenset(k for k in update if k not in env)

    try:
        for name, value in update.items():
            env[name] = value
            libc_setenv(name, value)
        for name in unset:
            env.pop(name, None)
            libc_unsetenv(name)
        yield
    finally:
        for name, value in update_after.items():
            env[name] = value
            libc_setenv(name, value)
        for name in unset_after:
            env.pop(name)
            libc_unsetenv(name)
        pass


@contextlib.contextmanager
def temp_locale(category, localename):
    """Context manager that temporarily changes locale.

    :param int category: Locale category identifier, one of the `LC_xxx` constants.
    :param str localename: System-specific locale identifier. Can be "" for the user-preferred locale or "C" for the
                           minimal locale.
    """
    try:
        old_locale, encoding = locale.getlocale(category)
        locale.setlocale(category, localename)
        yield
    finally:
        if old_locale is not None and encoding is not None:
            locale.setlocale(category, old_locale + "." + encoding)
        else:
            locale.resetlocale(category)
        pass
