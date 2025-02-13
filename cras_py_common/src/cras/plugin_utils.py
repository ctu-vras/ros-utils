# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with plugins defined in the `<export>` section of package.xml."""


import sys
from typing import Iterator, Tuple

import rospkg


def find_plugins(package, attribute):
    """Return all plugins defined in available ROS packages.

    E.g. `find_plugins('cras_py_common', 'attr')` will return [`foo`] if there is a package with
    `<exec_depend>cras_py_common</exec_depend>` and `<export><cras_py_common attr="foo"></export>`.

    :param str package: The base package (all packages have to exec_depend on this exact package).
    :param str attribute: The attribute in `<export>` to look for. Its values will be returned.
    :return: A generator of the attribute values defined in `<export>` of relevant packages. The returned items are
             tuples (containing package, value of the attribute).
    :rtype: Iterator[Tuple[str, str]]
    """
    rospack = rospkg.RosPack()
    to_check = rospack.get_depends_on(package, implicit=False)
    to_check.append(package)
    for pkg in to_check:
        try:
            m = rospack.get_manifest(pkg)
            plugin_exports = m.get_export(package, attribute)
            for plugin_export in plugin_exports:
                yield pkg, plugin_export
        except (rospkg.ResourceNotFound, rospkg.InvalidManifest, IOError):
            pass


def get_plugin_implementations(package, attribute, base_class):
    """If a plugin attribute defines the name of a Python module that contains one or more implementations of
    `base_class`, this function returns the implementation classes as Python types and automatically imports their
    modules.

    :param str package: The base package (all packages have to exec_depend on this exact package).
    :param str attribute: The attribute in `<export>` to look for. The values should be FQDN names of Python modules.
    :param base_class: A Python type all implementation classes should derive from.
    :return: A generator of tuples (module name, class name, Python type) for all defined implementations.
    :rtype: Iterator[Tuple[str, str, type]]
    """
    for _, module_name in find_plugins(package, attribute):
        try:
            module = __import__(module_name, fromlist=[""])
            for name, cls in module.__dict__.items():
                if not isinstance(cls, type):
                    continue
                if not issubclass(cls, base_class):
                    continue
                yield module_name, name, cls
        except ImportError as e:
            print("Failed to import module %s: %s" % (module_name, e), file=sys.stderr)
