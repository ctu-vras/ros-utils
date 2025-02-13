#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for plugin_utils."""

import os
import sys

import rostest
import unittest

from cras.plugin_utils import find_plugins


class PluginUtils(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(PluginUtils, self).__init__(*args, **kwargs)

    def test_find_plugins(self):
        plugins = list(find_plugins("image_transport", "plugin"))
        self.assertLessEqual(1, len(plugins))
        for pkg, plugin in plugins:
            if pkg == "image_transport":
                self.assertTrue(plugin.endswith("/default_plugins.xml"))
                break
        else:
            self.fail("Test package image_transport not found")


if __name__ == '__main__':
    rostest.rosrun("cras_py_common", "test_plugin_utils", PluginUtils)
