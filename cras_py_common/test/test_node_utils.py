#!/usr/bin/env python3

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for node_utils."""

import unittest

import cras


class NodeUtils(unittest.TestCase):

    def test_reset(self):
        with cras.init():
            with cras.Node('test_node'):
                self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
