# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when writing ROS-related unit tests in the unittest framework."""

import logging

from .log_utils import log_level_names, log_level_py_to_ros


class RosconsoleCapture(logging.Handler):
    """Helper class for peeking into rosout in a test and checking that the output corresponds to the expected values.

    Usage from a test::

        class MyTest(unittest.TestCase):
            def test_foo(self):
                with RosconsoleCapture(self, Log.INFO, "Test message"):
                    call_foo()

    This example tests that `call_foo()` calls `rospy.loginfo("Test message")`. If not, the test is marked as failed.
    """

    def __init__(self, test, expected_level=None, expected_message=None):
        """
        :param unittest.TestCase test: The test that examines the rosout.
        :param int|Tuple[int] expected_level: Expected log level of the captured message. One of
                                              :class:`rosgraph_msgs.msg.Log` constants. Multiple logged messages may be
                                              captured by passing a tuple.
        :param str|Tuple[str] expected_message: The expected captured message. Multiple logged messages may be captured
                                                by passing a tuple.
        """
        super(RosconsoleCapture, self).__init__()

        self._test = test

        self._expected_level = expected_level if expected_level is not None else list()
        if isinstance(self._expected_level, tuple):
            self._expected_level = list(self._expected_level)
        if not isinstance(self._expected_level, list):
            self._expected_level = [self._expected_level]

        self._expected_message = expected_message if expected_message is not None else list()
        if isinstance(self._expected_message, tuple):
            self._expected_message = list(self._expected_message)
        if not isinstance(self._expected_message, list):
            self._expected_message = [self._expected_message]

        if len(self._expected_level) != len(self._expected_message):
            raise RuntimeError("Length of expected_level and expected_message arguments has to be the same.")
        self._message = list()
        self._level = list()

    def emit(self, record):
        assert isinstance(record, logging.LogRecord)
        self._message.append(record.getMessage())
        self._level.append(log_level_py_to_ros[record.levelno])

    def __enter__(self):
        logging.getLogger('rosout').addHandler(self)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        logging.getLogger('rosout').removeHandler(self)

        if len(self._expected_level) == 0:
            if len(self._level) == 1:
                self._test.fail("Expected no message logged, but a message with level %s and text '%s' was logged." %
                                (log_level_names[self._level[0]], self._message[0]))
            elif len(self._level) > 1:
                self._test.fail("Expected no message logged, but multiple messages with highest level %s were logged." %
                                (log_level_names[max(self._level)]))
        elif len(self._expected_level) != 0 and len(self._level) == 0:
            if len(self._expected_level) == 1:
                self._test.fail("Expected message '%s' with level %s to be logged, but nothing was logged." %
                                (self._expected_message[0], log_level_names[self._expected_level[0]]))
            else:
                self._test.fail("Expected multiple messages with highest level %s to be logged, but nothing was "
                                "logged." % (log_level_names[max(self._expected_level)]))
        elif self._expected_level != self._level:
            self._test.fail("Expected messages with levels (%s), but messages with levels (%s) were logged instead." %
                            (",".join([log_level_names[level] for level in self._expected_level]),
                             ",".join([log_level_names[level] for level in self._level])))
        elif self._expected_message != self._message:
            self._test.fail("Expected messages '%s' to be logged, but messages '%s' were logged instead." %
                            ("\n".join(self._expected_message), "\n".join(self._message)))
