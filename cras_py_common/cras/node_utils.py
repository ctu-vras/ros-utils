# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when writing nodes."""

from types import TracebackType
from typing import List, Optional, Type

import rclpy
import rclpy.node

try:
    # Kilted and newer have InitContextManager, Jazzy not
    from rclpy import InitContextManager
    from rclpy import init
except ImportError:
    class InitContextManager:
        """
        A proxy object for initialization.

        One of these is returned when calling `rclpy.init`, and can be used with context managers to
        properly cleanup after initialization.
        """

        def __init__(self,
                     args: Optional[List[str]],
                     context: Optional[rclpy.Context],
                     domain_id: Optional[int],
                     signal_handler_options: Optional[rclpy.SignalHandlerOptions]) -> None:
            self.context = rclpy.get_default_context() if context is None else context
            if signal_handler_options is None:
                if context is None or context is rclpy.get_default_context():
                    signal_handler_options = rclpy.SignalHandlerOptions.ALL
                else:
                    signal_handler_options = rclpy.SignalHandlerOptions.NO

            if signal_handler_options == rclpy.SignalHandlerOptions.NO:
                self.installed_signal_handlers = False
            else:
                self.installed_signal_handlers = True
            self.context.init(args, domain_id=domain_id)
            # Install signal handlers after initializing the context because the rclpy signal
            # handler only does something if there is at least one initialized context.
            # It is desirable for sigint or sigterm to be able to terminate the process if rcl_init
            # takes a long time, and the default signal handlers work well for that purpose.
            rclpy.install_signal_handlers(signal_handler_options)

        def __enter__(self) -> 'InitContextManager':
            return self

        def __exit__(self,
                     exc_type: Optional[Type[BaseException]],
                     exc_val: Optional[BaseException],
                     exc_tb: Optional[TracebackType]) -> None:
            # The Context class can only be initialized once.  Thus when using the default context,
            # we have to destroy it every time we are done using it and create a new one.
            # utilities.try_shutdown will only replace the default context if we pass 'None', so make
            # sure to do that for the default context.
            shutdown_context = None if self.context is rclpy.get_default_context() else self.context
            rclpy.try_shutdown(context=shutdown_context)
            if (
                    self.installed_signal_handlers or (
                    self.installed_signal_handlers is None and (
                    shutdown_context is None or shutdown_context is rclpy.get_default_context()))
            ):
                rclpy.uninstall_signal_handlers()

    def init(*,
             args: Optional[List[str]] = None,
             context: Optional[rclpy.Context] = None,
             domain_id: Optional[int] = None,
             signal_handler_options: Optional[rclpy.SignalHandlerOptions] = None) -> InitContextManager:
        """
        Initialize ROS communications for a given context.

        :param args: List of command line arguments.
        :param context: The context to initialize. If ``None``, then the default context is used
            (see :func:`.get_default_context`).
        :param domain_id: ROS domain id.
        :param signal_handler_options: Indicate which signal handlers to install.
            If `None`, SIGINT and SIGTERM will be installed when initializing the default context.
        :return: an InitContextManager that can be used with Python context managers to cleanup.
        """
        return InitContextManager(args, context, domain_id, signal_handler_options)


class Node(rclpy.node.Node):
    """
    Base class for all nodes.

    Currently only implements the context manager interface for nodes on Jazzy.
    """

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

    def __enter__(self) -> 'Node':
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        self.destroy_node()
