# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Bag file reader that shows progressbars when loading index or reading messages."""

import os
from tqdm import tqdm

import rosbag

from .bag_utils import MultiBag


class _TqdmBagReader200(rosbag.bag._BagReader200):  # noqa
    def _read_connection_index_records(self):
        chunks = self.bag._chunks
        self.bag._chunks = tqdm(self.bag._chunks, total=len(self.bag._chunks), desc="Reading bag index", unit="chunks")
        super(_TqdmBagReader200, self)._read_connection_index_records()
        self.bag._chunks = chunks


class TqdmBag(rosbag.bag.Bag):
    """Drop-in replacement for :py:class:`rosbag.bag.Bag` which shows nice progressbars when loading index and reading
    messages."""

    def read_index(self):
        """Force reading index if it was skipped in the constructor."""

        if not self._connection_indexes_read and self._version == 200:  # read bag index with a nice progressbar
            self._reader._read_connection_index_records()  # noqa

    def _create_reader(self):
        super(TqdmBag, self)._create_reader()
        if self._version == 200:  # read bag index with a nice progressbar
            self._reader = _TqdmBagReader200(self)

    def read_messages(self, topics=None, start_time=None, end_time=None, connection_filter=None, raw=False,
                      return_connection_header=False):
        msg_count = self.get_message_count(topics)
        basename = os.path.basename(self.filename)
        msg_iter = super(TqdmBag, self).read_messages(
            topics, start_time, end_time, connection_filter, raw, return_connection_header)
        return tqdm(msg_iter, total=msg_count, desc=basename, unit='msg')


class TqdmMultiBag(MultiBag):
    """Generalization of TqdmBag which can read multiple bags simultaneously."""

    def open_bag(self, b, compression, mode, options, skip_index):
        return TqdmBag(b, mode, compression, options=options, skip_index=skip_index)

    def read_messages(self, topics=None, start_time=None, end_time=None, connection_filter=None, raw=False,
                      return_connection_header=False):
        msg_count = self.get_message_count(topics, start_time, end_time)
        basename = os.path.basename(self.bags[0].filename)
        msg_iter = self._read_messages(topics, start_time, end_time, connection_filter, raw, return_connection_header)
        return tqdm(msg_iter, total=msg_count, desc=basename, unit='msg')


__all__ = [
    TqdmBag.__name__,
    TqdmMultiBag.__name__,
]
