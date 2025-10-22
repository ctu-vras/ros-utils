# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Object-oriented wrapper around heapq with support for a custom sort key."""

import heapq
import itertools


class Heap(object):
    """A heap object that supports a custom sort key."""
    # Inspiration taken from https://stackoverflow.com/a/8875823/1076564

    def __init__(self, initial=None, key=lambda x: x):
        self.key = key
        self._counter = itertools.count(start=len(initial) if initial else 0)
        self.index = next(self._counter)

        if initial:
            self._data = [(key(item), i, item) for i, item in enumerate(initial)]
            heapq.heapify(self._data)
        else:
            self._data = []

    def push(self, item):
        heapq.heappush(self._data, (self.key(item), self.index, item))
        self.index = next(self._counter)

    def pop(self):
        return heapq.heappop(self._data)[2]

    def peek(self):
        return self._data[0][2]

    def __len__(self):
        return len(self._data)

    def __str__(self):
        return str(self._data)

    def __repr__(self):
        return repr(self._data)
