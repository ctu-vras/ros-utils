# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Efficient data structure to hold a static set of topics with super-fast is-in-set queries.

The querying is done using expressions like `topic in set` or `topic not in set`.
"""


import sys
from typing import Iterable, Optional


class TopicSet(object):
    """Efficient data structure to hold a static set of topics with super-fast is-in-set queries."""

    def __init__(self, items: Optional[Iterable[str]] = None):
        """
        Build the data structure. Adding items later is not supported.

        :param items: The topics to search. Leading slash will be removed.
        """
        self._empty = False
        if items is None or not items:
            self._empty = True
            return
        self._items = tuple(item.lstrip('/') for item in items)
        self._has_marisa = False
        try:
            import marisa
            self._has_marisa = True
            self._marisa_invalid_key_id = marisa.INVALID_KEY_ID
            keyset = marisa.Keyset()
            for item in self._items:
                keyset.push_back(item)
            self._trie = marisa.Trie()
            self._trie.build(keyset)
        except ImportError:
            if not TopicSet._warned:
                print('!!!\n!!!\nInstall python-marisa or python3-marisa for up to 100% speedup!!!\n!!!\n!!!',
                      file=sys.stderr)
                TopicSet._warned = True
            self._set = set(self._items)

    def __contains__(self, item):
        if self._empty:
            return False
        if self._has_marisa:
            return self._trie.lookup(item.lstrip('/')) != self._marisa_invalid_key_id
        return item.lstrip('/') in self._set

    def __iter__(self):
        if self._empty:
            return iter([])
        if self._has_marisa:
            return iter(self._items)
        return iter(self._set)

    def __str__(self):
        return str(self._items) if not self._empty else str(set())

    def __repr__(self):
        return repr(self._items) if not self._empty else repr(set())

    # correctly resolve "if topic_set:" statements
    def __bool__(self):
        return not self._empty and bool(self._items)

    __nonzero__ = __bool__
