# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Efficient data structure to hold a static set of topics with super-fast is-in-set queries.

The querying is done using expressions like `topic in set` or `topic not in set`.
"""


from typing import AnyStr, Iterable


class TopicSet(object):
    """Efficient data structure to hold a static set of topics with super-fast is-in-set queries."""

    def __init__(self, items=None):
        """Build the data structure. Adding items later is not supported.

        :param Iterable[AnyStr]|None items: The topics to search.
        """
        self._empty = False
        if items is None or not items:
            self._empty = True
            return
        self._items = items

        # Internally, this library uses MARISA - an efficient trie library
        import marisa
        self._invalid_key_id = marisa.INVALID_KEY_ID
        keyset = marisa.Keyset()
        for item in items:
            keyset.push_back(item)
        self._trie = marisa.Trie()
        self._trie.build(keyset)

    def __contains__(self, item):
        if self._empty:
            return False
        return self._trie.lookup(item) != self._invalid_key_id

    def __str__(self):
        return str(self._items) if not self._empty else str(set())

    def __repr__(self):
        return repr(self._items) if not self._empty else repr(set())

    # correctly resolve "if topic_set:" statements
    def __bool__(self):
        return not self._empty and bool(self._items)

    __nonzero__ = __bool__
