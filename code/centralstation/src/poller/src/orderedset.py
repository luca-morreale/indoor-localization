from collections import MutableSet
from threading import Lock


class OrderedSet(MutableSet):
    def __init__(self, iterable=None):
        self.end = end = []
        end += [None, end, end]         # sentinel node for doubly linked list
        self.map = {}                   # key --> [key, prev, next]
        if iterable is not None:
            self |= iterable
        self.lock = Lock()

    def __len__(self):
        return len(self.map)

    def __contains__(self, key):
        return key in self.map

    def isEmpty(self):
        with self.lock:
            return self.__len__() == 0

    def add(self, key):      # append at rightmost position
        with self.lock:
            if key not in self.map:
                end = self.end
                curr = end[1]
                curr[2] = end[1] = self.map[key] = [key, curr, end]

    def discard(self, key):
        with self.lock:
            self.__remove__(key)

    def __iter__(self):
        end = self.end
        curr = end[2]
        while curr is not end:
            yield curr[0]
            curr = curr[2]

    def __reversed__(self):
        end = self.end
        curr = end[1]
        while curr is not end:
            yield curr[0]
            curr = curr[1]

    def pop(self):      # remove leftmost element
        if not self:
            raise KeyError('set is empty')

        with self.lock:
            key = self.end[-1][0]
            self.__remove__(key)
        return key

    def __remove__(self, key):
        if key in self.map:
            key, prev, next = self.map.pop(key)
            prev[2] = next
            next[1] = prev

    def __repr__(self):
        if not self:
            return '%s()' % (self.__class__.__name__,)
        return '%s(%r)' % (self.__class__.__name__, list(self))

    def __eq__(self, other):
        if isinstance(other, OrderedSet):
            return len(self) == len(other) and list(self) == list(other)
        return set(self) == set(other)