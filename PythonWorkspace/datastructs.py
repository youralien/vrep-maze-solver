from Queue import heapq
from collections import defaultdict
from json import loads, dumps, dump

import sys
sys.setrecursionlimit(10000)

class PriorityQueueSet(object):
    """ Combined priority queue and set data structure. Acts like
        a priority queue, except that its items are guaranteed to
        be unique.

        Provides O(1) membership test, O(log N) insertion and
        O(log N) removal of the smallest item.

        Important: the items of this data structure must be both
        comparable and hashable (i.e. must implement __cmp__ and
        __hash__). This is true of Python's built-in objects, but
        you should implement those methods if you want to use
        the data structure for custom objects.

        Adapted from function found on SO:
        http://stackoverflow.com/questions/407734/a-generic-priority-queue-for-python
    """
    def __init__(self, items=[]):
        """ Create a new PriorityQueueSet.

            items:
                An initial item list - it can be unsorted and
                non-unique. The data structure will be created in
                O(N).
        """
        self.set = dict((item, True) for item in items)
        self.heap = self.set.keys()
        heapq.heapify(self.heap)

    def contains(self, item):
        """ Check if *item* exists in the queue
        """
        return item in self.set

    def empty(self):
        if len(self.set) > 0:
            return False
        else:
            return True

    def get(self):
        """ Remove and return the smallest item from the queue
        """
        smallest = heapq.heappop(self.heap)
        del self.set[smallest]
        return smallest

    def put(self, item):
        """ Add *item* to the queue. The item will be added only
            if it doesn't already exist in the queue.
        """
        if not (item in self.set):
            self.set[item] = True
            heapq.heappush(self.heap, item)


class Tree(defaultdict):
    """ awesome reddit:
    https://gist.github.com/obeleh/4451005
    """
    def __init__(self, parent=None):
        self.parent = parent
        defaultdict.__init__(self, lambda: Tree(self))

    def __str__(self, *args, **kwargs):
        return dumps(self, sort_keys=True, indent=4)

    def rememberChain(self, finish, chain, filepath):
        already_been = []
        found_one = False
        if len(self) > 0:
            for node in self.iterkeys():
                if node not in already_been:
                    already_been.append(node)
                    chain.append(node)
                    if node == str(finish):
                        dump(chain, open(filepath, 'w'))
                        found_one = True
                        return
                    else:
                        if found_one:
                            return
                        else:
                            self[node].rememberChain(finish, chain, filepath)

if __name__ == "__main__":
    a = Tree()
    a['1']['2'] = 3
    a['1']['3']['4'] = 4
    print a['1']['3']
    print a['1']['3'].parent
    print a['1']['3'].parent.parent
    print a['1'].parent
    b = Tree({"1": 1, "2": {"1": 3}})
    print b
    c = Tree('{"1": 1, "2": {"1": 3}}')
    print c