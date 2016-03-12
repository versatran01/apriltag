import numpy as np
import matplotlib.pyplot as plt


class DisjointSets(object):
    def __init__(self, n):
        self._parent = np.arange(n, dtype=int)
        self._size = np.ones(n, dtype=int)

    def find(self, id):
        # terminal case: a node is it's own parent
        parent_id = self._parent[id]
        if parent_id == id:
            return id
        # otherwise, recurse
        root = self.find(parent_id)
        # short circuit the path / path compression
        self._parent[id] = root
        return root

    def union(self, id0, id1):
        root0 = self.find(id0)
        root1 = self.find(id1)

        # two nodes in the same set
        if root0 == root1:
            return root0

        size0 = self._size[root0]
        size1 = self._size[root1]

        if size0 > size1:
            self._parent[root1] = root0
            self._size[root0] += size1
            return root0
        else:
            self._parent[root0] = root1
            self._size[root1] += size0
            return root1

    def set_size(self, id):
        return self._size[self.find(id)]


def imshow(*args, **kwargs):
    n = len(args)
    figsize = kwargs.get('figsize', (7.5, 7.5))
    cmap = kwargs.get('cmap', plt.cm.gray)
    title = kwargs.get('title', [])
    if isinstance(title, basestring):
        title = [title]

    f, axes = plt.subplots(nrows=1, ncols=n, figsize=figsize)
    axes = np.atleast_1d(axes)
    for ax, im in zip(axes, args):
        ax.imshow(im, cmap=cmap)

    for ax, t in zip(axes, title):
        ax.set_title(t)
    return np.ravel(axes)


def mod2pi(a, b=0):
    theta = a - b

    twopi = np.pi * 2.0
    twopi_inv = 1.0 / twopi

    abs_theta = abs(theta)
    q = abs_theta * twopi_inv + 0.5
    qi = int(q)

    r = abs_theta - qi * twopi
    if theta < 0:
        return -r
    else:
        return r


def angle_dist(a, b=0):
    return np.pi - abs(abs(a - b) - np.pi)
