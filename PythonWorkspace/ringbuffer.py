import numpy as np

class RingBuffer():
    """ A 1D ring buffer using numpy arrays
    inspired by: https://scimusing.wordpress.com/2013/10/25/ring-buffers-in-pythonnumpy/
    """
    def __init__(self, length):
        self.data = np.zeros(length, dtype='f')
        self.index = 0
        self.weights = np.ones(self.data.size)

    def append(self, x):
        "adds element x to ring buffer"
        x_index = (self.index - 1) % self.data.size
        self.data[x_index] = x
        self.index += 1

    def extend(self, x):
        "adds array x to ring buffer"
        x_index = (self.index + np.arange(x.size)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

    def get(self):
        "Returns the first-in-first-out data in the ring buffer"
        idx = (self.index + np.arange(self.data.size)) %self.data.size
        return self.data[idx]

    def weighted_average(self, scheme, mathfunc=lambda x: x):
        "choose from same, no weighting"
        if scheme == 'same':
            self.weights = np.ones(self.data.size)
        elif scheme == "last":
            for idx, weight in enumerate(range(1, self.data.size + 1)):
                self.weights[(self.index + idx) % self.data.size] = mathfunc(weight)

        return np.sum(self.data * self.weights) / np.sum(self.weights)

    def __len__(self):
        return self.data.size

    def __iter__(self):
        for count in range(self.index, self.index + self.data.size):
            yield self.data[count % self.data.size]

    def __getitem__(self, idx):
        return self.data[idx % self.data.size]

if __name__ == '__main__':
    rb = RingBuffer(5)

    for i in range(14):
    # for i in np.random.rand(15):
        rb.append(i)
        print rb.index % rb.data.size, rb.data, rb.weighted_average('same'), rb.weighted_average('last'), rb.weighted_average('lastsquared')

    for elem in rb:
        print elem