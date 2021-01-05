from collections import deque

class SampledStatistics:

    def __init__(self, window_size, dtype=float):
        self._window = deque()
        self._window_size = window_size
        self._dtype = dtype
        self.reset()

    def reset(self):
        self._accumulate = self._dtype()
        self._average = self._dtype()
        self._minimum = self._dtype()
        self._maximum = self._dtype()
        self._window.clear()

    def update(self, sample):
        sample = self._dtype(sample)

        if len(self._window) > 0:
            if self._maximum < sample:
                self._maximum = sample
            elif self._minimum > sample:
                self._minimum = sample
        else:
            self._minimum = self._maximum = sample

        if len(self._window) == self._window_size:
            self._accumulate -= self._window.popleft()
        self._accumulate += sample
        self._window.append(sample)

        self._average = self._accumulate / len(self._window)

    @property
    def sample_count(self):
        return len(self._window)

    @property
    def average(self):
        return self._average

    @property
    def maximum(self):
        return self._maximum

    @property
    def minimum(self):
        return self._minimum
