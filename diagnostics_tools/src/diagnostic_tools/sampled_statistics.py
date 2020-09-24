
class SampledStatistics:

    def __init__(self, zero):
        self._zero = zero
        self.reset()

    def reset(self):
        self._average = self._zero()
        self._minimum = self._zero()
        self._maximum = self._zero()
        self._sample_count = 0

    def update(self, sample):
        self._average = (
            sample + self._average * self._sample_count
        ) / (self._sample_count + 1)
        if self._sample_count > 0:
            if self._maximum < sample:
                self._maximum = sample
            elif self._minimum > sample:
                self._minimum = sample
        else:
            self._minimum = self._maximum = sample
        self._sample_count += 1

    @property
    def sample_count(self):
        return self._sample_count

    @property
    def average(self):
        return self._average

    @property
    def maximum(self):
        return self._maximum

    @property
    def minimum(self):
        return self._minimum
