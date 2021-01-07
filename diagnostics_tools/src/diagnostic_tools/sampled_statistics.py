"""
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
"""

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
