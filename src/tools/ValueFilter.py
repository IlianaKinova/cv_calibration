from scipy.signal import lfilter
import numpy as np

class ValueFilter:
    """
    Filters the values over time
    """
    def __init__(self, n:int, a:float, iterations:int):
        self.buff = np.zeros((2, iterations))
        self.n = n
        self.a = a
        self.b = [1.0 / n] * n 
        self.iterations = iterations
        self.index = 0

    def write(self, x, y):
        self.buff[0, self.index] = x
        self.buff[1, self.index] = y
        self.index = (self.index + 1) % self.iterations

    @property
    def value(self):
        """
        Filtered value
        """
        filx = lfilter(self.b, self.a, self.buff[0])
        fily = lfilter(self.b, self.a, self.buff[1])
        return filx.mean(), fily.mean()