class Calibrator():
    @property
    def error(self):
        return self.currentVal

    @property
    def lastError(self):
        return self.lastVal

    def __init__(self, init_tuning:float, threshold:float):
        self.init_tuning = self.crawlingSpeed = init_tuning
        self.threshold = threshold
        self.currentVal = 0
        self.lastVal = 0
        self.output = 0

    def reset(self):
        self.crawlingSpeed = self.init_tuning
        self.currentVal = 0
        self.lastVal = 0
        self.output = 0

    def compute(self, currentVal:float):
        """
        @brief Compute one iteration of the calibration algorythm
        @returns True if the threshold has been reached
        @param currentVal new process value
        """
        self.currentVal = currentVal # Update process value

        # if (abs(self.error) < self.threshold): # Threshold reached
        #     return True
        if (self.lastError > 0) == (self.error > 0): # Has not overshot
            # If last attempt made the error bigger, we are going the wrong way, flip the crawling direction
            self.crawlingSpeed = self.crawlingSpeed if abs(self.lastError) - abs(self.error) > 0 else -self.crawlingSpeed
        else: # Overshot
            self.crawlingSpeed *= -0.8 # Reverse and reduce crawling speed
        
        self.output += self.crawlingSpeed # Update output
        self.lastVal = currentVal # Update last value
        return False # Threshold not reached