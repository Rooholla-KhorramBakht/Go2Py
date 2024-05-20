import numpy as np


class HysteresisContactDetector:
    def __init__(self, upper_limit, lower_limit):
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit
        self.contact_state = np.zeros(4)

    def update(self, contact_forces):
        self.contact_state[np.where(contact_forces > self.upper_limit)[0]] = 1
        self.contact_state[np.where(contact_forces < self.lower_limit)[0]] = 0

    def getContactStates(self):
        return self.contact_state
