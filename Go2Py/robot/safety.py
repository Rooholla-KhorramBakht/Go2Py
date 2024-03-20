import time
class SafetyHypervisor():
    def __init__(self, robot):
        self.robot = robot

    def unsafe(self):
        return False
    
    def controlTimeout(self):
        if time.time() - self.robot.latest_command_stamp > 0.1:
            print('controller timeout')
            return True
        else:
            return False 