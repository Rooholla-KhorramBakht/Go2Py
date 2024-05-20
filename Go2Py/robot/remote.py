import threading
from pynput import keyboard


class BaseRemote:
    def __init__(self):
        pass

    def startSeq(self):
        return False

    def standUpDownSeq(self):
        return False

    def flushStates(self):
        pass


remote = BaseRemote()


class KeyboardRemote(BaseRemote):
    def __init__(self):
        super().__init__()
        self.start_seq_flag = False
        self.stand_up_down_seq_flag = False
        self.listener_thread = threading.Thread(target=self._listen_to_keyboard)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def _on_press(self, key):
        try:
            if key.char == 's':  # Start sequence
                self.start_seq_flag = True
            elif key.char == 'u':  # Stand up/down sequence
                self.stand_up_down_seq_flag = True
        except AttributeError:
            pass  # Special keys (like space) will be handled here

    def _on_release(self, key):
        try:
            if key.char == 's':  # Start sequence
                self.start_seq_flag = False
            elif key.char == 'u':  # Stand up/down sequence
                self.stand_up_down_seq_flag = False
        except AttributeError:
            pass  # Special keys (like space) will be handled here

    def _listen_to_keyboard(self):
        with keyboard.Listener(on_press=self._on_press, on_release=self._on_release) as listener:
            listener.join()

    def startSeq(self):
        if self.start_seq_flag:
            self.start_seq_flag = False
            return True
        return False

    def standUpDownSeq(self):
        if self.stand_up_down_seq_flag:
            self.stand_up_down_seq_flag = False
            return True
        return False

    def flushStates(self):
        self.stand_up_down_seq_flag = False
        self.start_seq_flag = False


class UnitreeRemote(BaseRemote):
    def __init__(self, robot):
        self.robot = robot

    def startSeq(self):
        remote = self.robot.getRemoteState()
        if remote.btn.start:
            return True
        else:
            return False

    def standUpDownSeq(self):
        remote = self.robot.getRemoteState()
        if remote.btn.L2 and remote.btn.A:
            return True
        else:
            return False

    def getEstop(self):
        remote = self.robot.getRemoteState()
        if remote.btn.L2 and remote.btn.R2:
            return True
        else:
            return False
