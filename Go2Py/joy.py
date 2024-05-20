import threading
import time
from time import sleep
import numpy as np
import pygame
from dataclasses import dataclass
from pygame.locals import (
    JOYAXISMOTION,
    JOYBUTTONDOWN,
    JOYBUTTONUP,
    K_ESCAPE,
    KEYDOWN,
    NOEVENT,
    QUIT,
    K_q,
)


@dataclass
class xKeySwitch:
    R1: int
    L1: int
    start: int
    select: int
    R2: int
    L2: int
    F1: int
    F2: int
    A: int
    B: int
    X: bool
    Y: int
    up: int
    right: int
    down: int
    left: int


@dataclass
class xRockerBtn:
    head: list
    btn: xKeySwitch
    lx: float
    rx: float
    ry: float
    L2: float
    ly: float


class PyGameJoyManager:
    def __init__(self, user_callback=None):
        self.dt = 0.01  # Sampling frequency of the joystick
        self.running = False
        self.joy_thread_handle = threading.Thread(target=self.joy_thread)
        # an optional callback that can be used to send the reading values to a
        # user provided function
        self.user_callback = user_callback
        pygame.init()
        self.offset = None

    def joy_thread(self):
        while self.running:
            for event in [
                pygame.event.wait(200),
            ] + pygame.event.get():
                # QUIT             none
                # ACTIVEEVENT      gain, state
                # KEYDOWN          unicode, key, mod
                # KEYUP            key, mod
                # MOUSEMOTION      pos, rel, buttons
                # MOUSEBUTTONUP    pos, button
                # MOUSEBUTTONDOWN  pos, button
                # JOYAXISMOTION    joy, axis, value
                # JOYBALLMOTION    joy, ball, rel
                # JOYHATMOTION     joy, hat, value
                # JOYBUTTONUP      joy, button
                # JOYBUTTONDOWN    joy, button
                # VIDEORESIZE      size, w, h
                # VIDEOEXPOSE      none
                # USEREVENT        code
                if event.type == QUIT:
                    self.stop_daq()
                elif event.type == KEYDOWN and event.key in [K_ESCAPE, K_q]:
                    self.stop_daq()
                elif event.type == JOYAXISMOTION:
                    self.analog_cmd[event.axis] = event.value
                elif event.type == JOYBUTTONUP:
                    self.digital_cmd[event.button] = 0
                elif event.type == JOYBUTTONDOWN:
                    self.digital_cmd[event.button] = 1
                if self.user_callback is not None and event.type != NOEVENT:
                    if self.offset is None:
                        self.user_callback(self.analog_cmd, self.digital_cmd)
                    else:
                        self.user_callback(
                            np.array(self.analog_cmd) - self.offset, self.digital_cmd
                        )

    def start_daq(self, joy_idx):
        # Get the joy object
        assert (
            pygame.joystick.get_count() != 0
        ), "No joysticks detected, you can not start the class"
        assert (
            pygame.joystick.get_count() >= joy_idx
        ), "The requested joystick ID exceeds the number of available devices"
        self.joy = pygame.joystick.Joystick(joy_idx)

        self.analog_cmd = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        self.digital_cmd = [
            self.joy.get_button(i) for i in range(self.joy.get_numbuttons())
        ]
        self.running = True
        self.joy_thread_handle.start()

    def stop_daq(self):
        self.running = False
        self.joy_thread_handle.join()

    def read_raw(self):
        return self.analog_cmd, self.digital_cmd

    def offset_calibration(self):
        analog, _ = self.read_raw()
        offset = np.array(analog)
        print("Put your stick at zero location and do not touch it")
        sleep(1)
        for i in range(2000):
            sleep(0.001)
            analog, _ = self.read_raw()
            offset += np.array(analog)
        self.offset = offset / 2000

    def read_values(self):
        analog, digital = self.read_raw()
        if self.offset is None:
            return analog, digital
        else:
            return analog - self.offset, digital


class Logitech3DPro:
    """
    Handles operations specific to the Logitech 3D Pro joystick.

    This class serves as a wrapper around PyGameJoyManager, offering methods
    for decoding and interpreting joystick values specific to Logitech 3D Pro.

    Attributes:
        joymanager (PyGameJoyManager): Instance of PyGameJoyManager to handle
                                       the joystick events.
        joy_id (int): The identifier for the joystick.
        analog_raw (list of float): The raw analog values from the joystick axes.
        digital_raw (list of int): The raw digital values from the joystick buttons.
    """

    def __init__(self, joy_id=0):
        """
        Initializes a Logitech3DPro object.

        Starts data acquisition and performs initial calibration.

        Parameters:
            joy_id (int): Identifier for the joystick. Default is 0.
        """
        self.joymanager = PyGameJoyManager()
        self.joymanager.start_daq(joy_id)
        print("Calibrating joystick, please do not touch the stick and wait...")
        time.sleep(2)
        self.joymanager.offset_calibration()
        print("Calibration finished.")
        self.joy_id = joy_id

    def decode(self):
        """
        Decode Raw Values from Joystick.

        Decodes the raw analog and digital values from the joystick. The
        decoded values are stored in the object's attributes.
        """
        self.analog_raw, self.digital_raw = self.joymanager.read_values()

    def readAnalog(self):
        """
        Reads Analog Values.

        Retrieves the analog values from the joystick and presents them in a
        dictionary format. Performs decoding before fetching the values.

        Returns:
            dict: A dictionary containing the analog axis values with keys as 'x', 'y', 'z', and 'aux'.
        """
        self.decode()
        data = {
            "x": self.analog_raw[0],
            "y": self.analog_raw[1],
            "z": self.analog_raw[2],
            "aux": self.analog_raw[3],
        }
        return data

    def readDigital(self):
        """
        Reads Digital Values.

        Retrieves the digital button values from the joystick and presents
        them in a dictionary format. Performs decoding before fetching the values.

        Returns:
            dict: A dictionary containing the digital button states with keys as 'shoot', '2', '3',
                  up to '12'.
        """
        self.decode()
        data = {
            "shoot": self.digital_raw[0],
            "2": self.digital_raw[1],
            "3": self.digital_raw[2],
            "4": self.digital_raw[3],
            "5": self.digital_raw[4],
            "6": self.digital_raw[5],
            "7": self.digital_raw[6],
            "8": self.digital_raw[7],
            "9": self.digital_raw[8],
            "10": self.digital_raw[9],
            "11": self.digital_raw[10],
            "12": self.digital_raw[11],
        }
        return data
