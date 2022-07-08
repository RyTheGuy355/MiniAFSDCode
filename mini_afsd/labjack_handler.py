# -*- coding: utf-8 -*-
"""Class and functions for communicating with a Labjack."""

import threading

from labjack import ljm


class LabjackHandler:
    """An object for controlling communication to the mill through a serial port."""

    def __init__(self, controller, measure_force=False):
        self.controller = controller
        self.labjackHandle = None
        self.labjackThread = None
        self.measure_force = measure_force

        self.forceData = []
        self.TC_one_Data = []
        self.TC_two_Data = []

        self.start_thread()

    def start_thread(self):
        """Spawns the thread for communicating with the Labjack."""
        try:
            self.labjackHandle = ljm.openS("T7", "ANY", "ANY")
        except Exception as ex:
            if type(ex).__name__ != "LJMError":  # TODO is this trying to catch ljm.LJMError?
                print("No Labjack Connected")
        else:
            self.labjackThread = threading.Thread(target=self.startLabjack, daemon=True)
            print("Connected to LabJack")
            self.labjackThread.start()

    def startLabjack(self):
        """The thread for reading data from the LabJack."""
        numFrames = 2
        addresses = [7002, 7004]  # AIN1, AIN2
        dataTypes = [ljm.constants.FLOAT32, ljm.constants.FLOAT32]
        while True:
            if self.controller.running.wait(timeout=1):
                ljm.eWriteAddress(self.labjackHandle, 1000, ljm.constants.FLOAT32, 2.67)
                while self.controller.running.is_set():
                    try:
                        while not self.controller.readTempData.wait(timeout=0.05):  # TODO what does readTempData mean vs running?
                            if not self.controller.running.is_set():
                                break
                        self.controller.readTempData.clear()
                        results = ljm.eReadAddresses(
                            self.labjackHandle, numFrames, addresses, dataTypes
                        )
                        self.TC_one_Data.append(results[0])
                        self.TC_two_Data.append(results[1])
                        self.controller.gui.tcOneVariable.set(round(results[0], 2))
                        self.controller.gui.tcTwoVariable.set(round(results[1], 2))
                    except KeyboardInterrupt:
                        break
                    except Exception as ex:
                        print(ex)
                        if type(ex).__name__ != "LJMError":
                            break
                        self.controller.readTempData.clear()
                        while not self.controller.readTempData.wait(timeout=0.01):
                            if not self.controller.running.is_set():
                                break
                ljm.eWriteAddress(self.labjackHandle, 1000, ljm.constants.FLOAT32, 0)
            else:
                results = ljm.eReadAddresses(self.labjackHandle, numFrames, addresses, dataTypes)
                self.controller.gui.tcOneVariable.set(round(results[0], 2))
                self.controller.gui.tcTwoVariable.set(round(results[1], 2))

    def clear_data(self):
        """Cleans up all of the collected data."""
        self.forceData.clear()
        self.TC_one_Data.clear()
        self.TC_two_Data.clear()

    def close(self):
        """Ensures the Labjack is closed correctly."""
        if self.labjackHandle is not None:
            ljm.close(self.labjackHandle)
