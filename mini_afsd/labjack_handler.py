# -*- coding: utf-8 -*-
"""Class and functions for communicating with a Labjack."""

import threading
import time

from labjack import ljm


class LabjackHandler:
    """An object for controlling communication to the mill through a serial port."""

    def __init__(self, controller, measure_force=False):
        """
        Initializes the Labjack handler.

        Parameters
        ----------
        controller : _type_
            _description_
        measure_force : bool, optional
            _description_. Default is False.
        """
        self.controller = controller
        self.labjackHandle = None
        self.labjackThread = None
        self.measure_force = measure_force

        self.forceData = []
        self.TC_one_Data = []
        self.TC_two_Data = []

        self.start_threads()

    def start_threads(self):
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
        numFrames = 3
        addresses = [7002, 7004, 26]  # AIN1, AIN2
        dataTypes = [ljm.constants.FLOAT32, ljm.constants.FLOAT32, ljm.constants.FLOAT32]
        while True:
            if self.controller.running.wait(timeout=1):
                ljm.eWriteAddress(self.labjackHandle, 1000, ljm.constants.FLOAT32, 2.67)
                #Made from trial and error, sets the AIN pins for the thermocouples to output compensated temperatures in C
                ljm.eWriteAddresses(
                    self.labjackHandle, 4, [9002, 9004, 9302, 9304],
                    [ljm.constants.UINT32,ljm.constants.UINT32, ljm.constants.UINT32, ljm.constants.UINT32],
                    [22, 22, 1, 1]
                )
                avgNum = 0
                avgResults = [0,0,0]
                numAvg = 10
                while self.controller.running.is_set():
                    try:
                        results = ljm.eReadAddresses(
                            self.labjackHandle, numFrames, addresses, dataTypes
                        )
                        avgResults[0] += results[0]
                        avgResults[1] += results[1]
                        avgResults[2] += results[2]
                        avgNum += 1
                        if (avgNum == numAvg):
                            avgResults[0] /= numAvg
                            avgResults[1] /= numAvg
                            avgResults[2] /= numAvg
                            force = (avgResults[2]-0.5)*333.61
                            self.TC_one_Data.append(avgResults[0])
                            self.TC_two_Data.append(avgResults[1])
                            self.forceData.append(force)
                            self.controller.gui.tcOneVariable.set(round(avgResults[0], 2))
                            self.controller.gui.tcTwoVariable.set(round(avgResults[1], 2))
                            self.controller.gui.display(force)
                            self.controller.timeData.append(
                                round(time.time() - self.controller.startTime, 2)
                            )
                            self.controller.gui.display(force)
                            avgResults = [0,0,0]
                            avgNum = 0
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
                    time.sleep(0.025)
                ljm.eWriteAddress(self.labjackHandle, 1000, ljm.constants.FLOAT32, 0)
            else:
                results = ljm.eReadAddresses(self.labjackHandle, numFrames, addresses, dataTypes)
                self.controller.gui.tcOneVariable.set(round(results[0], 2))
                self.controller.gui.tcTwoVariable.set(round(results[1], 2))
                force =  (results[2]-0.5)*333.61
                self.controller.gui.display(force)

    def clear_data(self):
        """Cleans up all of the collected data."""
        self.forceData.clear()
        self.TC_one_Data.clear()
        self.TC_two_Data.clear()

    def close(self):
        """Ensures the Labjack is closed correctly."""
        if self.labjackHandle is not None:
            ljm.close(self.labjackHandle)
