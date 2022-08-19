# -*- coding: utf-8 -*-
"""Class and functions for communicating with serial ports and a Labjack."""

import threading
import time

import serial


class SerialProcessor:
    """An object for controlling communication to the mill through a serial port."""

    def __init__(self, controller, port=None, measure_force=True):
        self.controller = controller
        self.esp = None
        self.labjackHandle = None
        self.port = port
        self.close_port = threading.Event()
        self.waitingForAck = threading.Event()

        self.forceData = []
        self.espBuffer = []
        self.espTypeBuffer = []

    @property
    def port(self):
        """The current port for the processor.

        Returns
        -------
        str or None
            The current port.
        """
        return self._port

    @port.setter
    def port(self, port):
        """Sets the COM port and tries to initiate communication.

        Parameters
        ----------
        port : str or None
            The port to connect to. If None, no connection attempt will be made; otherwise,
            the port will be opened for communication.

        Raises
        ------
        serial.SerialException
            Raised if the port connection failed.
        """
        self._port = port
        if self._port is None:
            self.esp = None
        else:
            try:
                # TODO is baudrate fixed or variable? can be set on esp, but can be variable
                self.esp = serial.Serial(port=self._port, baudrate=115200, timeout=1)
            except serial.SerialException:  # wrong port selected
                self._port = None
                raise
            else:
                self.start_threads()

    def start_threads(self):
        """Spawns the threads for communicating with the serial ports and Labjack."""
        self.listener = threading.Thread(target=self.serialListen, daemon=True)
        self.listener.start()

        self.reporter = threading.Thread(target=self.serialReporter, daemon=True)
        self.reporter.start()

    def serialListen(self):
        """The event loop for the thread that reads messages from the serial port."""
        print("Starting serial listener")
        while not self.close_port.is_set():
            data = self.esp.read_until(b'\x03\x04')
            if len(data) > 0:
                if data[0] == 1:
                    if data[1] == 8:
                        print("CTG")
                        self.waitingForAck.clear()
                    else:
                        xPos = data[2:6]
                        yPos = data[6:10]
                        aPos = data[10:14]
                        if data[1] == 70:  # ACSII F
                            aForce = data[14:16]
                            code = b'F'
                        else:  # ACSII P
                            aForce = None
                            code = b'P'
                        self.controller.gui.display(aForce, xPos, yPos, aPos, code, data)
                elif data[0] == 6:
                    print("Ack")
                else:
                    print(data.removesuffix(b'\x03\x04'))

        print("Stopping serial listener")
        self.esp.flush()
        self.esp.close()
        try:
            self.controller.gui.sBut["state"] = "disabled"
        except Exception:
            print("Window appears to be closed")

    def serialReporter(self):
        """The event loop for the thread that sends messages to the serial port."""
        print("Starting serial reporter")
        while self.esp.is_open:
            if len(self.espBuffer) > 0:
                print('reading')
                for i, (bufferValue, typeValue) in enumerate(
                    zip(self.espBuffer, self.espTypeBuffer)
                ):
                    setAck = not self.waitingForAck.is_set() and self.controller.running.is_set()
                    if typeValue == 0 or setAck:
                        self.esp.write(bufferValue)
                        self.esp.write(b'\x04')
                        print(bufferValue)
                        self.espBuffer.pop(i)
                        self.espTypeBuffer.pop(i)
                        if setAck:
                            self.waitingForAck.set()
                        break

            time.sleep(0.01)

    def clear_data(self):
        """Cleans up all of the collected data."""
        self.forceData.clear()

    def close(self):
        """Ensures the serial port is closed correctly."""
        self.close_port.set()
        if self.esp is not None:
            self.esp.write(b"S")
            self.esp.write(b"\x03\x04")
            self.esp.flush()
            self.esp.close()

    def zeroCord(self, axis):
        """
        Sets the current position as the zero point for the given axis.

        Parameters
        ----------
        axis : {b'A', b'X', b'Y'}
            The byte designating which axis to zero.
        """
        if self.controller.running.is_set():
            code = b"G92 " + axis + b"0"
            self.sendCode(code, 1)

    def goToZero(self, axis):  # Not Implemented in Arduino Yet
        """
        Goes to the zero point for the given axis.

        Parameters
        ----------
        axis : {b'A', b'X', b'Y'}
            The byte designating which axis to move to zero: X, Y, or A (Z).
        """
        if self.controller.running.is_set():
            message = b"G0 "
            if axis != 3:
                message = message + axis + b"0"
            else:
                message = message + b"X0 Y0 A0"
            self.sendCode(message, 1)

    def sendCode(self, code, type):
        """
        Sends the specified code the the ESP.

        Parameters
        ----------
        code : bytes
            The byte G-code to send to the serial port.
        type : {0, 1}
            0 means send the code immediately and 1 means to wait for acknowledgement.
        """
        self.espBuffer.append(code)
        self.espTypeBuffer.append(type)
        print(len(self.espBuffer))
