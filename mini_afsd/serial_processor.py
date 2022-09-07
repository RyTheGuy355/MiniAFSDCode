# -*- coding: utf-8 -*-
"""Class and functions for communicating with serial ports and a Labjack."""

import threading
import time

import serial


class SerialProcessor:
    """An object for controlling communication to the mill through a serial port."""

    def __init__(self, controller, port=None, measure_force=True, skip_home=False):
        """
        Initializes the serial port processor.

        Parameters
        ----------
        controller : _type_
            _description_
        port : _type_, optional
            _description_, by default None
        measure_force : bool, optional
            _description_, by default True
        skip_home : bool, optional
            If True, the serial port will send b'$X' to skip homing and directly
            be ready to send commands. If False (default), b'$X' or b'$H' (home)
            will have to be sent manually through the serial port to begin using
            the mill.
        """
        self.controller = controller
        self.esp = None
        self.port = port
        self.buffer_length = 15
        self.close_port = threading.Event()
        self.waitingForAck = threading.Event()
        self.commandInvalid = threading.Event()
        self.serialUnlocked = threading.Event()
        self.serialUnlocked.set()

        self.forceData = []
        self.espBuffer = []
        self.espTypeBuffer = []

        if skip_home:
            self.sendCode(b'$X', 0)

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
                # TODO is baudrate fixed or variable?
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

        self.status_updater = threading.Thread(target=self.status_update, daemon=True)
        self.status_updater.start()

    def serialListen(self):
        """The event loop for the thread that reads messages from the serial port."""
        print("Starting serial listener")
        while not self.close_port.is_set():  # TODO put a try-except in for potential errors
            if self.esp.in_waiting and self.serialUnlocked.is_set():
                data = self.esp.read_until(b'\n')
                data.strip(b'\n')
                if (b'\r' in data):
                    data = data.strip(b'\r')
                if not data:
                    continue
                print(data)

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
                for i, (bufferValue, typeValue) in enumerate(
                    zip(self.espBuffer, self.espTypeBuffer)
                ):
                    setAck = not self.waitingForAck.is_set() and self.controller.running.is_set()
                    if typeValue == 0 or setAck:
                        # TODO change logic and remove setAck; instead, check actual buffer length
                        # is > 8 (buffer starts at 15; 0-indexed)
                        self.waitingForAck.set()
                        self.esp.write(bufferValue)
                        self.esp.write(b'\n')
                        while not self.waitingForAck.is_set() or self.commandInvalid.is_set():  # TODO maybe have different flag for if a response is detected
                            time.sleep(0.01)
                        if not self.commandInvalid.is_set():
                            self.espBuffer.pop(i)
                            self.espTypeBuffer.pop(i)
                            if setAck:
                                self.waitingForAck.set()
                        break

            time.sleep(0.01)

    def parse_state_message(self, message):
        """
        Parses the state message output by the serial port by sending b'?'.


        Updates the state, positions, and buffer length in the GUI depending
        on the message contents.

        Parameters
        ----------
        message : bytes
            The state message from the serial port.
        """
        state = None
        machine_position = None
        work_position = None
        buffer_length = None
        total_message = message.decode().split('|')
        # message is sent as
        # b'<state|machine positions: x, y, z, a|BF:buffer size|FS:?,?|Work positions(optional):x,y,z,a>'
        total_message[0] = total_message[0].lstrip('<')
        total_message[-1] = total_message[-1].rstrip('>')
        for entry in total_message:
            if ':' not in entry:  # machine state
                state = entry
            else:
                # headers are 'MPos', 'Bf', 'FS', 'WCO', 'Ov', 'Pn'
                try:
                    header, values = entry.split(':')
                except ValueError:  # message has multiple : at startup
                    break
                print(header, values)
                if header == 'MPos':
                    machine_position = [float(val) for val in values.split(',')]
                elif header == 'WCO':
                    work_position = [float(val) for val in values.split(',')]
                elif header == 'Bf':
                    buffer_length = values.split(',')[0]
        print(len(total_message))
        print(total_message)
        print(state)
        if machine_position is not None:
            xSign = "+" if machine_position[0] >= 0 else ""
            ySign = "+" if machine_position[1] >= 0 else ""
            zSign = "+" if machine_position[2] >= 0 else ""

            self.controller.gui.xAbsVar.set(f'{xSign}{machine_position[0]:.3f}')
            self.controller.gui.yAbsVar.set(f'{ySign}{machine_position[1]:.3f}')
            self.controller.gui.zAbsVar.set(f'{zSign}{machine_position[2]:.3f}')
            self.controller.gui.aAbsVar.set(f'{machine_position[3]:.3f}')

            print(machine_position)
        if work_position is not None:
            xSign = "+" if work_position[0] >= 0 else ""
            ySign = "+" if work_position[1] >= 0 else ""
            zSign = "+" if work_position[2] >= 0 else ""

            self.controller.gui.xRelVar.set(f'{xSign}{work_position[0]:.3f}')
            self.controller.gui.yRelVar.set(f'{ySign}{work_position[1]:.3f}')
            self.controller.gui.zRelVar.set(f'{zSign}{work_position[2]:.3f}')
            self.controller.gui.aRelVar.set(f'{work_position[3]:.3f}')

            print(work_position)
        if buffer_length is not None:
            self.controller.gui.bufferVar.set(buffer_length)
            self.buffer_length = buffer_length
            print(buffer_length)

        if state is not None:
            self.controller.gui.stateVar.set(state)

    def status_update(self):
        """Sends and receives querries to the port to receive the position and state of the mill."""
        while not self.close_port.wait(timeout=0.5):
            # TODO add b'?' to buffer and query if serialListener has received the output

            self.serialUnlocked.wait()
            self.serialUnlocked.clear()
            self.esp.write(b'?')
            self.esp.write(b'/n')
            while not self.esp.in_waiting:
                time.sleep(0.001)
            self.parse_state_message(self.esp.read_until(b'\n').strip(b'\r\n'))
            self.serialUnlocked.set()

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

    def goToZero(self, axis):
        """
        Goes to the zero point for the given axis.

        Parameters
        ----------
        axis : {b'X', b'Y', b'Z', b'A'}
            The byte designating which axis to move to zero: X, Y, Z, or A.
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
