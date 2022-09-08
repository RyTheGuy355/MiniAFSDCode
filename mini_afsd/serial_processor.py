# -*- coding: utf-8 -*-
"""Class and functions for communicating with serial ports and a Labjack."""

import threading
import time

import serial


class SerialProcessor:
    """
    An object for controlling communication to the mill through a serial port.

    Attributes
    ----------
    esp : serial.Serial or None
        The connected serial port. Is None if not connected to any serial port.
    port : str or None
        The string denoting the connected port. Is None if not connected to a serial port.
    buffer_length : int
        The buffer length of the connected serial port. Initially starts at 15 (0-indexed) and
        reduces to 0 if buffer is full.
    work_offsets : tuple(int, int, int, int)
        The offsets for the x, y, z, and a axes on the mill, respectively.
    close_port : threading.Event

    commandInvalid : threading.Event

    serialUnlocked : threading.Event

    """

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
        self.state = 'Idle'
        self.work_offsets = (0, 0, 0, 0)
        self.close_port = threading.Event()
        self.commandInvalid = threading.Event()
        self.serialUnlocked = threading.Event()
        self.serialUnlocked.set()

        self.forceData = []
        self.espBuffer = []
        self.espTypeBuffer = []

        if skip_home:
            self.sendCode(b'$X', False)

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
                for i, (bufferValue, wait_in_queue) in enumerate(
                    zip(self.espBuffer, self.espTypeBuffer)
                ):
                    # TODO should still check self.controller.running.is_set()?
                    if (not wait_in_queue and self.buffer_length > 1) or self.buffer_length > 8:
                        self.esp.write(bufferValue)
                        self.esp.write(b'\n')
                        self.espBuffer.pop(i)
                        self.espTypeBuffer.pop(i)
                        self.buffer_length += 1
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
                self.state = entry
            else:
                # headers are 'MPos', 'Bf', 'FS', 'WCO', 'Ov', 'Pn'
                try:
                    header, values = entry.split(':')
                except ValueError:  # message has multiple : at startup
                    print(f'Encountered parsing error from: {entry}')
                    break
                if header == 'MPos':
                    machine_position = [float(val) for val in values.split(',')]
                elif header == 'WCO':
                    work_position = [float(val) for val in values.split(',')]
                elif header == 'Bf':
                    buffer_length = int(values.split(',')[0])
        # print(message.decode())
        if machine_position is not None:
            machine_x = machine_position[0]
            machine_y = machine_position[1]
            machine_z = machine_position[2]
            machine_a = machine_position[3]
            xSign = "+" if machine_position[0] >= 0 else ""
            ySign = "+" if machine_position[1] >= 0 else ""
            zSign = "+" if machine_position[2] >= 0 else ""
            aSign = "+" if machine_position[3] >= 0 else ""

            self.controller.gui.xAbsVar.set(f'{xSign}{machine_position[0]:.3f}')
            self.controller.gui.yAbsVar.set(f'{ySign}{machine_position[1]:.3f}')
            self.controller.gui.zAbsVar.set(f'{zSign}{machine_position[2]:.3f}')
            self.controller.gui.aAbsVar.set(f'{aSign}{machine_position[3]:.3f}')

            if work_position is None:
                work_position = self.work_offsets
            else:
                self.work_offsets = work_position

            rel_x, rel_y, rel_z, rel_a = work_position
            work_x = machine_x - rel_x
            work_y = machine_y - rel_y
            work_z = machine_z - rel_z
            work_a = machine_a - rel_a
            xSign = "+" if work_x >= 0 else ""
            ySign = "+" if work_y >= 0 else ""
            zSign = "+" if work_z >= 0 else ""
            aSign = "+" if work_a >= 0 else ""

            self.controller.gui.xRelVar.set(f'{xSign}{work_x:.3f}')
            self.controller.gui.yRelVar.set(f'{ySign}{work_y:.3f}')
            self.controller.gui.zRelVar.set(f'{zSign}{work_z:.3f}')
            self.controller.gui.aRelVar.set(f'{aSign}{work_a:.3f}')

        if buffer_length is not None:
            self.controller.gui.bufferVar.set(buffer_length)
            self.buffer_length = buffer_length

        self.controller.gui.stateVar.set(self.state)

    def status_update(self):
        """Sends and receives querries to the port to receive the position and state of the mill."""
        while not self.close_port.wait(timeout=0.5):
            if not self.controller.running.wait(timeout=0.5):
                continue
            self.serialUnlocked.wait()
            self.serialUnlocked.clear()
            self.esp.write(b'?')
            message = self.esp.read_until(b'\n').strip(b'\r\n')
            self.parse_state_message(message)
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
            self.sendCode(code, False)

    def sendCode(self, code, wait_in_queue):
        """
        Sends the specified code the the ESP.

        Parameters
        ----------
        code : bytes
            The byte G-code to send to the serial port.
        wait_in_queue : bool
            False means send the code immediately and True means to wait for the
            buffer to be open.
        """
        self.espBuffer.append(code)
        self.espTypeBuffer.append(wait_in_queue)
        print(f'added to buffer: {code}')
        print(f'internal buffer: {len(self.espBuffer)}')
