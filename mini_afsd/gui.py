# -*- coding: utf-8 -*-
"""The User Interface class for interacting with the mill."""

from collections import deque
import csv
from pathlib import Path
import tkinter as tk
from tkinter import filedialog

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt


class Gui:
    """
    The User Interface for controlling the mill.

    Attributes
    ----------
    controller : Controller
        The Controller object that allows communication between the GUI and the
        serial port and LabJack.
    times : tuple
        The time data to use for plotting the temperature versus time graphs. Has
        a fixed length of `display_data`.
    displayData : collections.deque
        The temperature data to use for display. Has a fixed length of `display_data`.
    aAbsVar : tkinter.StringVar
        The label for the absolute position in the actuator direction.
    aRelVar : tkinter.StringVar
        The label for the relative position in the actuator direction.
    xAbsVar : tkinter.StringVar
        The label for the absolute position in the x-direction.
    xRelVar : tkinter.StringVar
        The label for the relative position in the x-direction.
    yAbsVar : tkinter.StringVar
        The label for the absolute position in the y-direction.
    yRelVar : tkinter.StringVar
        The label for the relative position in the y-direction.
    zAbsVar : tkinter.StringVar
        The label for the absolute position in the z-direction.
    zRelVar : tkinter.StringVar
        The label for the relative position in the z-direction.
    topFrame : tkinter.Frame
        The frame for the actuator, traversal, and save frames.
    botFrame : tkinter.Frame
        The frame for the start mill button.
    enXYBut : tkinter.Button
        The button for enabling XY movement.
    """

    def __init__(self, controller, display_size=140, confirm_run=True):
        """
        Initializes the user interface.

        Parameters
        ----------
        controller : Controller
            The controller object for the GUI.
        display_size : int, optional
            The number of data points to display when plotting. Default is 140.
        confirm_run : bool, optional
            If True (default), will ask for confirmation for running a GCode file if
            data collection is not turned on; If False, will directly run the GCode.
        """
        self.controller = controller
        self.times = tuple(t * 3 for t in range(display_size))
        self.displayData = deque([190] * display_size, maxlen=display_size)
        self.gcode_directory = '/'
        self.confirm_run = confirm_run

        self.createMainFrames()
        self.createTraverseFrame()
        self.createActuatorFrame()
        self.createSaveFrame()
        self.createBottomButtons()

    def createMainFrames(self):
        """Creates the main frames for the GUI."""
        self.topFrame = tk.Frame(width=1300, height=600, bg="#e3f0fa")
        self.topFrame.grid(column=0, row=0)
        self.topFrame.grid_propagate(0)

        self.botFrame = tk.Frame(
            width=1300, height=130, bg="#e3f0fa", bd=5, relief="groove", padx=10, pady=10,
        )
        self.botFrame.grid(column=0, row=1)
        self.botFrame.grid_propagate(0)

    def createTraverseFrame(self):
        """Creates the traversal section of the GUI."""
        tFrame = tk.LabelFrame(
            width=530,
            height=600,
            bg="#e3f0fa",
            labelwidget=tk.Label(font=("Times New Roman", 22), text="TRAVERSE", fg="black"),
            bd=5,
            relief="groove",
            padx=10,
            pady=10,
        )
        tFrame.grid(column=0, row=0, in_=self.topFrame)
        tFrame.grid_propagate(0)

        # Create Main Frames
        posFrame = tk.Frame(width=240, height=175, bg="#e3f0fa", padx=2, pady=2,)
        posFrame.grid_propagate(0)
        posFrame.grid(column=0, row=0, in_=tFrame)

        zeroFrame = tk.Frame(width=240, height=200, bg="#e3f0fa", padx=15, pady=10)
        zeroFrame.grid_propagate(0)
        zeroFrame.grid(column=1, row=0, in_=tFrame)

        commandFrame = tk.Frame(width=235, height=220, bg="#e3f0fa",)
        commandFrame.grid_propagate(0)
        commandFrame.grid(column=0, row=1, in_=tFrame)

        # Create Position Frame Widgets
        workLabel = tk.Label(
            text="Work", font=("Times New Roman", 12), fg="black", bg="#e3f0fa", width=12,
        )
        workLabel.grid(column=1, row=0, in_=posFrame)

        machineLabel = tk.Label(
            text="Machine", font=("Times New Roman", 12), fg="black", bg="#e3f0fa", width=8,
        )
        machineLabel.grid(column=2, row=0, in_=posFrame)

        tk.Label(
            text="X", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10,
        ).grid(column=0, row=1, in_=posFrame)

        tk.Label(
            text="Y", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10,
        ).grid(column=0, row=2, in_=posFrame)

        tk.Label(
            text="Z", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10,
        ).grid(column=0, row=3, in_=posFrame)

        tk.Label(
            text="A", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10,
        ).grid(column=0, row=4, in_=posFrame)

        tk.Label(
            text="State", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=5, pady=10,
        ).grid(column=0, row=5, in_=posFrame)

        tk.Label(
            text="Buffer", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=5, pady=10,
        ).grid(column=0, row=6, in_=posFrame)

        self.xRelVar = tk.StringVar(value='+0.000')
        xRelLabel = tk.Label(
            textvariable=self.xRelVar,
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        xRelLabel.grid(column=1, row=1, in_=posFrame)

        self.xAbsVar = tk.StringVar(value='+0.000')
        xAbsLabel = tk.Label(
            textvariable=self.xAbsVar,
            width=6,
            font=("Times New Roman", 14),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=2,
        )
        xAbsLabel.grid(column=2, row=1, in_=posFrame)

        self.yRelVar = tk.StringVar(value='+0.000')
        yRelLabel = tk.Label(
            textvariable=self.yRelVar,
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        yRelLabel.grid(column=1, row=2, in_=posFrame)

        self.yAbsVar = tk.StringVar(value='+0.000')
        yAbsLabel = tk.Label(
            textvariable=self.yAbsVar,
            width=6,
            font=("Times New Roman", 14),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=2,
        )
        yAbsLabel.grid(column=2, row=2, in_=posFrame)

        self.zRelVar = tk.StringVar(value='+0.000')
        zRelLabel = tk.Label(
            textvariable=self.zRelVar,
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        zRelLabel.grid(column=1, row=3, in_=posFrame)

        self.zAbsVar = tk.StringVar(value='+0.000')
        zAbsLabel = tk.Label(
            textvariable=self.zAbsVar,
            width=6,
            font=("Times New Roman", 14),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=2,
        )
        zAbsLabel.grid(column=2, row=3, in_=posFrame)

        self.aRelVar = tk.StringVar(value='+0.000')
        aRelLabel = tk.Label(
            textvariable=self.aRelVar,
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        aRelLabel.grid(column=1, row=4, in_=posFrame)

        self.aAbsVar = tk.StringVar(value='+0.000')
        aAbsLabel = tk.Label(
            textvariable=self.aAbsVar,
            width=6,
            font=("Times New Roman", 14),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=2,
        )
        aAbsLabel.grid(column=2, row=4, in_=posFrame)

        self.stateVar = tk.StringVar(value='Idle')
        stateLabel = tk.Label(
            textvariable=self.stateVar,
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        stateLabel.grid(column=1, row=5, in_=posFrame)

        self.bufferVar = tk.IntVar(value='15')
        bufferLabel = tk.Label(
            textvariable=self.bufferVar,
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        bufferLabel.grid(column=1, row=6, in_=posFrame)

        # Create Zero Button Frame Widgets
        zeroXBut = tk.Button(
            text="Zero X",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.zeroCord(b"X"),
        )
        zeroXBut.grid(column=0, row=0, in_=zeroFrame, pady=5, padx=5)

        zeroYBut = tk.Button(
            text="Zero Y",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.zeroCord(b"Y"),
        )
        zeroYBut.grid(column=0, row=1, in_=zeroFrame, pady=5)

        zeroZBut = tk.Button(
            text="Zero Z",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.zeroCord(b"Z"),
        )
        zeroZBut.grid(column=0, row=2, in_=zeroFrame, pady=5)

        zeroABut = tk.Button(
            text="Zero A",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.zeroCord(b"A"),
        )
        zeroABut.grid(column=0, row=3, in_=zeroFrame, pady=5)

        homeXBut = tk.Button(
            text="Home X",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.sendCode(b"$HX", False)
        )
        homeXBut.grid(column=1, row=0, in_=zeroFrame, pady=5, padx=5)

        homeYBut = tk.Button(
            text="Home Y",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.sendCode(b"$HY", False)
        )
        homeYBut.grid(column=1, row=1, in_=zeroFrame, pady=5)

        homeZBut = tk.Button(
            text="Home Z",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.sendCode(b"$HZ", False)
        )
        homeZBut.grid(column=1, row=2, in_=zeroFrame, pady=5)

        homeABut = tk.Button(
            text="Home A",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.sendCode(b"$HA", False)
        )
        homeABut.grid(column=1, row=3, in_=zeroFrame, pady=5)

        homeAllBut = tk.Button(
            text="Home All",
            font=("Times New Roman", 12),
            width=9,
            pady=5,
            bg="#91ceff",
            fg="black",
            relief="raised",
            command=lambda: self.sendCode(b"$H", False)
        )
        homeAllBut.grid(column=2, row=1, in_=zeroFrame, pady=5)

        self.enXYBut = tk.Button(
            text="Disable Drives",
            font=("Times New Roman", 12),
            width=9,
            pady=5,
            bg="#8efa8e",
            fg="black",
            relief="raised",
            command=lambda: self.sendCode(b"$MD", False),
        )
        self.enXYBut.grid(column=2, row=0, in_=zeroFrame)

        self.cancelJogBut = tk.Button(
            text="Cancel Jog",
            font=("Times New Roman bold", 12),
            bg="#8efa8e",
            fg="black",
            command=lambda: self.sendCode(b'\x85', False),
        )
        self.cancelJogBut.grid(column=2, row=4, in_=zeroFrame)

    def createActuatorFrame(self):
        """Creates the actuator control section of the GUI."""
        self.aFrame = tk.LabelFrame(
            width=470,
            height=600,
            bg="#e3f0fa",
            labelwidget=tk.Label(font=("Times New Roman", 22), text="SPINDLE", fg="black"),
            bd=5,
            relief="groove",
            padx=10,
            pady=10,
        )
        self.aFrame.grid(column=1, row=0, in_=self.topFrame)
        self.aFrame.grid_propagate(0)

        butFrame = tk.Frame(width=440, height=100, bg="#e3f0fa", padx=2, pady=2,)
        butFrame.grid(column=0, row=0, in_=self.aFrame, pady=5)
        butFrame.grid_propagate(0)

        displayFrame = tk.Frame(bg="#e3f0fa")
        displayFrame.grid(column=0, row=0, in_=butFrame)

        self.figure = Figure(figsize=(4, 3), tight_layout=True)
        self.axis = self.figure.add_subplot()
        self.line = self.axis.plot(self.times, self.displayData)[0]
        self.axis.set_xlabel("Time")
        self.axis.set_ylabel("Force")
        self.axis.set_xticklabels([])

        self.canvas = FigureCanvasTkAgg(self.figure)
        self.canvas.get_tk_widget().grid(
            column=0, row=0, rowspan=2, columnspan=2, in_=displayFrame, sticky=tk.E + tk.W
        )
        self.canvas.draw_idle()

        gCodeFrame = tk.Frame(bg="#e3f0fa")
        gCodeFrame.grid(column=0, row=2, in_=displayFrame, pady=30, sticky=tk.E + tk.W)
        gCodeFrame.grid_propagate(0)

        codeFieldFrame = tk.Frame(bg="#e3f0fa")
        codeFieldFrame.grid(column=0, row=0, in_=gCodeFrame, sticky=tk.E)

        gCodeLabel = tk.Label(
            text="Enter GCode:",
            font=("Times New Roman", 14),
            width=35,
            pady=2,
            padx=4,
            bg="#EFF",
            fg="black",
            justify=tk.LEFT,
        )
        gCodeLabel.grid(column=0, row=0, in_=codeFieldFrame, sticky=tk.W)

        self.gCodeText = tk.StringVar()
        gCodeEntry = tk.Entry(
            width=30, font=("Times New Roman", 18), bg="white", fg="black",
            textvariable=self.gCodeText,
        )
        gCodeEntry.grid(column=0, row=1, in_=codeFieldFrame, sticky=tk.W)
        gCodeEntry.bind("<KeyRelease-Return>", self.sendGCode)

        gCodeFileFrame = tk.Frame(bg="#e3f0fa",)
        gCodeFileFrame.grid(column=0, row=2, pady=10, in_=codeFieldFrame)

        gCodeFileLabel = tk.Label(
            text="Enter GCode Filepath:",
            font=("Times New Roman", 14),
            width=28,
            pady=2,
            padx=4,
            bg="#EFF",
            fg="black",
            justify=tk.LEFT,
        )
        gCodeFileLabel.grid(column=0, row=0, in_=gCodeFileFrame, sticky=tk.W)

        self.gCodeFileText = tk.StringVar()
        gCodeFileEntry = tk.Entry(
            width=32,
            font=("Times New Roman", 14),
            bg="white",
            fg="black",
            textvariable=self.gCodeFileText,
            state='readonly'
        )
        gCodeFileEntry.grid(column=0, row=1, in_=gCodeFileFrame)

        gCodeFileBrowseButton = tk.Button(
            text="Browse",
            font=("Times New Roman", 12),
            width=7,
            pady=2,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=self.browseFiles,
        )
        gCodeFileBrowseButton.grid(column=1, padx=2, row=1, in_=gCodeFileFrame)

        gCodeFileRunButton = tk.Button(
            text="Run",
            font=("Times New Roman", 12),
            width=7,
            pady=2,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=self.runFile,
        )
        gCodeFileRunButton.grid(column=2, padx=2, row=1, in_=gCodeFileFrame)

        gCodeFileClearButton = tk.Button(
            text="Clear File",
            font=("Times New Roman", 12),
            width=7,
            pady=2,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.gCodeFileText.set(''),
        )
        gCodeFileClearButton.grid(column=1, pady=2, row=2, in_=gCodeFileFrame)

    def createSaveFrame(self):
        """Creates the section for saving, clearing, and collecting data."""
        sFrame = tk.LabelFrame(
            width=300,
            height=600,
            bg="#e3f0fa",
            labelwidget=tk.Label(font=("Times New Roman", 22), text="SAVE/REPORT", fg="black"),
            bd=5,
            relief="groove",
            padx=10,
            pady=10,
        )
        sFrame.grid(column=2, row=0, in_=self.topFrame)
        sFrame.grid_propagate(0)

        self.saveDataBut = tk.Button(
            text="Save All Data",
            font=("Times New Roman bold", 18),
            width=15,
            pady=5,
            bg="#8efa8e",
            fg="grey",
            relief="raised",
        )
        self.saveDataBut.grid(column=0, row=1, in_=sFrame, pady=10, padx=10)

        self.clearDataBut = tk.Button(
            text="Clear All Data",
            font=("Times New Roman bold", 18),
            width=15,
            pady=5,
            bg="#ff475d",
            fg="grey",
            relief="raised",
        )
        self.clearDataBut.grid(column=0, row=2, in_=sFrame, pady=10, padx=10)

        self.startStopDataBut = tk.Button(
            text="Start Data Collection",
            font=("Times New Roman bold", 18),
            width=15,
            pady=5,
            bg="#8efa8e",
            fg="black",
            relief="raised",
            command=self.startStopData,
        )
        self.startStopDataBut.grid(column=0, row=0, in_=sFrame, pady=20, padx=10)

        tcFrame = tk.Frame(height=20, bg="#e3f0fa")
        tcFrame.grid(column=0, row=3, in_=sFrame)
        tk.Label(text="TC 1: ", font=("Times New Roman bold", 12), fg="black", bg="#e3f0fa").grid(
            column=0, row=0, in_=tcFrame
        )
        tk.Label(
            text=" C   TC 2: ", font=("Times New Roman bold", 12), fg="black", bg="#e3f0fa"
        ).grid(column=2, row=0, in_=tcFrame)
        tk.Label(text="C", font=("Times New Roman bold", 12), fg="black", bg="#e3f0fa").grid(
            column=4, row=0, in_=tcFrame
        )

        self.tcOneVariable = tk.StringVar(value="N/A")
        tcOneLabel = tk.Label(
            textvariable=self.tcOneVariable, font=("Times New Roman bold", 12),
            pady=5, fg="black", bg="#e3f0fa", width=6
        )
        tcOneLabel.grid(column=1, row=0, in_=tcFrame)

        self.tcTwoVariable = tk.StringVar(value="N/A")
        tcTwoLabel = tk.Label(
            textvariable=self.tcTwoVariable, font=("Times New Roman bold", 12),
            pady=5, fg="black", bg="#e3f0fa", width=6
        )
        tcTwoLabel.grid(column=3, row=0, in_=tcFrame)

    def createBottomButtons(self):
        """Adds the bottom buttons to the bottom frame."""
        self.sBut = tk.Button(
            text="Start Mill",
            font=("Times New Roman bold", 20),
            bg="#8efa8e",
            fg="black",
            command=self.sendStartStop,
            width=20,
            pady=20,
        )
        self.sBut.grid(column=1, row=1, in_=self.botFrame)

    def sendStartStop(self):
        """Sends the code to turn the mill on and off."""
        if not self.controller.running.is_set():
            # b'$10=3' sets the Grbl data that is sent back when querried with b'?'
            self.sendCode(b'$10=3', False)
            self.sBut.config(text="Stop Mill", bg="#fc4747")
            self.controller.running.set()
        else:
            self.controller.serial_processor.espBuffer.clear()
            self.controller.serial_processor.espTypeBuffer.clear()
            self.sendCode(b'\x85', False)
            self.sBut.config(text="Start Mill", bg="#8efa8e")
            self.enXYBut.config(text="Enable XYA", bg="#8efa8e")
            self.controller.running.clear()

    def saveFile(self, source=None):
        """
        Saves the collected data from the serial port to a file.

        Parameters
        ----------
        source : tk.TopLevel, optional
            The window to close when saving is finished. Default is None, which
            will not close any windows.
        """
        data = self.controller.return_data()
        if data is None:
            self.clearAllData(source)
            return

        fileTypes = [('CSV', '*.csv'), ('Text', '*.txt'), ('All Files', '*.*')]
        filename = filedialog.asksaveasfilename(filetypes=fileTypes, defaultextension=fileTypes)
        if not filename:
            return
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(data)
        except PermissionError:
            print("File is currently open")
        except Exception:
            print("There was an error saving the file.")
        else:  # only clear data when the save is successful
            self.clearAllData(source)
            self.clearDataBut.configure(fg="grey", command='')
            self.saveDataBut.configure(fg="grey", command='')

    def startStopData(self):
        """Toggles data collection events and GUI elements."""
        if "Start" in self.startStopDataBut["text"]:
            if self.controller.labjack_handler.timeData:
                # have unsaved data, so cache it and then erase data
                self.controller.save_temp_file()
                self.controller.clear_data()
            self.controller.collecting.set()
            self.startStopDataBut.config(text="Stop Data Collection", bg="#ff475d")

        else:
            self.controller.collecting.clear()
            if not self.controller.labjack_handler.timeData:
                self.clearAllData()
            else:
                askSaveWin = tk.Toplevel(self.controller.root, takefocus=True)
                askSaveWin.protocol(
                    "WM_DELETE_WINDOW", lambda: [
                        self.controller.save_temp_file(), askSaveWin.destroy()
                    ]
                )
                askSaveWin.title("Save Data?")
                askSaveLabel = tk.Label(
                    askSaveWin,
                    font=("Times New Roman", 22),
                    text="Would you like to save the data?",
                    fg="black",
                    padx=5,
                )
                askSaveWin.geometry("%ix120" % askSaveLabel.winfo_reqwidth())
                askSaveLabel.grid(column=0, row=0, sticky=tk.E + tk.W)
                askSaveLabel.grid(column=0, row=0)
                askSaveButFrame = tk.Frame(askSaveWin, width=750, height=80)
                askSaveButFrame.grid(column=0, row=1, pady=10)
                tk.Button(
                    askSaveButFrame,
                    font=("Times New Roman", 22),
                    text="YES",
                    fg="black",
                    bg="#8efa8e",
                    command=lambda: self.saveFile(askSaveWin),
                ).grid(column=0, row=0, padx=30)
                tk.Button(
                    askSaveButFrame,
                    font=("Times New Roman", 22),
                    text="NO",
                    fg="black",
                    bg="#ff475d",
                    command=lambda: [self.controller.save_temp_file(), askSaveWin.destroy()],
                ).grid(column=1, row=0, padx=30)

                askSaveWin.grab_set()  # prevent interaction with main window until dialog closes
                askSaveWin.wm_transient(self.controller.root)  # set dialog above main window

                self.clearDataBut.configure(fg="black", command=self.clearDataPrompt)
                self.saveDataBut.configure(fg="black", command=self.saveFile)

            self.startStopDataBut.configure(text="Start Data Collection", bg="#8efa8e")

    def clearAllData(self, source=None):
        """Clears all collected data and resets GUI elements."""
        self.controller.clear_data()
        self.clearDataBut.configure(fg="grey", command='')
        self.saveDataBut.configure(fg="grey", command='')
        if source is not None:
            source.destroy()

    def clearDataPrompt(self):
        """Asks to save data when closing the window."""
        if not self.controller.labjack_handler.timeData:
            self.clearAllData()
        else:
            askSaveWin = tk.Toplevel(self.controller.root, takefocus=True)
            askSaveWin.title("Save Data?")
            askSaveLabel = tk.Label(
                askSaveWin,
                font=("Times New Roman", 22),
                text="Unsaved data. Are you sure?",
                fg="black",
                padx=5,
            )
            askSaveWin.geometry("%ix120" % askSaveLabel.winfo_reqwidth())
            askSaveLabel.grid(column=0, row=0, sticky=tk.E + tk.W)
            askSaveButFrame = tk.Frame(askSaveWin, width=750, height=80)
            askSaveButFrame.grid(column=0, row=1, pady=10, sticky=tk.E + tk.W)
            tk.Button(
                askSaveButFrame,
                font=("Times New Roman", 22),
                text="SAVE",
                fg="black",
                bg="#8efa8e",
                command=lambda: self.saveFile(askSaveWin),
            ).grid(column=0, row=0, padx=30)
            tk.Button(
                askSaveButFrame,
                font=("Times New Roman", 22),
                text="CLEAR",
                fg="black",
                bg="#ff475d",
                command=lambda: self.clearAllData(askSaveWin),
            ).grid(column=1, row=0, padx=30)

    def sendGCode(self, event):
        """Runs user-input GCode and ensures it is upper-case."""
        gcode = self.gCodeText.get()
        if gcode:
            if self.controller.running.is_set():
                try:
                    if int(event.type) == 3:
                        self.sendCode(gcode.upper().encode(), False)
                except Exception:
                    print("There was an exception?")
            else:
                print("Machine Must Be Started First!")

    def browseFiles(self):
        """Browses files for running GCode."""
        filename = filedialog.askopenfilename(
            initialdir=self.gcode_directory,
            title="Select a File",
            filetypes=(("GCode Files", "*.gcode*"), ("Text files", "*.txt*"), ("All files", "*.*")),
        )
        if filename:
            self.gcode_directory = Path(filename).parent
            self.gCodeFileText.set(filename)

    def runFile(self):
        """Runs GCode from the specified file."""
        filename = self.gCodeFileText.get()
        if not filename:
            return  # ignore if no file is selected

        with open(filename, 'r') as f:
            self.controller.serial_processor.espBuffer = [line.rstrip('\n').encode() for line in f]
        self.controller.serial_processor.espTypeBuffer = (
            [1] * len(self.controller.serial_processor.espBuffer)
        )
        print(self.controller.serial_processor.espTypeBuffer)

        if not self.controller.collecting.is_set():
            if self.confirm_run:
                self.confirm_run_with_data()
            elif self.confirm_run is None:
                self.startStopData()

    def set_cofirm_run(self, set_run, confirm_run):
        if set_run and confirm_run:
            self.confirm_run = None
            self.startStopData()
        elif set_run:
            self.startStopData()
        elif confirm_run:
            self.confirm_run = False

    def confirm_run_with_data(self):
        confirmRunWin = tk.Toplevel(self.controller.root, takefocus=True)
        confirmRunWin.title("Running without saving")
        askSaveLabel = tk.Label(
            confirmRunWin,
            font=("Times New Roman", 22),
            text="Turn data collection on?",
            fg="black",
            padx=5,
        )
        askSaveLabel.grid(column=0, row=0, sticky=tk.E + tk.W)
        askSaveLabel.grid(column=0, row=0)
        askSaveButFrame = tk.Frame(confirmRunWin, width=750, height=80)
        askSaveButFrame.grid(column=0, row=1, pady=10)

        checkbox_var = tk.BooleanVar()
        confirm_checkbox = tk.Checkbutton(
            askSaveButFrame, text="Remember Selection", font=("Times New Roman", 18),
            variable=checkbox_var
        )
        confirm_checkbox.grid(column=0, row=0, pady=20)

        tk.Button(
            askSaveButFrame,
            font=("Times New Roman", 22),
            text="YES",
            fg="black",
            bg="#8efa8e",
            command=lambda: [
                self.set_cofirm_run(True, checkbox_var.get()), confirmRunWin.destroy()
            ],
        ).grid(column=0, row=1, padx=30)
        tk.Button(
            askSaveButFrame,
            font=("Times New Roman", 22),
            text="NO",
            fg="black",
            bg="#ff475d",
            command=lambda: [
                self.set_cofirm_run(False, checkbox_var.get()), confirmRunWin.destroy()
            ],
        ).grid(column=1, row=1, padx=30)
        confirmRunWin.grab_set()  # prevent interaction with main window until dialog closes
        confirmRunWin.wm_transient(self.controller.root)  # set dialog above main window

    def display(self, force):
        """
        Updates the GUI with new force data.

        """
        self.displayData.append(190 - force * 0.12)
        # self.line.set_ydata(self.displayData)
        self.line.remove()
        self.axis.set_prop_cycle(plt.rcParams['axes.prop_cycle'])
        self.line = self.axis.plot(self.times, self.displayData)[0]
        self.canvas.draw_idle()

    def sendCode(self, code, wait_in_queue):
        """
        Sends code to the mill for controlling movement if the serial port is connected.

        Parameters
        ----------
        code : bytes
            The byte G-code to send to the serial port.
        wait_in_queue : bool
            False means send the code immediately and True means to wait for the
            buffer to be open.
        """
        if self.controller.serial_processor.esp is not None:
            self.controller.serial_processor.sendCode(code, wait_in_queue)

    def zeroCord(self, axis):
        """
        Sets the current position as the zero point for the given axis.

        Parameters
        ----------
        axis : {b'X', b'Y', b'Z', b'A'}
            The byte designating which axis to zero.
        """
        if self.controller.serial_processor.esp is not None:
            self.controller.serial_processor.zeroCord(axis)
