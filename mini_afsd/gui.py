# -*- coding: utf-8 -*-
"""The User Interface class for interacting with the mill."""

from collections import deque
import csv
from pathlib import Path
import struct
import time
import tkinter as tk
from tkinter import filedialog


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
    aAbsLabel : tkinter.Label
        The label for the absolute position in the actuator direction.
    aRelLabel : tkinter.Label
        The label for the relative position in the actuator direction.
    xAbsLabel : tkinter.Label
        The label for the absolute position in the x-direction.
    xRelLabel : tkinter.Label
        The label for the relative position in the x-direction.
    yAbsLabel : tkinter.Label
        The label for the absolute position in the y-direction.
    yRelLabel : tkinter.Label
        The label for the relative position in the y-direction.
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
        posFrame = tk.Frame(width=235, height=175, bg="#e3f0fa", padx=2, pady=2,)
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

        xLabel = tk.Label(
            text="X", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10,
        )
        xLabel.grid(column=0, row=1, in_=posFrame)

        yLabel = tk.Label(
            text="Y", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10,
        )
        yLabel.grid(column=0, row=2, in_=posFrame)

        aLabel = tk.Label(
            text="A", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10,
        )
        aLabel.grid(column=0, row=3, in_=posFrame)

        self.xRelLabel = tk.Label(
            text="+0.000",
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        self.xRelLabel.grid(column=1, row=1, in_=posFrame)

        self.xAbsLabel = tk.Label(
            text="+0.000",
            width=6,
            font=("Times New Roman", 14),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=2,
        )
        self.xAbsLabel.grid(column=2, row=1, in_=posFrame)

        self.yRelLabel = tk.Label(
            text="+0.000",
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        self.yRelLabel.grid(column=1, row=2, in_=posFrame)

        self.yAbsLabel = tk.Label(
            text="+0.000",
            width=6,
            font=("Times New Roman", 14),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=2,
        )
        self.yAbsLabel.grid(column=2, row=2, in_=posFrame)

        self.aRelLabel = tk.Label(
            text="+0.000",
            width=8,
            font=("Times New Roman", 18),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=6,
        )
        self.aRelLabel.grid(column=1, row=3, in_=posFrame)

        self.aAbsLabel = tk.Label(
            text="+0.000",
            width=6,
            font=("Times New Roman", 14),
            fg="black",
            bg="#EEE",
            relief="groove",
            bd=1,
            pady=2,
        )
        self.aAbsLabel.grid(column=2, row=3, in_=posFrame)

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
        zeroABut.grid(column=0, row=2, in_=zeroFrame, pady=5)

        homeXBut = tk.Button(
            text="Home X",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.goToZero(b"X"),
            state='disabled'
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
            command=lambda: self.goToZero(b"Y"),
            state='disabled'
        )
        homeYBut.grid(column=1, row=1, in_=zeroFrame, pady=5)

        homeABut = tk.Button(
            text="Home A",
            font=("Times New Roman", 12),
            width=7,
            pady=5,
            bg="#8f8f8f",
            fg="black",
            relief="raised",
            command=lambda: self.goToZero(b"A"),
            state='disabled'
        )
        homeABut.grid(column=1, row=2, in_=zeroFrame, pady=5)

        homeAllBut = tk.Button(
            text="Home All",
            font=("Times New Roman", 12),
            width=9,
            pady=5,
            bg="#91ceff",
            fg="black",
            relief="raised",
            command=lambda: self.goToZero(3),
            state='disabled'
        )
        homeAllBut.grid(column=2, row=1, in_=zeroFrame, pady=5)

        self.enXYBut = tk.Button(
            text="Enable XYA",
            font=("Times New Roman", 12),
            width=9,
            pady=5,
            bg="#8efa8e",
            fg="black",
            relief="raised",
            command=self.toggleOutputs,
        )
        self.enXYBut.grid(column=2, row=0, in_=zeroFrame)

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

        aForceFrame = tk.Frame(bg="#e3f0fa",)
        aForceFrame.grid(column=0, row=0, in_=butFrame)

        forceLabel = tk.Label(
            text="Max Force \n(N)",
            font=("Times New Roman", 10),
            width=14,
            pady=2,
            padx=4,
            bg="#EFF",
            fg="black",
        )
        forceLabel.grid(column=0, row=0, in_=aForceFrame)

        self.aForceText = tk.DoubleVar(value="0.0")
        self.aForceEntry = tk.Entry(
            width=9, font=("Times New Roman", 18), bg="white", fg="black",
            textvariable=self.aForceText,
        )
        self.aForceEntry.grid(column=0, row=1, in_=aForceFrame)
        self.aForceEntry.bind("<KeyRelease-Return>", self.updateForce)

        self.dataOutputPane = tk.Canvas(height=200, bg="#ffffff")
        self.dataOutputPane.grid(column=0, row=1, in_=self.aFrame, sticky=tk.E + tk.W)

        gCodeFrame = tk.Frame(bg="#e3f0fa")
        gCodeFrame.grid(column=0, row=2, in_=self.aFrame, pady=30, sticky=tk.E + tk.W)
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
            self.sendCode(b'B', 0)
            self.sBut.config(text="Stop Mill", bg="#fc4747")
            self.controller.running.set()
        else:
            self.controller.serial_processor.espBuffer.clear()
            self.controller.serial_processor.espTypeBuffer.clear()
            self.controller.serial_processor.waitingForAck.clear()
            self.sendCode(b'S', 0)
            self.sBut.config(text="Start Mill", bg="#8efa8e")
            self.enXYBut.config(text="Enable XYA", bg="#8efa8e")
            self.controller.running.clear()

    def toggleOutputs(self):
        """Toggles the buttons for X, Y, and A states."""
        if self.controller.running.is_set():
            if self.enXYBut["text"] == "Disable XYA":
                self.enXYBut.config(text="Enable XYA", bg="#8efa8e")
                self.sendCode(b"M18 X Y", 0)
                self.sendCode(b"M18 A", 0)
            else:
                self.enXYBut.config(text="Disable XYA", bg="#ff475d")
                self.sendCode(b"M17 X Y", 0)
                self.sendCode(b"M17 A", 0)

    def saveFile(self, source=None):
        """
        Saves the collected data from the serial port to a file.

        Parameters
        ----------
        source : tk.TopLevel, optional
            The window to close when saving is finished. Default is None, which
            will not close any windows.
        """
        fileTypes = [('CSV', '*.csv'), ('Text', '*.txt'), ('All Files', '*.*')]
        filename = filedialog.asksaveasfilename(filetypes=fileTypes, defaultextension=fileTypes)
        if not filename:
            return

        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(self.controller.serial_processor.return_data())
        except PermissionError:
            print("File is currently open")
        except Exception:
            print("There was an error saving the file.")
        else:  # only clear data when the save is successful
            self.clearAllData(source)
            self.controller.serial_processor.clear_data()
            self.clearDataBut.configure(fg="grey", command='')
            self.saveDataBut.configure(fg="grey", command='')
            self.controller.startTime = 0
            if source is not None:
                source.destroy()

    def startStopData(self):
        """Toggles data collection events and GUI elements."""
        if "Start" in self.startStopDataBut["text"]:
            if not self.controller.startTime:
                self.controller.startTime = time.time() + 0.4
            self.controller.collecting.set()
            self.startStopDataBut.config(text="Stop Data Collection", bg="#ff475d")

        else:
            self.controller.collecting.clear()
            askSaveWin = tk.Toplevel(self.controller.root, takefocus=True)
            askSaveWin.protocol(
                "WM_DELETE_WINDOW", lambda: [self.controller.save_temp_file(), askSaveWin.destroy()]
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

            self.startStopDataBut.configure(text="Start Data Collection", bg="#8efa8e")
            self.clearDataBut.configure(fg="black", command=self.clearDataPrompt)
            self.saveDataBut.configure(fg="black", command=self.saveFile)
            askSaveWin.grab_set()  # prevent interaction with main window until dialog closes
            askSaveWin.wm_transient(self.controller.root)  # set dialog above main window

    def clearAllData(self, source=None):
        """Clears all collected data and resets GUI elements."""
        self.controller.clear_data()
        self.controller.startTime = 0
        self.clearDataBut.configure(fg="grey", command='')
        self.saveDataBut.configure(fg="grey", command='')
        if source is not None:
            source.destroy()

    def clearDataPrompt(self):
        """Asks to save data when closing the window."""
        if not self.controller.timeData:
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
                        self.sendCode(gcode.upper().encode(), 1)
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

    def display(self, aForce, xPos, yPos, aPos, code, data):
        """
        Updates the GUI with new force or position data.

        Parameters
        ----------
        aForce : bytes
            The actuator force in 'H' byte format.
        xPos : bytes
            The mill's x position in 'i' byte format.
        yPos : bytes
            The mill's y position in 'i' byte format.
        aPos : bytes
            The actuator position in 'i' byte format.
        code : bytes
            _description_
        data : _type_
            _description_
        """
        if code == b'F':
            try:
                aForceN = (struct.unpack("H", aForce)[0] - 359) * 0.596  # To N
                if aForceN < 0:
                    aForceN = 0

                if self.controller.collecting.is_set():
                    self.controller.readTempData.set()
                    self.controller.serial_processor.forceData.append(aForceN)
                    self.controller.timeData.append(
                        round(time.time() - self.controller.startTime, 2)
                    )

                self.displayData.append(190 - aForceN * 0.12)
                self.dataOutputPane.delete('all')
                self.dataOutputPane.create_line(tuple(zip(self.times, self.displayData)))
            except Exception:
                print("There was a force error")
                print(data)

        try:
            xRelPos = struct.unpack("i", xPos)[0] / self.controller.xyPulPerMil
            yRelPos = struct.unpack("i", yPos)[0] / self.controller.xyPulPerMil
            aRelPos = struct.unpack("i", aPos)[0] / self.controller.aPulPerMil

            xRelSign = "+" if xRelPos >= 0 else ""
            self.xRelLabel["text"] = f"{xRelSign}{xRelPos:.3f}"
            yRelSign = "+" if yRelPos >= 0 else ""
            self.yRelLabel["text"] = f"{yRelSign}{yRelPos:.3f}"
            aRelSign = "+" if aRelPos >= 0 else ""
            self.aRelLabel["text"] = f"{aRelSign}{aRelPos:.3f}"
        except Exception:
            print("There was a position error")
            print(data)

    def updateForce(self, event):
        """Updates the force display."""
        if event.widget is self.aForceEntry:
            try:
                maxForce = self.aForceText.get()
                self.aForceText.set(f"{maxForce:.1f}")
                code = b'G12 L'
                code += str(maxForce).encode('utf-8')
                self.sendCode(code, 1)
            except Exception:
                self.aForceText.set("0.0")

    def sendCode(self, code, type):
        """Sends code to the mill for controlling movement."""
        if self.controller.serial_processor.esp is not None:
            self.controller.serial_processor.sendCode(code, type)

    def zeroCord(self, axis):
        """
        Sets the current position as the zero point for the given axis.

        Parameters
        ----------
        axis : {b'A', b'X', b'Y'}
            The byte designating which axis to zero.
        """
        if self.controller.serial_processor.esp is not None:
            self.controller.serial_processor.zeroCord(axis)

    def goToZero(self, axis):
        """
        Goes to the zero point for the given axis.

        Parameters
        ----------
        axis : {b'A', b'X', b'Y'}
            The byte designating which axis to move to zero.
        """
        if self.controller.serial_processor.esp is not None:
            self.controller.serial_processor.goToZero(axis)
