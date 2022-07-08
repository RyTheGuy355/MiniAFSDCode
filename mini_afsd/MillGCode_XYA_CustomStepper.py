import csv
from datetime import datetime
import os
from pathlib import Path
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog

from labjack import ljm
import serial
from serial.tools import list_ports


port = None
esp = None
initial_dir = '/'
check_before_running = True
close = 0
collecting = False
readTempData = False
labjackHandle = None
running = 0
waitingForAck = 0
startTime = 0
espBuffer = []
espTypeBuffer = []
timeData = []
forceData = []
TC_one_Data = []
TC_two_Data = []

displayData = [190] * 140

xHome = 0
yHome = 0
aHome = 0

xyStepsPerMil = 40
xyPulPerStep = 2
aStepsPerMil = 1020
aPulPerStep = 4

xyPulPerMil = xyStepsPerMil * xyPulPerStep
aPulPerMil = aStepsPerMil * aPulPerStep

# # xyPulPerMil = 160 #With 4 microsteps and 5mm lead
# xyPulPerMil = 816 #With 2 microsteps and 0.1" lead
# aPulPerMil = 2040 #With 2 microsteps


def _get_save_location():
    """
    Gets the filepath for saving the unsaved files depending on the operating system.

    Returns
    -------
    pathlib.Path
        The absolute path to where the files will be saved.

    Notes
    -----
    Tries to use environmental variables before using default locations, and
    tries to follow standard conventions. See the reference links (and the
    additional links in the links) for more information.

    References
    ----------
    https://stackoverflow.com/questions/1024114/location-of-ini-config-files-in-linux-unix,
    https://specifications.freedesktop.org/basedir-spec/latest/
    """
    path = None
    if sys.platform.startswith('win'):  # Windows
        path = Path(os.environ.get('LOCALAPPDATA') or '~/AppData/Local').joinpath('mini_afsd')
    elif sys.platform.startswith('darwin'):  # Mac
        path = Path('~/Library/Application Support/mini_afsd')
    elif sys.platform.startswith(('linux', 'freebsd')):  # Linux
        path = Path(os.environ.get('XDG_DATA_HOME') or '~/.local/share').joinpath('mini_afsd')

    if path is not None:
        try:
            if not path.expanduser().parent.is_dir():
                path = None
        except PermissionError:
            # permission is denied in the desired folder; will not really help
            # accessing, but allows this function to not fail so that user can
            # manually set SAVE_FOLDER
            path = None

    if path is None:
        # unspecified os, the Windows/Mac/Linux places were wrong, or access denied
        path = Path('~/.mini_afsd')

    return path.expanduser()


def actual_save(data):
    try:
        output_file = SAVE_FOLDER.joinpath(datetime.now().strftime('%Y-%m-%d %H-%M-%S.csv'))
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(data)
    except PermissionError:
        pass  # silently ignore permission error when caching data
    else:
        # try to remove all but the newest 10 files
        # default sort works since file names use ISO8601 date format
        for old_file in sorted(SAVE_FOLDER.iterdir(), reverse=True)[10:]:
            try:
                old_file.unlink()
            except Exception:
                pass


def save_temp_file():
    size = 1000000
    timeData = range(size)
    forceData = range(size, size * 2)
    TC_one_Data = range(size)
    TC_two_Data = range(size)
    labjackHandle = 2
    if labjackHandle is not None:
        combinedData = zip(timeData, forceData, TC_one_Data, TC_two_Data)
    else:
        combinedData = zip(timeData, forceData)

    # TODO threading probably not needed if files aren't that large
    io_thread = threading.Thread(target=actual_save, args=(combinedData,), daemon=True)
    io_thread.start()


def serialListen():
    print("Starting serial listener")
    global close
    while not close:
        data = esp.read_until(b'\x03\x04')
        if len(data) > 0:
            if data[0] == 1:
                if data[1] == 70:  # ACSII F
                    xPos = data[2:6]
                    yPos = data[6:10]
                    aPos = data[10:14]
                    aForce = data[14:16]
                    display(aForce, xPos, yPos, aPos, b'F', data)
                elif data[1] == 80:  # ACSII P
                    xPos = data[2:6]
                    yPos = data[6:10]
                    aPos = data[10:14]
                    display(None, xPos, yPos, aPos, b'P', data)
                elif data[1] == 8:
                    print("CTG")
                    global waitingForAck
                    waitingForAck = 0
            elif data[0] == 6:
                print("Ack")
            else:
                print(data.removesuffix(b'\x03\x04'))

    print("Stopping serial listener")
    esp.close()
    try:
        # TODO need to use an Event or something to trigger this, do not
        # want to change from child thread
        sBut["state"] = "disabled"
    except:
        print("Window appears to be closed")


def serialReporter():
    print("Starting serial reporter")
    global waitingForAck
    while esp.is_open:
        if len(espBuffer) > 0:
            for i in range(0, len(espBuffer)):
                if espTypeBuffer[i] == 0:
                    esp.write(espBuffer[i])
                    esp.write(b'\x04')
                    print(espBuffer[i])
                    espTypeBuffer.pop(i)
                    espBuffer.pop(i)
                    break
                elif not waitingForAck and running:
                    esp.write(espBuffer[i])
                    esp.write(b'\x04')
                    print(espBuffer[i])
                    espTypeBuffer.pop(i)
                    espBuffer.pop(i)
                    waitingForAck = 1
                    break
        time.sleep(0.01)


def display(aForce, xPos, yPos, aPos, code, data):
    if code == b'F':
        try:
            aForceN = (struct.unpack("H", aForce)[0] - 359) * 0.596  # To N
            if aForceN < 0:
                aForceN = 0

            if collecting:
                global readTempData
                readTempData = True
                forceData.append(aForceN)
                timeData.append(round(time.time() - startTime, 2))

            displayData.pop(0)
            displayData.append(190 - aForceN * 0.12)
            dataOutputPane.delete('all')
            times = list(range(0, len(displayData)))
            times = [t * 3 for t in times]
            dataOutputPane.create_line(tuple(zip(times, displayData)))
        except:
            print("There was a force error")
            print(data)

    try:
        xRelPos = struct.unpack("i", xPos)[0] / xyPulPerMil
        yRelPos = struct.unpack("i", yPos)[0] / xyPulPerMil
        aRelPos = struct.unpack("i", aPos)[0] / aPulPerMil

        xRelSign = "+" if xRelPos >= 0 else ""
        xRelLabel["text"] = f"{xRelSign}{xRelPos:.3f}"
        yRelSign = "+" if yRelPos >= 0 else ""
        yRelLabel["text"] = f"{yRelSign}{yRelPos:.3f}"
        aRelSign = "+" if aRelPos >= 0 else ""
        aRelLabel["text"] = f"{aRelSign}{aRelPos:.3f}"
    except:
        print("There was a position error")
        print(data)


def sendStartStop():
    global running
    if running == 0:
        sendCode(b'B', 0)
        sBut["text"] = "Stop Mill"
        sBut["bg"] = "#fc4747"
        running = 1
    else:
        espBuffer.clear()
        espTypeBuffer.clear()
        global waitingForAck
        waitingForAck = 0
        sendCode(b'S', 0)
        sBut["text"] = "Start Mill"
        sBut["bg"] = "#8efa8e"
        enXYBut["text"] = "Enable XY"
        enXYBut["bg"] = "#8efa8e"
        enABut["text"] = "Enable A"
        enABut["bg"] = "#8efa8e"
        running = 0


def sendGCode(event):
    if running:
        try:
            if int(event.type) == 3:
                sendCode(gCodeText.get().encode(), 1)
        except:
            print("There was an exception?")
    else:
        print("Machine Must Be Started First!")


def updateForce(event):
    if event.widget is aForceEntry:
        try:
            maxForce = aForceText.get()
            aForceText.set("{:.1f}".format(maxForce))
            code = b'G12 L'
            code += str(maxForce).encode('utf-8')
            sendCode(code, 1)
        except:
            maxForce = 0
            aForceText.set("0.0")


def zeroCord(axis):
    if running:
        code = b'G92 ' + axis + b'0'
        sendCode(code, 1)


def goToZero(axis):  # Not Implemented in Arduino Yet
    if running:
        message = b'G0 '
        if axis != 3:
            message = message + axis + b'0'
        else:
            message = message + b'X0 Y0 A0'
        sendCode(message, 1)


def toggleOutputs(axis):
    if running:
        if axis == b'XY':
            if enXYBut["text"] == "Disable XY":
                sendCode(b'M18 X Y', 0)
                enXYBut["text"] = "Enable XY"
                enXYBut["bg"] = "#8efa8e"
            else:
                enXYBut["text"] = "Disable XY"
                sendCode(b'M17 X Y', 0)
                enXYBut["bg"] = "#ff475d"
        else:
            if enABut["text"] == "Disable A":
                sendCode(b'M18 A', 0)
                enABut["text"] = "Enable A"
                enABut["bg"] = "#8efa8e"
            else:
                enABut["text"] = "Disable A"
                sendCode(b'M17 A', 0)
                enABut["bg"] = "#ff475d"


def sendCode(code, type):
    espBuffer.append(code)
    espTypeBuffer.append(type)
    print(len(espBuffer))


def browseFiles():
    global filename
    global initial_dir
    filename = filedialog.askopenfilename(
        initialdir=initial_dir,
        title="Select a File",
        filetypes=(("GCode Files", "*.gcode*"), ("Text files", "*.txt*"), ("All files", "*.*")),
    )
    if filename:
        initial_dir = Path(filename).parent
        gCodeFileText.set(filename)


def set_cofirm_run(set_run, confirm_run):
    print(confirm_run)
    global check_before_running

    if set_run and confirm_run:
        check_before_running = None
        startStopData()
    elif set_run:
        startStopData()
    elif confirm_run:
        check_before_running = False


def confirm_run_with_data():

    confirmRunWin = tk.Toplevel(window, takefocus=True)
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
        command=lambda: [set_cofirm_run(True, checkbox_var.get()), confirmRunWin.destroy()],
    ).grid(column=0, row=1, padx=30)
    tk.Button(
        askSaveButFrame,
        font=("Times New Roman", 22),
        text="NO",
        fg="black",
        bg="#ff475d",
        command=lambda: [set_cofirm_run(False, checkbox_var.get()), confirmRunWin.destroy()],
    ).grid(column=1, row=1, padx=30)
    confirmRunWin.grab_set()  # prevent interaction with main window until dialog closes
    confirmRunWin.wm_transient(window)  # set dialog above main window


def runFile():
    global filename
    filename = gCodeFileText.get()
    if not filename:
        return  # ignore if no file is selected

    global espTypeBuffer
    global espBuffer
    espTypeBuffer.clear()
    espBuffer.clear()
    with open(filename, 'r') as f:
        espBuffer = [line.rstrip('\n').encode() for line in f]
        espTypeBuffer = [1] * len(espBuffer)
    print(espBuffer)

    print(check_before_running)
    if not collecting:
        if check_before_running is True:
            confirm_run_with_data()
        elif  check_before_running is None:
            startStopData()


def saveFile(source=None):
    fileTypes = [('CSV', '*.csv'), ('Text', '*.txt'), ('All Files', '*.*')]
    saveFile = filedialog.asksaveasfilename(filetypes=fileTypes, defaultextension=fileTypes)
    if not saveFile:
        return

    size = 10000
    timeData = list(range(size))
    forceData = list(range(size))
    TC_one_Data = list(range(size))
    TC_two_Data = list(range(size))
    labjackHandle = 2
    if labjackHandle is not None:
        combinedData = zip(timeData, forceData, TC_one_Data, TC_two_Data)
    else:
        combinedData = zip(timeData, forceData)

    try:
        with open(saveFile, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(combinedData)
    except PermissionError:
        print("File is currently open")
    except Exception:
        print("There was an error saving the file.")
    else:  # only clear data when the save is successful
        clearAllData(source)


def startStopData():
    global collecting
    if "Start" in startStopDataBut["text"]:
        global startTime
        if not startTime:
            startTime = time.time() + 0.4
        collecting = True
        startStopDataBut["text"] = "Stop Data Collection"
        startStopDataBut["bg"] = "#ff475d"

    else:
        collecting = False
        askSaveWin = tk.Toplevel(window, takefocus=True)
        askSaveWin.protocol("WM_DELETE_WINDOW", lambda: [save_temp_file(), askSaveWin.destroy()])
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
            command=lambda: saveFile(askSaveWin),
        ).grid(column=0, row=0, padx=30)
        tk.Button(
            askSaveButFrame,
            font=("Times New Roman", 22),
            text="NO",
            fg="black",
            bg="#ff475d",
            command=lambda: [save_temp_file(), askSaveWin.destroy()],
        ).grid(column=1, row=0, padx=30)

        startStopDataBut.configure(text="Start Data Collection", bg="#8efa8e")
        clearDataBut.configure(fg="black", command=clearDataPrompt)
        saveDataBut.configure(fg="black", command=saveFile)
        askSaveWin.grab_set()  # prevent interaction with main window until dialog closes
        askSaveWin.wm_transient(window)  # set dialog above main window


def clearAllData(source=None):
    if source is not None:
        source.destroy()
    timeData.clear()
    forceData.clear()
    TC_one_Data.clear()
    TC_two_Data.clear()
    global startTime
    startTime = 0
    clearDataBut.configure(fg="grey", command='')
    saveDataBut.configure(fg="grey", command='')


def clearDataPrompt():
    if not timeData:  # should never be True, but useful for testing
        clearAllData()
    else:
        askSaveWin = tk.Toplevel(window, takefocus=True)
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
            command=lambda: saveFile(askSaveWin),
        ).grid(column=0, row=0, padx=30)
        tk.Button(
            askSaveButFrame,
            font=("Times New Roman", 22),
            text="CLEAR",
            fg="black",
            bg="#ff475d",
            command=lambda: clearAllData(askSaveWin),
        ).grid(column=1, row=0, padx=30)


# Creating GUI
def createMainFrames():
    global topFrame
    topFrame = tk.Frame(width=1300, height=600, bg="#e3f0fa")
    topFrame.grid(column=0, row=0)
    topFrame.grid_propagate(0)

    global botFrame
    botFrame = tk.Frame(
        width=1300, height=130, bg="#e3f0fa", bd=5, relief="groove", padx=10, pady=10
    )
    botFrame.grid(column=0, row=1)
    botFrame.grid_propagate(0)


def createTraverseFrame():
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
    tFrame.grid(column=0, row=0, in_=topFrame)
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
        text="Work", font=("Times New Roman", 12), fg="black", bg="#e3f0fa", width=12
    )
    workLabel.grid(column=1, row=0, in_=posFrame)

    machineLabel = tk.Label(
        text="Machine", font=("Times New Roman", 12), fg="black", bg="#e3f0fa", width=8
    )
    machineLabel.grid(column=2, row=0, in_=posFrame)

    xLabel = tk.Label(
        text="X", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10
    )
    xLabel.grid(column=0, row=1, in_=posFrame)

    yLabel = tk.Label(
        text="Y", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10
    )
    yLabel.grid(column=0, row=2, in_=posFrame)

    aLabel = tk.Label(
        text="A", font=("Times New Roman", 18), fg="black", bg="#e3f0fa", width=3, pady=10
    )
    aLabel.grid(column=0, row=3, in_=posFrame)

    global xRelLabel
    xRelLabel = tk.Label(
        text="+0.000",
        width=8,
        font=("Times New Roman", 18),
        fg="black",
        bg='#EEE',
        relief='groove',
        bd=1,
        pady=6,
    )
    xRelLabel.grid(column=1, row=1, in_=posFrame)

    global xAbsLabel
    xAbsLabel = tk.Label(
        text="+0.000",
        width=6,
        font=("Times New Roman", 14),
        fg="black",
        bg='#EEE',
        relief='groove',
        bd=1,
        pady=2,
    )
    xAbsLabel.grid(column=2, row=1, in_=posFrame)

    global yRelLabel
    yRelLabel = tk.Label(
        text="+0.000",
        width=8,
        font=("Times New Roman", 18),
        fg="black",
        bg='#EEE',
        relief='groove',
        bd=1,
        pady=6,
    )
    yRelLabel.grid(column=1, row=2, in_=posFrame)

    global yAbsLabel
    yAbsLabel = tk.Label(
        text="+0.000",
        width=6,
        font=("Times New Roman", 14),
        fg="black",
        bg='#EEE',
        relief='groove',
        bd=1,
        pady=2,
    )
    yAbsLabel.grid(column=2, row=2, in_=posFrame)

    global aRelLabel
    aRelLabel = tk.Label(
        text="+0.000",
        width=8,
        font=("Times New Roman", 18),
        fg="black",
        bg='#EEE',
        relief='groove',
        bd=1,
        pady=6,
    )
    aRelLabel.grid(column=1, row=3, in_=posFrame)

    global aAbsLabel
    aAbsLabel = tk.Label(
        text="+0.000",
        width=6,
        font=("Times New Roman", 14),
        fg="black",
        bg='#EEE',
        relief='groove',
        bd=1,
        pady=2,
    )
    aAbsLabel.grid(column=2, row=3, in_=posFrame)

    # Create Zero Button Frame Widgets
    zeroXBut = tk.Button(
        text="Zero X",
        font=("Times New Roman", 12),
        width=7,
        pady=5,
        bg="#8f8f8f",
        fg="black",
        relief="raised",
        command=lambda: zeroCord(b'X'),
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
        command=lambda: zeroCord(b'Y'),
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
        command=lambda: zeroCord(b'A'),
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
        command=lambda: goToZero(b'X'),
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
        command=lambda: goToZero(b'Y'),
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
        command=lambda: goToZero(b'A'),
    )
    homeABut.grid(column=1, row=2, in_=zeroFrame, pady=5)

    homeAllBut = tk.Button(
        text="Home All",
        font=("Times New Roman", 12),
        width=7,
        pady=5,
        bg="#91ceff",
        fg="black",
        relief="raised",
        command=lambda: goToZero(3),
    )
    homeAllBut.grid(column=2, row=1, in_=zeroFrame, pady=5)

    # Disable unused buttons
    homeXBut["state"] = "disabled"
    homeYBut["state"] = "disabled"
    homeABut["state"] = "disabled"
    homeAllBut["state"] = "disabled"

    global enXYBut
    enXYBut = tk.Button(
        text="Enable XY",
        font=("Times New Roman", 12),
        width=7,
        pady=5,
        bg="#8efa8e",
        fg="black",
        relief="raised",
        command=lambda: toggleOutputs(b'XY'),
    )
    enXYBut.grid(column=2, row=0, in_=zeroFrame)


def createActuatorFrame():
    global aFrame
    aFrame = tk.LabelFrame(
        width=470,
        height=600,
        bg="#e3f0fa",
        labelwidget=tk.Label(font=("Times New Roman", 22), text="SPINDLE", fg="black"),
        bd=5,
        relief="groove",
        padx=10,
        pady=10,
    )
    aFrame.grid(column=1, row=0, in_=topFrame)
    aFrame.grid_propagate(0)

    butFrame = tk.Frame(width=440, height=100, bg="#e3f0fa", padx=2, pady=2,)
    butFrame.grid(column=0, row=0, in_=aFrame, pady=5)
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

    global aForceText
    global aForceEntry
    aForceText = tk.DoubleVar(value="0.0")
    aForceEntry = tk.Entry(
        width=9, font=("Times New Roman", 18), bg='white', fg='black', textvariable=aForceText,
    )
    aForceEntry.grid(column=0, row=1, in_=aForceFrame)
    aForceEntry.bind("<KeyRelease-Return>", updateForce)

    global enABut
    enABut = tk.Button(
        text="Enable A",
        font=("Times New Roman bold", 18),
        width=7,
        pady=5,
        bg="#8efa8e",
        fg="black",
        relief="raised",
        command=lambda: toggleOutputs(b'A'),
    )
    enABut.grid(column=1, row=0, in_=butFrame, pady=10, padx=10)

    global dataOutputPane
    dataOutputPane = tk.Canvas(height=200, bg="#ffffff")
    dataOutputPane.grid(column=0, row=1, in_=aFrame, sticky=tk.E + tk.W)

    gCodeFrame = tk.Frame(bg="#e3f0fa")
    gCodeFrame.grid(column=0, row=2, in_=aFrame, pady=30, sticky=tk.E + tk.W)
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

    global gCodeText
    gCodeText = tk.StringVar()
    gCodeEntry = tk.Entry(
        width=30, font=("Times New Roman", 18), bg='white', fg='black', textvariable=gCodeText,
    )
    gCodeEntry.grid(column=0, row=1, in_=codeFieldFrame, sticky=tk.W)
    gCodeEntry.bind("<KeyRelease-Return>", sendGCode)

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

    global gCodeFileText
    gCodeFileText = tk.StringVar()
    gCodeFileEntry = tk.Entry(
        width=32,
        font=("Times New Roman", 14),
        bg='white',
        fg='black',
        textvariable=gCodeFileText,
        state='readonly',
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
        command=browseFiles,
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
        command=runFile,
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
        command=lambda : gCodeFileText.set(''),
    )
    gCodeFileClearButton.grid(column=1, pady=2, row=2, in_=gCodeFileFrame)


def createSaveFrame():
    global sFrame
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
    sFrame.grid(column=2, row=0, in_=topFrame)
    sFrame.grid_propagate(0)

    global saveDataBut
    saveDataBut = tk.Button(
        text="Save All Data",
        font=("Times New Roman bold", 18),
        width=15,
        pady=5,
        bg="#8efa8e",
        fg="grey",
        relief="raised",
    )
    saveDataBut.grid(column=0, row=1, in_=sFrame, pady=10, padx=10)

    global clearDataBut
    clearDataBut = tk.Button(
        text="Clear All Data",
        font=("Times New Roman bold", 18),
        width=15,
        pady=5,
        bg="#ff475d",
        fg="grey",
        relief="raised",
    )
    clearDataBut.grid(column=0, row=2, in_=sFrame, pady=10, padx=10)

    global startStopDataBut
    startStopDataBut = tk.Button(
        text="Start Data Collection",
        font=("Times New Roman bold", 18),
        width=15,
        pady=5,
        bg="#8efa8e",
        fg="black",
        relief="raised",
        command=startStopData,
    )
    startStopDataBut.grid(column=0, row=0, in_=sFrame, pady=20, padx=10)

    tcFrame = tk.Frame(height=20, bg="#e3f0fa")
    tcFrame.grid(column=0, row=3, in_=sFrame)
    tk.Label(text="TC 1: ", font=("Times New Roman bold", 12), fg="black", bg="#e3f0fa").grid(
        column=0, row=0, in_=tcFrame
    )
    tk.Label(text=" C   TC 2: ", font=("Times New Roman bold", 12), fg="black", bg="#e3f0fa").grid(
        column=2, row=0, in_=tcFrame
    )
    tk.Label(text="C", font=("Times New Roman bold", 12), fg="black", bg="#e3f0fa").grid(
        column=4, row=0, in_=tcFrame
    )
    global tcOneVariable
    tcOneVariable = tk.StringVar(value="N/A")
    tcOneLabel = tk.Label(
        textvariable=tcOneVariable,
        font=("Times New Roman bold", 12),
        pady=5,
        fg="black",
        bg="#e3f0fa",
        width=6,
    )
    tcOneLabel.grid(column=1, row=0, in_=tcFrame)
    global tcTwoVariable
    tcTwoVariable = tk.StringVar(value="N/A")
    tcTwoLabel = tk.Label(
        textvariable=tcTwoVariable,
        font=("Times New Roman bold", 12),
        pady=5,
        fg="black",
        bg="#e3f0fa",
        width=6,
    )
    tcTwoLabel.grid(column=3, row=0, in_=tcFrame)


def createBottomButtons():
    global sBut
    sBut = tk.Button(
        text="Start Mill",
        font=("Times New Roman bold", 20),
        bg="#8efa8e",
        fg="black",
        command=sendStartStop,
        width=20,
        pady=20,
    )
    sBut.grid(column=1, row=1, in_=botFrame)


def on_closing():
    if timeData:
        askSaveWin = tk.Toplevel(window, takefocus=True)
        askSaveWin.protocol("WM_DELETE_WINDOW", closeAll)
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
        askSaveLabel.grid(column=0, row=0)
        askSaveButFrame = tk.Frame(askSaveWin, width=750, height=80)
        askSaveButFrame.grid(column=0, row=1, pady=10)
        tk.Button(
            askSaveButFrame,
            font=("Times New Roman", 22),
            text="SAVE",
            fg="black",
            bg="#8efa8e",
            command=lambda: [saveFile(askSaveWin), closeAll()],
        ).grid(column=0, row=0, padx=30)
        tk.Button(
            askSaveButFrame,
            font=("Times New Roman", 22),
            text="CLEAR",
            fg="black",
            bg="#ff475d",
            command=closeAll,
        ).grid(column=1, row=0, padx=30)
    else:
        closeAll()


def closeAll():
    if labjackHandle is not None:
        ljm.close(labjackHandle)
    window.destroy()
    esp.write(b'S')
    esp.write(b'\x04')
    esp.flush()
    global running
    running = False
    global close
    close = True
    # esp.close()  # TODO causes issue since thread_1 is still using esp -> need to sort out
    sys.exit()


def startLabjack():
    numFrames = 2
    addresses = [7002, 7004]  # AIN1, AIN2
    dataTypes = [ljm.constants.FLOAT32, ljm.constants.FLOAT32]
    global readTempData
    while labjackHandle is not None:
        if collecting:
            ljm.eWriteAddress(labjackHandle, 1000, ljm.constants.FLOAT32, 2.67)
            while collecting:
                try:
                    while not readTempData:
                        if not collecting:
                            break
                        time.sleep(0.05)
                    readTempData = False
                    results = ljm.eReadAddresses(labjackHandle, numFrames, addresses, dataTypes)
                    TC_one_Data.append(results[0])
                    TC_two_Data.append(results[1])
                    tcOneVariable.set(round(results[0], 2))
                    tcTwoVariable.set(round(results[1], 2))
                except KeyboardInterrupt:
                    break
                except Exception as ex:
                    print(ex)
                    if type(ex).__name__ != "LJMError":
                        break
                    readTempData = False
                    while not readTempData:
                        if not collecting:
                            break
                        time.sleep(0.01)
            ljm.eWriteAddress(labjackHandle, 1000, ljm.constants.FLOAT32, 0)
        else:
            results = ljm.eReadAddresses(labjackHandle, numFrames, addresses, dataTypes)
            tcOneVariable.set(round(results[0], 2))
            tcTwoVariable.set(round(results[1], 2))
            curTime = time.time()
            while (time.time() - curTime) < 1:
                time.sleep(0.01)
                if collecting:
                    break


def serial_selection_dialog():
    all_ports = list_ports.comports()
    handles = [' '.join(vals) for vals in all_ports]
    if not handles:
        print('No serial ports found, entering test mode')
        set_port(None)
        return
    portWindow = tk.Toplevel(window, takefocus=True)
    portWindow.protocol("WM_DELETE_WINDOW", lambda: [set_port(None), portWindow.destroy()])
    portWindow.title("Select Serial Port")
    portLabel = tk.Label(
        portWindow,
        font=("Times New Roman", 22),
        text="Select the serial port to use",
        fg="black",
        padx=5,
    )
    portLabel.grid(column=0, row=0)
    selections = tk.StringVar(value=handles[0])
    port_options = tk.OptionMenu(portWindow, selections, *handles)
    port_options.grid(column=0, row=1)
    askSaveButFrame = tk.Frame(portWindow, width=750, height=80)
    askSaveButFrame.grid(column=0, row=2, pady=10, sticky='se')
    select_button = tk.Button(
        askSaveButFrame,
        text="Select",
        fg="black",
        bg="#8efa8e",
        font=("Times New Roman", 15),
        command=lambda: [set_port(selections.get().split(' ')[0]), portWindow.destroy()],
    )
    select_button.grid(column=0, row=0, padx=30)
    portWindow.geometry("%ix160" % port_options.winfo_reqwidth())
    portWindow.grab_set()  # prevent interaction with main window until dialog closes
    portWindow.wm_transient(window)  # set dialog above main window


def set_port(port_string):
    global port
    port = port_string
    print(f'Setting port to {port}')


def find_port(com_regex='(CP21)'):
    """
    Finds the correct serial port for communicating with drill.

    Parameters
    ----------
    com_regex : str
        The regular expression to use when searching for the com port.
        Default is '(CP21)', which searches for the string 'CP21'.
    """
    matching_ports = list(list_ports.grep(com_regex))
    if len(matching_ports) == 1:
        set_port(matching_ports[0][0])
    else:  # multiple or no ports found
        serial_selection_dialog()


if __name__ == "__main__":

    SAVE_FOLDER = _get_save_location()
    SAVE_FOLDER.mkdir(exist_ok=True)

    window = tk.Tk()
    createMainFrames()
    createTraverseFrame()
    createActuatorFrame()
    createSaveFrame()
    createBottomButtons()

    find_port('(COM3)')
    if port is not None:
        esp = serial.Serial(port=port, baudrate=115200)
        listener = threading.Thread(target=serialListen, daemon=True)
        listener.start()

        reporter = threading.Thread(target=serialReporter, daemon=True)
        reporter.start()

        try:
            labjackHandle = ljm.openS("T7", "ANY", "ANY")
        except Exception as ex:
            if type(ex).__name__ != "LJMError":  # TODO is this trying to catch ljm.LJMError?
                print("No Labjack Connected")
        else:
            labjackThread = threading.Thread(target=startLabjack, daemon=True)
            print("Connected to LabJack")
            labjackThread.start()

    window.protocol("WM_DELETE_WINDOW", on_closing)
    window.mainloop()
