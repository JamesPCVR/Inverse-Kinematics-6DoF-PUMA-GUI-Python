from re import T
import numpy as np
from tkinter import *
from tkinter import messagebox, colorchooser
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from datetime import datetime
from time import perf_counter
from PIL import Image, ImageTk
import colorsys
import InvKin
import FwdKin

presetperformanceselect = None

# calculate inverse kinematics
def calcInverseKinematics():
    global xdata
    global ydata
    global zdata
    global currentposition
    global xdataghost
    global ydataghost
    global zdataghost
    global ghostposition

    updateLog("Calculating inverse kinematics")
    start = perf_counter()

    # collect joint lengths
    jointlengths = np.zeros(6)
    for i in range(6):
        jointlengths[i] = globalsettings[f"A{i+1}_length"]

    # collect target data
    position = np.array(
        [
            float(tx.get()),
            float(ty.get()),
            float(tz.get()),
            np.radians(float(rx.get())),
            np.radians(float(ry.get())),
            np.radians(float(rz.get())),
        ]
    )

    # calculation
    raw = InvKin.invKin(jointlengths, position)

    if raw:
        angles = raw[0]
        xdataghost = xdata
        ydataghost = ydata
        zdataghost = zdata
        ghostposition = currentposition
        xdata = raw[1]
        ydata = raw[2]
        zdata = raw[3]
        currentposition = position

        # prepare new data
        a1angle = round(np.degrees(angles[0]), 2)
        a2angle = round(np.degrees(angles[1]), 2)
        a3angle = round(np.degrees(angles[2]), 2)
        a4angle = round(np.degrees(angles[3]), 2)
        a5angle = round(np.degrees(angles[4]), 2)
        a6angle = round(np.degrees(angles[5]), 2)

        # validata new data
        error = False
        if not (
            globalsettings["A1_constr_neg"]
            <= a1angle
            <= globalsettings["A1_constr_pos"]
        ):
            error = True
        elif not (
            globalsettings["A2_constr_neg"]
            <= a2angle
            <= globalsettings["A2_constr_pos"]
        ):
            error = True
        elif not (
            globalsettings["A3_constr_neg"]
            <= a3angle
            <= globalsettings["A3_constr_pos"]
        ):
            error = True
        elif not (
            globalsettings["A4_constr_neg"]
            <= a4angle
            <= globalsettings["A4_constr_pos"]
        ):
            error = True
        elif not (
            globalsettings["A5_constr_neg"]
            <= a5angle
            <= globalsettings["A5_constr_pos"]
        ):
            error = True
        elif not (
            globalsettings["A6_constr_neg"]
            <= a6angle
            <= globalsettings["A6_constr_pos"]
        ):
            error = True

        if error:
            # finish
            end = perf_counter()
            updateLog(
                f"ERROR occurred in inverse kinematics (took {round((end-start)*1000, 2)}ms)"
            )
            if bool(globalsettings["Update"]):
                if globalsettings["Ghost"] == 1.0:
                    plotData(True)
                plotData()
            messagebox.showerror(
                title="An error occurred",
                message="Angles fall outside constraints"
            )

        else:
            # erase old data
            a1.delete(0, END)
            a2.delete(0, END)
            a3.delete(0, END)
            a4.delete(0, END)
            a5.delete(0, END)
            a6.delete(0, END)

            # insert new data
            a1.insert(0, a1angle)
            a2.insert(0, a2angle)
            a3.insert(0, a3angle)
            a4.insert(0, a4angle)
            a5.insert(0, a5angle)
            a6.insert(0, a6angle)

            # finish
            end = perf_counter()
            updateLog(f"Calculation completed (took {round((end-start)*1000, 2)}ms)")
            if bool(globalsettings["Update"]):
                if globalsettings["Ghost"] == 1.0:
                    plotData(True)
                plotData()
            if bool(globalsettings["Automate"]) and not error:
                directTravelFunc()

    else:
        # finish
        end = perf_counter()
        updateLog(
            f"ERROR occurred in inverse kinematics (took {round((end-start)*1000, 2)}ms)"
        )
        if bool(globalsettings["Update"]):
            if globalsettings["Ghost"] == 1.0:
                plotData(True)
            plotData()
        messagebox.showerror(
            title="An error occurred",
            message="Destination is too far away"
        )


# calculate forward kinematics
def calcForwardKinematics():
    global xdata
    global ydata
    global zdata
    global currentposition
    global xdataghost
    global ydataghost
    global zdataghost
    global ghostposition

    updateLog("Calculating forward kinematics")
    start = perf_counter()

    # collect joint lengths
    jointlengths = np.zeros(6)
    for i in range(6):
        jointlengths[i] = globalsettings[f"A{i+1}_length"]

    # collect target data
    jointangles = np.array(
        [
            float(a1.get()),
            float(a2.get()),
            float(a3.get()),
            float(a4.get()),
            float(a5.get()),
            float(a6.get()),
        ]
    )

    # calculation
    raw = FwdKin.fwdKin(jointlengths, np.radians(jointangles))

    positions = raw[0]
    xdataghost = xdata
    ydataghost = ydata
    zdataghost = zdata
    ghostposition = currentposition
    xdata = raw[1]
    ydata = raw[2]
    zdata = raw[3]
    currentposition = [
        positions[0],
        positions[1],
        positions[2],
        positions[3],
        positions[4],
        positions[5]
    ]

    # erase old data
    tx.delete(0, END)
    ty.delete(0, END)
    tz.delete(0, END)
    rx.delete(0, END)
    ry.delete(0, END)
    rz.delete(0, END)

    # insert new data
    tx.insert(0, round(positions[0], 2))
    ty.insert(0, round(positions[1], 2))
    tz.insert(0, round(positions[2], 2))
    rx.insert(0, round(np.degrees(positions[3]), 2))
    ry.insert(0, round(np.degrees(positions[4]), 2))
    rz.insert(0, round(np.degrees(positions[5]), 2))

    # finish
    end = perf_counter()
    updateLog(f"Calculation completed (took {round((end-start)*1000, 2)}ms)")
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()
    if bool(globalsettings["Automate"]):
        directTravelFunc()


# set joint data to the home position
def goHome():
    global xdata
    global ydata
    global zdata
    global currentposition
    global xdataghost
    global ydataghost
    global zdataghost
    global ghostposition

    # erase old data
    tx.delete(0, END)
    ty.delete(0, END)
    tz.delete(0, END)
    rx.delete(0, END)
    ry.delete(0, END)
    rz.delete(0, END)
    a1.delete(0, END)
    a2.delete(0, END)
    a3.delete(0, END)
    a4.delete(0, END)
    a5.delete(0, END)
    a6.delete(0, END)

    # insert new data
    tx.insert(
        0,
        str(
            globalsettings["A3_length"]
            + globalsettings["A4_length"]
            + globalsettings["A5_length"]
            + globalsettings["A6_length"]
        ),
    )
    ty.insert(0, "0.0")
    tz.insert(0, str(globalsettings["A1_length"] + globalsettings["A2_length"]))
    rx.insert(0, "0.0")
    ry.insert(0, "0.0")
    rz.insert(0, "0.0")
    a1.insert(0, "0.0")
    a2.insert(0, "0.0")
    a3.insert(0, "90.0")
    a4.insert(0, "0.0")
    a5.insert(0, "0.0")
    a6.insert(0, "0.0")

    # create plot data
    xpoint1 = globalsettings["A3_length"]
    xpoint2 = xpoint1 + globalsettings["A4_length"]
    xpoint3 = xpoint2 + globalsettings["A5_length"]
    xpoint4 = xpoint3 + globalsettings["A6_length"]
    zpoint1 = globalsettings["A1_length"]
    zpoint2 = zpoint1 + globalsettings["A2_length"]
    flag = False
    try:
        xdataghost = xdata
        ydataghost = ydata
        zdataghost = zdata
        ghostposition = currentposition
    except:
        flag = True
    xdata = [0, 0, 0, xpoint1, xpoint2, xpoint3, xpoint4]
    ydata = [0, 0, 0, 0, 0, 0, 0]
    zdata = [0, zpoint1, zpoint2, zpoint2, zpoint2, zpoint2, zpoint2]
    currentposition = [
        xpoint4,
        0,
        zpoint2,
        0,
        0,
        0
    ]
    if flag:
        xdataghost = xdata
        ydataghost = ydata
        zdataghost = zdata
        ghostposition = currentposition

    # finish
    updateWindow()
    updateLog("Robot homed")
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()
    if bool(globalsettings["Automate"]):
        directTravelFunc()


# pass data to another program direct
def directTravelFunc():
    data = {
        "tx": float(tx.get()),
        "ty": float(ty.get()),
        "tz": float(tz.get()),
        "rx": float(rx.get()),
        "ry": float(ry.get()),
        "rz": float(rz.get()),
        "a1": float(a1.get()),
        "a2": float(a2.get()),
        "a3": float(a3.get()),
        "a4": float(a4.get()),
        "a5": float(a5.get()),
        "a6": float(a6.get()),
        "end": float(endeffectorslider.get()),
    }
    updateLog("Travelling to position directly")
    directTravel(data)

# pass data to another program linear interpolation
def lerpTravelFunc():
    updateLog("Calculating linear interpolation")
    start = perf_counter()
    filename = datetime.now().strftime("lerps/lerp (%y-%m-%d %H-%M-%S).csv")

    #perform the calculation
    jointlengths = np.zeros(6)
    for i in range(6):
        jointlengths[i] = globalsettings[f"A{i+1}_length"]
    file = open(filename, "w+")
    file.write("a1,a2,a3,a4,a5,a6")
    lerpsize = globalsettings['Lerp_resolution']
    for i in range(lerpsize):
        t = i/lerpsize
        newtarget = [
            lerp(ghostposition[0], currentposition[0], t),
            lerp(ghostposition[1], currentposition[1], t),
            lerp(ghostposition[0], currentposition[2], t),
            lerp(ghostposition[3], currentposition[3], t),
            lerp(ghostposition[4], currentposition[4], t),
            lerp(ghostposition[5], currentposition[5], t)
        ]
        targetpos = np.array(newtarget)
        angles = InvKin.invKin(jointlengths, targetpos)
        file.write(f"\n{angles[0][0]},{angles[0][1]},{angles[0][2]},{angles[0][3]},{angles[0][4]},{angles[0][5]}")
    file.close()

    # finish
    end = perf_counter()
    updateLog(f"Calculation completed (took {round((end-start)*1000, 2)}ms)")
    updateLog(f"Saved to '{filename}'")
    lerpTravel(filename)

# linear interpolation
def lerp(a, b, t):
    return (a * (1 - t)) + (b * t)

# handle eStop button input
def eStopFunc():
    updateLog("Emergency stop")
    eStop()


# plot data onto window
def plotData(ghost=False):
    global xdata
    global ydata
    global zdata
    global currentposition
    global xdataghost
    global ydataghost
    global zdataghost
    global ghostposition

    plotx = xdataghost if ghost else xdata
    ploty = ydataghost if ghost else ydata
    plotz = zdataghost if ghost else zdata

    # clear the current plot (ghost plotted first)
    if ghost or globalsettings["Ghost"] == 0.0:
        ax.clear()

    # collect joint data
    totallength = globalsettings["A1_length"]
    totallength += globalsettings["A2_length"]
    totallength += globalsettings["A3_length"]
    totallength += globalsettings["A4_length"]
    totallength += globalsettings["A5_length"]
    totallength += globalsettings["A6_length"]

    # scale plot to robot
    xytune = 0.65
    ztune = 1
    xylim = (-totallength * xytune, totallength * xytune)
    ax.set(xlim=xylim, ylim=xylim, zlim=(0, totallength * ztune))

    # plot data
    ax.scatter(
        plotx[0],
        ploty[0],
        plotz[0],
        c="#f0f0f0" if ctk.get_appearance_mode() == "Dark" else "#000000",
        linewidths=globalsettings["Marker_size"],
        marker="D",
    )
    ax.scatter(
        plotx[1],
        ploty[1],
        plotz[1],
        c=getHex("Col2_plt"),
        linewidths=globalsettings["Marker_size"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
        marker="o",
    )
    ax.scatter(
        plotx[2],
        ploty[2],
        plotz[2],
        c=getHex("Col3_plt"),
        linewidths=globalsettings["Marker_size"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
        marker="o",
    )
    ax.scatter(
        plotx[3],
        ploty[3],
        plotz[3],
        c=getHex("Col4_plt"),
        linewidths=globalsettings["Marker_size"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
        marker="o",
    )
    ax.scatter(
        plotx[4],
        ploty[4],
        plotz[4],
        c=getHex("Col5_plt"),
        linewidths=globalsettings["Marker_size"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
        marker="o",
    )
    ax.scatter(
        plotx[5],
        ploty[5],
        plotz[5],
        c=getHex("Col6_plt"),
        linewidths=globalsettings["Marker_size"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
        marker="o",
    )
    ax.scatter(
        plotx[6],
        ploty[6],
        plotz[6],
        c="#f0f0f0" if ctk.get_appearance_mode() == "Dark" else "#000000",
        linewidths=globalsettings["Marker_size"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
        marker="D",
    )
    ax.plot(
        plotx[0:2],
        ploty[0:2],
        plotz[0:2],
        c=getHex("Col1_plt"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )
    ax.plot(
        plotx[1:3],
        ploty[1:3],
        plotz[1:3],
        c=getHex("Col2_plt"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )
    ax.plot(
        plotx[2:4],
        ploty[2:4],
        plotz[2:4],
        c=getHex("Col3_plt"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )
    ax.plot(
        plotx[3:5],
        ploty[3:5],
        plotz[3:5],
        c=getHex("Col4_plt"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )
    ax.plot(
        plotx[4:6],
        ploty[4:6],
        plotz[4:6],
        c=getHex("Col5_plt"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )
    ax.plot(
        plotx[5:7],
        ploty[5:7],
        plotz[5:7],
        c=getHex("Col6_plt"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )

    # plot origins
    if globalsettings["Show_origins"] == 1:
        plotOrigin(
            0, 0, 0, 0, 0, 0, totallength / 50 * globalsettings["Origin_size"] * xytune
        )
        plotOrigin(
            float(tx.get()),
            float(ty.get()),
            float(tz.get()),
            float(rx.get()),
            float(ry.get()),
            float(rz.get()),
            totallength / 50 * globalsettings["Origin_size"] * xytune,
            ghost,
        )
    ax.set_xlabel("X-axis (mm)")
    ax.set_ylabel("Y-axis (mm)")
    ax.set_zlabel("Z-axis (mm)")
    ax.xaxis.label.set_color(
        "white" if ctk.get_appearance_mode() == "Dark" else "black"
    )
    ax.yaxis.label.set_color(
        "white" if ctk.get_appearance_mode() == "Dark" else "black"
    )
    ax.zaxis.label.set_color(
        "white" if ctk.get_appearance_mode() == "Dark" else "black"
    )
    canvas.draw()
    updateLog("Preview updated")


# plot axis origins onto point with rotation
def plotOrigin(tx, ty, tz, rx, ry, rz, length, ghost=False):
    # transform origin
    rx = np.radians(rx)
    ry = np.radians(ry)
    rz = np.radians(rz)
    rotx = ((1, 0, 0), (0, np.cos(rx), -np.sin(rx)), (0, np.sin(rx), np.cos(rx)))
    roty = ((np.cos(ry), 0, np.sin(ry)), (0, 1, 0), (-np.sin(ry), 0, np.cos(ry)))
    rotz = ((np.cos(rz), -np.sin(rz), 0), (np.sin(rz), np.cos(rz), 0), (0, 0, 1))
    rotall = np.matmul(np.matmul(rotz, roty), rotx)
    xplots = np.matmul(rotall, ((0, length), (0, 0), (0, 0)))
    yplots = np.matmul(rotall, ((0, 0), (0, length), (0, 0)))
    zplots = np.matmul(rotall, ((0, 0), (0, 0), (0, length)))
    xplots[0] += tx
    xplots[1] += ty
    xplots[2] += tz
    yplots[0] += tx
    yplots[1] += ty
    yplots[2] += tz
    zplots[0] += tx
    zplots[1] += ty
    zplots[2] += tz

    # plot origin
    ax.plot(
        *xplots,
        c=getHex("Colx_axs"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )
    ax.plot(
        *yplots,
        c=getHex("Coly_axs"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )
    ax.plot(
        *zplots,
        c=getHex("Colz_axs"),
        linewidth=globalsettings["Line_width"],
        alpha=globalsettings["Ghost_opacity"] / 100 if ghost else 1,
    )


# set settings saved in 'settings.txt'
def setSettings():
    global globalsettings
    file = open("settings.txt", "r")
    settings = {}
    for i in file:
        isname = True
        issetting = False
        name = ""
        setting = ""
        for j in i:
            if j == ":":
                isname = False
                issetting = True
                continue
            if j == "\n":
                break
            if isname:
                name += j
            elif issetting:
                setting += j
        try:
            settings[name] = int(setting)
        except:
            settings[name] = float(setting)
    file.close()
    globalsettings = settings


# save current settings to 'settings.txt'
def saveSettings(parent=None, tell=False):
    # collect all settings and perform data type verification
    error = False
    try:
        msg = "Configure Tx, Ty and Tz"
        globalsettings["Inv_t_step"] = float(inverseconfigstepsizet.get())
        msg = "Configure Rx, Ry and Rz"
        globalsettings["Inv_r_step"] = float(inverseconfigstepsizer.get())
        msg = "Configure A1, A2, A3, A4, A5 and A6"
        globalsettings["Fwd_step"] = float(forwardconfigstepsize.get())
        if parent:
            try:
                msg = "Axis 1 to Axis 2"
                globalsettings["A1_length"] = float(a1length.get())
                msg = "Axis 2 to Axis 3"
                globalsettings["A2_length"] = float(a2length.get())
                msg = "Axis 3 to Axis 4"
                globalsettings["A3_length"] = float(a3length.get())
                msg = "Axis 4 to Axis 5"
                globalsettings["A4_length"] = float(a4length.get())
                msg = "Axis 5 to Axis 6"
                globalsettings["A5_length"] = float(a5length.get())
                msg = "Axis 6 to End"
                globalsettings["A6_length"] = float(a6length.get())

                try:
                    msg = "Axis 1"
                    globalsettings["A1_constr_pos"] = float(a1constraintpositive.get())
                    globalsettings["A1_constr_neg"] = float(a1constraintnegative.get())
                    msg = "Axis 2"
                    globalsettings["A2_constr_pos"] = float(a2constraintpositive.get())
                    globalsettings["A2_constr_neg"] = float(a2constraintnegative.get())
                    msg = "Axis 3"
                    globalsettings["A3_constr_pos"] = float(a3constraintpositive.get())
                    globalsettings["A3_constr_neg"] = float(a3constraintnegative.get())
                    msg = "Axis 4"
                    globalsettings["A4_constr_pos"] = float(a4constraintpositive.get())
                    globalsettings["A4_constr_neg"] = float(a4constraintnegative.get())
                    msg = "Axis 5"
                    globalsettings["A5_constr_pos"] = float(a5constraintpositive.get())
                    globalsettings["A5_constr_neg"] = float(a5constraintnegative.get())
                    msg = "Axis 6"
                    globalsettings["A6_constr_pos"] = float(a6constraintpositive.get())
                    globalsettings["A6_constr_neg"] = float(a6constraintnegative.get())
                except:
                    error = True
                    messagebox.showerror(
                        title="An error occurred",
                        message=f"Constraint '{msg}' must be float",
                        parent=settingswindow,
                    )
                    settingswindow.focus_set()
            except:
                error = True
                messagebox.showerror(
                    title="An error occurred",
                    message=f"Length '{msg}' must be float",
                    parent=settingswindow,
                )
                settingswindow.focus_set()
    except:
        error = True
        messagebox.showerror(
            title="An error occurred",
            message=f"'{msg}' must be float",
            parent=root
        )
        root.focus_set()

    if parent:
        globalsettings["Marker_size"] = int(markersize.get())
        globalsettings["Line_width"] = int(linewidth.get())
        globalsettings["Show_origins"] = int(showoriginstate.get())
        globalsettings["Origin_size"] = int(originsize.get())
        globalsettings["Automate"] = int(automatestate.get())
        globalsettings["Update"] = int(updatestate.get())
        globalsettings["Corner_radius"] = int(cornerradius.get())
        globalsettings["Ghost"] = int(showghoststate.get())
        globalsettings["Ghost_opacity"] = int(ghostopacity.get())

    # perform other validation
    steperror = None
    if globalsettings["Inv_t_step"] < 0 and not error:
        steperror = "Tx, Ty and Tz"
    elif globalsettings["Inv_r_step"] < 0 and not error:
        steperror = "Rx, Ry and Rz"
    elif globalsettings["Fwd_step"] < 0 and not error:
        steperror = "A1, A2, A3, A4, A5 and A6"
    if steperror:
        error = True
        messagebox.showerror(
            title="An error occurred",
            message=f"'{steperror}' cannot be negative",
            parent=root
        )

    if parent:
        negativeerror = None
        if globalsettings["A1_length"] < 0 and not error:
            negativeerror = "Axis 1 to Axis 2"
        elif globalsettings["A2_length"] < 0 and not error:
            negativeerror = "Axis 2 to Axis 3"
        elif globalsettings["A3_length"] < 0 and not error:
            negativeerror = "Axis 3 to Axis 4"
        elif globalsettings["A4_length"] < 0 and not error:
            negativeerror = "Axis 4 to Axis 5"
        elif globalsettings["A5_length"] < 0 and not error:
            negativeerror = "Axis 5 to Axis 6"
        elif globalsettings["A6_length"] < 0 and not error:
            negativeerror = "Axis 6 to End"
        if negativeerror:
            error = True
            messagebox.showerror(
                "An error occurred",
                f"'{negativeerror}' cannot be negative",
                parent=settingswindow
            )
            settingswindow.focus_set()

        ordererror = None
        if (
            globalsettings["A1_constr_neg"] > globalsettings["A1_constr_pos"]
            and not error
        ):
            ordererror = "Axis 1"
        elif (
            globalsettings["A2_constr_neg"] > globalsettings["A2_constr_pos"]
            and not error
        ):
            ordererror = "Axis 2"
        elif (
            globalsettings["A3_constr_neg"] > globalsettings["A3_constr_pos"]
            and not error
        ):
            ordererror = "Axis 3"
        elif (
            globalsettings["A4_constr_neg"] > globalsettings["A4_constr_pos"]
            and not error
        ):
            ordererror = "Axis 4"
        elif (
            globalsettings["A5_constr_neg"] > globalsettings["A5_constr_pos"]
            and not error
        ):
            ordererror = "Axis 5"
        elif (
            globalsettings["A6_constr_neg"] > globalsettings["A6_constr_pos"]
            and not error
        ):
            ordererror = "Axis 6"
        if ordererror:
            error = True
            messagebox.showerror(
                title="An error occurred",
                message=f"'{ordererror}' must have the smaller value to the left",
                parent=settingswindow
            )
            settingswindow.focus_set()

    if parent == "colourswindow":
        globalsettings["Dimming"] = buttonsdimmer.get()
        globalsettings["Colour_blind"] = colourblindselector.get()
        globalsettings["Theme"] = themeselector.get()

    # save to file if no errors occurred
    if not error:
        file = open("settings.txt", "w")
        for i in globalsettings:
            file.write(f"{i}:{globalsettings[i]}\n")
        file.close()

        # finish
        if not tell and parent == "colourswindow":
            messagebox.showinfo(
                title="Restart",
                message="Please restart the program\nfor all changes to take effect",
                parent=colourswindow
            )
        elif tell:
            if parent == "settingswindow":
                messagebox.showinfo(
                    title="Success",
                    message="Settings saved",
                    parent=settingswindow
                )
                messagebox.showinfo(
                    "Restart",
                    "Please restart the program\nfor all changes to take effect",
                    parent=settingswindow
                )
                settingswindow.focus_set()
            elif parent == "colourswindow":
                messagebox.showinfo(
                    title="Success",
                    message="Settings saved",
                    parent=colourswindow
                )
                messagebox.showinfo(
                    "Restart",
                    "Please restart the program\nfor all changes to take effect",
                    parent=colourswindow
                )
                colourswindow.focus_set()
            else:
                messagebox.showinfo(
                    title="Success",
                    message="Settings saved",
                    parent=root
                )
                root.focus_set()
        updateLog("Settings saved")
        updateWindow()
    else:
        updateLog("Settings NOT saved due to ERROR")


# load settings from 'settings.txt'
def loadSettings(parent=None, tell=False, save=False, preset=None, colourblind=None, theme=None):
    confirm = True
    if preset:
        # change preset
        preset = int(preset) - 1
        performance = [2, 2, 0, 4, 0, 0, 0, 0, 0, 100]
        slim = [2, 2, 1, 16, 0, 1, 4, 1, 50, 500]
        normal = [4, 4, 1, 8, 0, 1, 7, 1, 40, 500]
        thick = [8, 8, 1, 8, 0, 1, 10, 1, 30, 500]
        presets = [
            performance,
            slim,
            normal,
            thick
        ]
        presetnames = [
            "Performance",
            "Slim",
            "Normal",
            "Thick"
        ]
        confirm = messagebox.askokcancel(
            title="Confirm",
            message="This will override current settings",
            parent=settingswindow
        )
        if confirm:
            globalsettings["Marker_size"] = presets[preset][0]
            globalsettings["Line_width"] = presets[preset][1]
            globalsettings["Show_origins"] = presets[preset][2]
            globalsettings["Origin_size"] = presets[preset][3]
            globalsettings["Automate"] = presets[preset][4]
            globalsettings["Update"] = presets[preset][5]
            globalsettings["Corner_radius"] = presets[preset][6]
            globalsettings["Ghost"] = presets[preset][7]
            globalsettings["Ghost_opacity"] = presets[preset][8]
            globalsettings["Lerp_resolution"] = presets[preset][9]
            loadSettings()

    elif colourblind:
        # change to colourblind
        colourblind = int(colourblind) - 1
        colournames = [
            "Colx_axs",
            "Coly_axs",
            "Colz_axs",
            "Col1_plt",
            "Col2_plt",
            "Col3_plt",
            "Col4_plt",
            "Col5_plt",
            "Col6_plt",
            "Col_btn_fg",
            "Col1_btn",
            "Col2_btn",
            "Col3_btn",
            "Col4_btn",
            "Col5_btn",
            "Col6_btn",
            "Col_estp_fg",
            "Col_estp_bg",
            "Col_btn_sc"
        ]
        none = [
            "#ff4444",
            "#44ff44",
            "#4444ff",
            "#ff7777",
            "#77ff77",
            "#7777ff",
            "#77ffff",
            "#ff77ff",
            "#ffff77",
            "#000000",
            "#ff7777",
            "#77ff77",
            "#7777ff",
            "#77ffff",
            "#ff77ff",
            "#ffff77",
            "#ffffff",
            "#ff4444",
            "#1199cc"
        ]
        protanope = [
            "#99ff00",
            "#0099ff",
            "#9900ff",
            "#ffaa00",
            "#aaff00",
            "#00ff88",
            "#00ddff",
            "#0088ff",
            "#0033ff",
            "#000000",
            "#ffaa00",
            "#aaff00",
            "#00ff88",
            "#00ddff",
            "#0088ff",
            "#0033ff",
            "#ffffff",
            "#0022ff",
            "#4477cc"
        ]
        deuteranope = [
            "#fff00",
            "#00ffff",
            "#0000ff",
            "#ffff00",
            "#22ff00",
            "#00ff99",
            "#00ffff",
            "#0099ff",
            "#0000ff",
            "#000000",
            "#ffff00",
            "#22ff00",
            "#00ff99",
            "#00ffff",
            "#0099ff",
            "#0000ff",
            "#ffffff",
            "#0022ff",
            "#9944bb"
        ]
        tritanope = [
            "#ff2200",
            "#00ccff",
            "#5500ff",
            "#ff2200",
            "#ee6622",
            "#44ee22",
            "#22eeaa",
            "#2288ee",
            "#2222ee",
            "#000000",
            "#ff2200",
            "#ee6622",
            "#44ee22",
            "#22eeaa",
            "#2288ee",
            "#2222ee",
            "#ffffff",
            "#ff0000",
            "#2277aa"
        ]
        colourblindset = [
            none,
            protanope,
            deuteranope,
            tritanope
        ]
        confirm = messagebox.askokcancel(
            title="Confirm",
            message="This will override current settings",
            parent=colourswindow
        )
        if confirm:
            for i in range(len(colournames)):
                hexcolour = colourblindset[colourblind][i]
                globalsettings[f"{colournames[i]}_r"] = int(f"0x{hexcolour[1:3]}", 0)
                globalsettings[f"{colournames[i]}_g"] = int(f"0x{hexcolour[3:5]}", 0)
                globalsettings[f"{colournames[i]}_b"] = int(f"0x{hexcolour[5:7]}", 0)

            globalsettings["Colour_blind"] = colourblind + 1

            updateSample()

    elif theme:
        # change theme
        theme = int(theme)
        globalsettings["Theme"] = theme

    else:
        # delete old values
        inverseconfigstepsizet.delete(0, END)
        inverseconfigstepsizer.delete(0, END)
        forwardconfigstepsize.delete(0, END)
        a1length.delete(0, END)
        a2length.delete(0, END)
        a3length.delete(0, END)
        a4length.delete(0, END)
        a5length.delete(0, END)
        a6length.delete(0, END)
        a1constraintpositive.delete(0, END)
        a1constraintnegative.delete(0, END)
        a2constraintpositive.delete(0, END)
        a2constraintnegative.delete(0, END)
        a3constraintpositive.delete(0, END)
        a3constraintnegative.delete(0, END)
        a4constraintpositive.delete(0, END)
        a4constraintnegative.delete(0, END)
        a5constraintpositive.delete(0, END)
        a5constraintnegative.delete(0, END)
        a6constraintpositive.delete(0, END)
        a6constraintnegative.delete(0, END)

        # insert new values
        inverseconfigstepsizet.insert(0, globalsettings["Inv_t_step"])
        inverseconfigstepsizer.insert(0, globalsettings["Inv_r_step"])
        forwardconfigstepsize.insert(0, globalsettings["Fwd_step"])
        a1length.insert(0, globalsettings["A1_length"])
        a2length.insert(0, globalsettings["A2_length"])
        a3length.insert(0, globalsettings["A3_length"])
        a4length.insert(0, globalsettings["A4_length"])
        a5length.insert(0, globalsettings["A5_length"])
        a6length.insert(0, globalsettings["A6_length"])
        a1constraintpositive.insert(0, globalsettings["A1_constr_pos"])
        a1constraintnegative.insert(0, globalsettings["A1_constr_neg"])
        a2constraintpositive.insert(0, globalsettings["A2_constr_pos"])
        a2constraintnegative.insert(0, globalsettings["A2_constr_neg"])
        a3constraintpositive.insert(0, globalsettings["A3_constr_pos"])
        a3constraintnegative.insert(0, globalsettings["A3_constr_neg"])
        a4constraintpositive.insert(0, globalsettings["A4_constr_pos"])
        a4constraintnegative.insert(0, globalsettings["A4_constr_neg"])
        a5constraintpositive.insert(0, globalsettings["A5_constr_pos"])
        a5constraintnegative.insert(0, globalsettings["A5_constr_neg"])
        a6constraintpositive.insert(0, globalsettings["A6_constr_pos"])
        a6constraintnegative.insert(0, globalsettings["A6_constr_neg"])
        markersize.set(globalsettings["Marker_size"])
        linewidth.set(globalsettings["Line_width"])
        showorigin.select() if globalsettings["Show_origins"] == 1.0 else showorigin.deselect()
        originsize.set(globalsettings["Origin_size"])
        automate.select() if globalsettings["Automate"] == 1.0 else automate.deselect()
        update.select() if globalsettings["Update"] == 1.0 else update.deselect()
        cornerradius.set(globalsettings["Corner_radius"])
        showghost.select() if globalsettings["Ghost"] == 1.0 else showghost.deselect()
        ghostopacity.set(globalsettings["Ghost_opacity"])
        lerpresolution.set(globalsettings["Lerp_resolution"])
        presetselector.set(1)

    # finish
    if confirm:
        if tell:
            if parent == "settingswindow":
                messagebox.showinfo(
                    title="Success",
                    message=f"{(presetnames[preset] + ' preset') if preset else 'Settings'} loaded",
                    parent=settingswindow
                )
                messagebox.showinfo(
                    title="Restart",
                    message="Please restart the program\nfor all changes to take effect",
                    parent=settingswindow
                )
                settingswindow.focus_set()
            elif parent == "colourswindow":
                messagebox.showinfo(
                    title="Success",
                    message=f"{(presetnames[preset] + ' preset') if preset else 'Settings'} loaded",
                    parent=colourswindow
                )
                colourswindow.focus_set()
            else:
                messagebox.showinfo(
                    title="Success",
                    message=f"{(presetnames[preset] + ' preset') if preset else 'Settings'} loaded",
                    parent=root
                )
                root.focus_set()
        updateLog(
            f"{(presetnames[preset] + ' preset') if preset else 'Settings'} loaded"
        )
        if bool(globalsettings["Update"]):
            if globalsettings["Ghost"] == 1.0:
                plotData(True)
            plotData()
        updateWindow()
        if save:
            saveSettings(parent=parent, tell=False)
    else:
        if parent == "settingswindow":
            settingswindow.focus_set()
        elif parent == "colourswindow":
            colourswindow.focus_set()
        else:
            root.focus_set()


# open settings window
def openSettingsWindow():
    global settingswindow
    global a1length
    global a2length
    global a3length
    global a4length
    global a5length
    global a6length
    global a1constraintpositive
    global a1constraintnegative
    global a2constraintpositive
    global a2constraintnegative
    global a3constraintpositive
    global a3constraintnegative
    global a4constraintpositive
    global a4constraintnegative
    global a5constraintpositive
    global a5constraintnegative
    global a6constraintpositive
    global a6constraintnegative
    global markersize
    global linewidth
    global showoriginstate
    global showorigin
    global originsize
    global automatestate
    global automate
    global updatestate
    global update
    global cornerradius
    global showghoststate
    global showghost
    global ghostopacity
    global lerpresolution
    global presetselector
    global presetimg

    # unminimise and focus window if already exists
    try:
        settingswindow.deiconify()
        return
    except:
        pass

    # create settings window
    settingswindow = Toplevel(root)
    settingswindow.title("Settings")
    settingswindow.wm_iconphoto(
        False,
        ImageTk.PhotoImage(
            Image.open("img/icon settings.png")
        )
    )
    settingswindow.resizable(width=0, height=0)
    settingswindow.geometry("+70+180")
    settingswindow.configure(
        bg="#323232" if ctk.get_appearance_mode() == "Dark" else "#ececec"
    )

    # define tkinter variables
    automatestate = IntVar()
    updatestate = IntVar()
    showoriginstate = IntVar()
    showghoststate = IntVar()

    # create kwargs that will be used frequently
    kwargs = {
        "corner_radius": radius,
        "fg_color": ("#d4d5d6", "#3f3f3f"),
        "justify": LEFT,
    }
    widgetpack = {
        "padx": 8,
        "pady": 8,
        "sticky": N + S + E + W}
    fontcolour = getHex("Col_btn_fg")

    # create settings frames
    lengthsframe = ctk.CTkFrame(
        master=settingswindow,
        width=272,
        height=279,
        corner_radius=radius
    )
    constraintsframe = ctk.CTkFrame(
        master=settingswindow,
        width=272,
        height=279,
        corner_radius=radius
    )
    preferencesframe = ctk.CTkFrame(
        master=settingswindow,
        width=332,
        height=250,
        corner_radius=radius
    )
    settingsactionsframe = ctk.CTkFrame(
        master=settingswindow,
        width=272,
        height=48,
        corner_radius=radius
    )
    presetsframe = ctk.CTkFrame(
        master=settingswindow,
        width=332,
        height=226,
        corner_radius=radius
    )
    coloursframe = ctk.CTkFrame(
        master=settingswindow,
        width=332,
        height=48,
        corner_radius=radius
    )

    # create lengths widgets
    lengthslabel = ctk.CTkLabel(
        master=lengthsframe,
        text="Lengths",
        width=60,
        **kwargs
    )
    a1lengthlabel = ctk.CTkLabel(
        master=lengthsframe,
        text="Axis 1 to Axis 2",
        width=100,
        **kwargs
    )
    a2lengthlabel = ctk.CTkLabel(
        master=lengthsframe,
        text="Axis 2 to Axis 3",
        width=100,
        **kwargs
    )
    a3lengthlabel = ctk.CTkLabel(
        master=lengthsframe,
        text="Axis 3 to Axis 4",
        width=100,
        **kwargs
    )
    a4lengthlabel = ctk.CTkLabel(
        master=lengthsframe,
        text="Axis 4 to Axis 5",
        width=100,
        **kwargs
    )
    a5lengthlabel = ctk.CTkLabel(
        master=lengthsframe,
        text="Axis 5 to Axis 6",
        width=100,
        **kwargs
    )
    a6lengthlabel = ctk.CTkLabel(
        master=lengthsframe,
        text="Axis 6 to end",
        width=100,
        **kwargs
    )
    a1length = ctk.CTkEntry(
        master=lengthsframe,
        fg_color=getHex("Col1_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=110,
        corner_radius=radius
    )
    a2length = ctk.CTkEntry(
        master=lengthsframe,
        fg_color=getHex("Col2_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=110,
        corner_radius=radius
    )
    a3length = ctk.CTkEntry(
        master=lengthsframe,
        fg_color=getHex("Col3_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=110,
        corner_radius=radius
    )
    a4length = ctk.CTkEntry(
        master=lengthsframe,
        fg_color=getHex("Col4_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=110,
        corner_radius=radius
    )
    a5length = ctk.CTkEntry(
        master=lengthsframe,
        fg_color=getHex("Col5_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=110,
        corner_radius=radius
    )
    a6length = ctk.CTkEntry(
        master=lengthsframe,
        fg_color=getHex("Col6_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=110,
        corner_radius=radius,
    )
    a1lengthsunit = ctk.CTkLabel(
        master=lengthsframe,
        text="mm",
        width=30,
        **kwargs)
    a2lengthsunit = ctk.CTkLabel(
        master=lengthsframe,
        text="mm",
        width=30,
        **kwargs)
    a3lengthsunit = ctk.CTkLabel(
        master=lengthsframe,
        text="mm",
        width=30,
        **kwargs
    )
    a4lengthsunit = ctk.CTkLabel(
        master=lengthsframe,
        text="mm",
        width=30,
        **kwargs
    )
    a5lengthsunit = ctk.CTkLabel(
        master=lengthsframe,
        text="mm",
        width=30,
        **kwargs
    )
    a6lengthsunit = ctk.CTkLabel(
        master=lengthsframe,
        text="mm",
        width=30,
        **kwargs
    )

    # create constraints widgets
    constraintslabel = ctk.CTkLabel(
        master=constraintsframe,
        text="Constraints",
        width=80,
        **kwargs
    )
    a1constraintlabel1 = ctk.CTkLabel(
        master=constraintsframe,
        text="Axis 1",
        width=40,
        **kwargs
    )
    a2constraintlabel1 = ctk.CTkLabel(
        master=constraintsframe,
        text="Axis 2",
        width=40,
        **kwargs
    )
    a3constraintlabel1 = ctk.CTkLabel(
        master=constraintsframe,
        text="Axis 3",
        width=40,
        **kwargs
    )
    a4constraintlabel1 = ctk.CTkLabel(
        master=constraintsframe,
        text="Axis 4",
        width=40,
        **kwargs
    )
    a5constraintlabel1 = ctk.CTkLabel(
        master=constraintsframe,
        text="Axis 5",
        width=40,
        **kwargs
    )
    a6constraintlabel1 = ctk.CTkLabel(
        master=constraintsframe,
        text="Axis 6",
        width=40,
        **kwargs
    )
    a1constraintpositive = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col1_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a2constraintpositive = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col2_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a3constraintpositive = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col3_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a4constraintpositive = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col4_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a5constraintpositive = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col5_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a6constraintpositive = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col6_btn"),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a1constraintlabel2 = ctk.CTkLabel(
        master=constraintsframe,
        text="≤ θ ≤",
        width=40,
        **kwargs
    )
    a2constraintlabel2 = ctk.CTkLabel(
        master=constraintsframe,
        text="≤ θ ≤",
        width=40,
        **kwargs
    )
    a3constraintlabel2 = ctk.CTkLabel(
        master=constraintsframe,
        text="≤ θ ≤",
        width=40,
        **kwargs
    )
    a4constraintlabel2 = ctk.CTkLabel(
        master=constraintsframe,
        text="≤ θ ≤",
        width=40,
        **kwargs
    )
    a5constraintlabel2 = ctk.CTkLabel(
        master=constraintsframe,
        text="≤ θ ≤",
        width=40,
        **kwargs
    )
    a6constraintlabel2 = ctk.CTkLabel(
        master=constraintsframe,
        text="≤ θ ≤",
        width=40,
        **kwargs
    )
    a1constraintnegative = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col1_btn", True),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a2constraintnegative = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col2_btn", True),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a3constraintnegative = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col3_btn", True),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a4constraintnegative = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col4_btn", True),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a5constraintnegative = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col5_btn", True),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a6constraintnegative = ctk.CTkEntry(
        master=constraintsframe,
        fg_color=getHex("Col6_btn", True),
        text_color=fontcolour,
        justify=RIGHT,
        width=70,
        corner_radius=radius,
    )
    a1constraintsunit = ctk.CTkLabel(
        master=constraintsframe,
        text="°",
        width=10,
        **kwargs
    )
    a2constraintsunit = ctk.CTkLabel(
        master=constraintsframe,
        text="°",
        width=10,
        **kwargs
    )
    a3constraintsunit = ctk.CTkLabel(
        master=constraintsframe,
        text="°",
        width=10,
        **kwargs
    )
    a4constraintsunit = ctk.CTkLabel(
        master=constraintsframe,
        text="°",
        width=10,
        **kwargs
    )
    a5constraintsunit = ctk.CTkLabel(
        master=constraintsframe,
        text="°",
        width=10,
        **kwargs
    )
    a6constraintsunit = ctk.CTkLabel(
        master=constraintsframe,
        text="°",
        width=10,
        **kwargs
    )

    # create preferences widgets
    preferenceslabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Preferences",
        width=80,
        **kwargs
    )
    markersizelabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Marker size",
        width=150,
        **kwargs
    )
    markersize = ctk.CTkSlider(
        master=preferencesframe,
        from_=1,
        to=8,
        number_of_steps=7,
        width=150
    )
    linewidthlabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Line width",
        width=150,
        **kwargs
    )
    linewidth = ctk.CTkSlider(
        master=preferencesframe,
        from_=1,
        to=8,
        number_of_steps=7,
        width=150
    )
    showoriginlabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Show origins",
        width=150,
        **kwargs
    )
    showorigin = ctk.CTkCheckBox(
        master=preferencesframe,
        text="",
        variable=showoriginstate
    )
    originsizelabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Origin size",
        width=150,
        **kwargs
    )
    originsize = ctk.CTkSlider(
        master=preferencesframe,
        from_=4,
        to=16,
        number_of_steps=12,
        width=150
    )
    automatelabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Travel on calculation",
        width=150,
        **kwargs
    )
    automate = ctk.CTkCheckBox(
        master=preferencesframe,
        text="",
        variable=automatestate
    )
    updatelabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Live preview",
        width=150,
        **kwargs
    )
    update = ctk.CTkCheckBox(
        master=preferencesframe,
        text="",
        variable=updatestate
    )
    cornerradiuslabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Corner radius",
        width=150,
        **kwargs
    )
    cornerradius = ctk.CTkSlider(
        master=preferencesframe,
        from_=0,
        to=10,
        number_of_steps=10,
        width=150
    )
    showghostlabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Show ghost",
        width=150,
        **kwargs
    )
    showghost = ctk.CTkCheckBox(
        master=preferencesframe,
        text="",
        variable=showghoststate
    )
    ghostopacitylabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Ghost opacity",
        width=150,
        **kwargs
    )
    ghostopacity = ctk.CTkSlider(
        master=preferencesframe,
        from_=0,
        to=100,
        number_of_steps=10,
        width=150
    )
    lerpresolutionlabel = ctk.CTkLabel(
        master=preferencesframe,
        text="Lerp resolution",
        width=150,
        **kwargs
    )
    lerpresolution = ctk.CTkSlider(
        master=preferencesframe,
        from_=100,
        to=1000,
        number_of_steps=9,
        width=150
    )

    # create saving widgets
    savebutton = ctk.CTkButton(
        master=settingsactionsframe,
        text="Save",
        command=lambda: saveSettings(parent="settingswindow", tell=True),
        corner_radius=radius,
    )
    loadsavebutton = ctk.CTkButton(
        master=settingsactionsframe,
        text="Load Saved",
        command=lambda: loadSettings(parent="settingswindow", tell=True),
        corner_radius=radius,
    )

    # create presets widgets
    presetlabel = ctk.CTkLabel(
        master=presetsframe,
        text="Presets",
        width=50,
        **kwargs
    )
    presetselector = ctk.CTkSlider(
        master=presetsframe,
        from_=1,
        to=4,
        number_of_steps=3,
        width=316
    )
    imgsrc = Image.open(f"img/img presets {ctk.get_appearance_mode().lower()}.png")
    presetimg = ImageTk.PhotoImage(imgsrc)
    presetimage = ctk.CTkLabel(
        master=presetsframe,
        image=presetimg,
        height=60,
        **kwargs
    )
    selectbutton = ctk.CTkButton(
        master=presetsframe,
        text="Load Preset and Save",
        command=lambda: loadSettings(
            parent="settingswindow",
            tell=True,
            save=True,
            preset=presetselector.get()
        ),
        width=60,
        corner_radius=radius
    )

    # create colours widgets
    colourslabel = ctk.CTkLabel(
        master=coloursframe,
        text="Colours",
        width=50,
        **kwargs
    )
    opencoloursbutton = ctk.CTkButton(
        master=coloursframe,
        text="Edit Colours",
        command=openColoursWindow,
        width=150,
        corner_radius=radius,
    )

    # configure settings grid system
    settingswindow.grid_columnconfigure(0, weight=0)
    settingswindow.grid_rowconfigure(0, weight=0)
    settingswindow.grid_rowconfigure(1, weight=0)
    settingswindow.grid_rowconfigure(2, weight=0)
    settingswindow.grid_rowconfigure(3, weight=1)
    settingswindow.grid_rowconfigure(4, weight=0)

    # configure lengths grid system
    lengthsframe.grid_columnconfigure(0, weight=0)
    lengthsframe.grid_columnconfigure(1, weight=1)
    lengthsframe.grid_columnconfigure(2, weight=0)
    lengthsframe.grid_columnconfigure(3, weight=0)

    # configure constraints grid system
    constraintsframe.grid_columnconfigure(0, weight=0)
    constraintsframe.grid_columnconfigure(1, weight=0)
    constraintsframe.grid_columnconfigure(2, weight=0)
    constraintsframe.grid_columnconfigure(3, weight=0)
    constraintsframe.grid_columnconfigure(4, weight=0)
    constraintsframe.grid_columnconfigure(5, weight=0)

    # configure preferences grid system
    preferencesframe.grid_columnconfigure(0, weight=0)
    preferencesframe.grid_columnconfigure(1, weight=1)
    preferencesframe.grid_columnconfigure(2, weight=0)

    # configure actions grid system
    settingsactionsframe.grid_columnconfigure(0, weight=1)
    settingsactionsframe.grid_columnconfigure(1, weight=1)

    # configure colours grid system
    coloursframe.grid_columnconfigure(0, weight=0)
    coloursframe.grid_columnconfigure(1, weight=1)
    coloursframe.grid_columnconfigure(2, weight=0)

    # configure presets grid system
    presetsframe.grid_columnconfigure(0, weight=0)
    presetsframe.grid_columnconfigure(1, weight=1)

    # pack settings frames
    lengthsframe.grid(
        row=0,
        column=0,
        **widgetpack)
    constraintsframe.grid(
        row=1,
        column=0,
        **widgetpack,
        rowspan=3
    )
    preferencesframe.grid(
        row=0,
        column=1,
        rowspan=2,
        **widgetpack
    )
    settingsactionsframe.grid(
        row=4,
        column=0,
        **widgetpack
    )
    coloursframe.grid(
        row=2,
        column=1,
        **widgetpack
    )
    presetsframe.grid(
        row=3,
        column=1,
        rowspan=2,
        **widgetpack
    )

    # pack lengths widgets
    lengthslabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    a1lengthlabel.grid(
        row=1,
        column=0,
        columnspan=2,
        **widgetpack
    )
    a2lengthlabel.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    a3lengthlabel.grid(
        row=3,
        column=0,
        columnspan=2,
        **widgetpack
    )
    a4lengthlabel.grid(
        row=4,
        column=0,
        columnspan=2,
        **widgetpack
    )
    a5lengthlabel.grid(
        row=5,
        column=0,
        columnspan=2,
        **widgetpack
    )
    a6lengthlabel.grid(
        row=6,
        column=0,
        columnspan=2,
        **widgetpack
    )
    a1length.grid(
        row=1,
        column=2,
        padx=(8, 0),
        pady=8,
        sticky=N + S + E + W
    )
    a2length.grid(
        row=2,
        column=2,
        padx=(8, 0),
        pady=8,
        sticky=N + S + E + W
    )
    a3length.grid(
        row=3,
        column=2,
        padx=(8, 0),
        pady=8,
        sticky=N + S + E + W
    )
    a4length.grid(
        row=4,
        column=2,
        padx=(8, 0),
        pady=8,
        sticky=N + S + E + W
    )
    a5length.grid(
        row=5,
        column=2,
        padx=(8, 0),
        pady=8,
        sticky=N + S + E + W
    )
    a6length.grid(
        row=6,
        column=2,
        padx=(8, 0),
        pady=8,
        sticky=N + S + E + W
    )
    a1lengthsunit.grid(
        row=1,
        column=3,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a2lengthsunit.grid(
        row=2,
        column=3,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a3lengthsunit.grid(
        row=3,
        column=3,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a4lengthsunit.grid(
        row=4,
        column=3,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a5lengthsunit.grid(
        row=5,
        column=3,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a6lengthsunit.grid(
        row=6,
        column=3,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )

    # pack constraints widgets
    constraintslabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        columnspan=2,
        sticky=N + S + E + W
    )
    a1constraintlabel1.grid(
        row=1,
        column=0,
        **widgetpack
    )
    a2constraintlabel1.grid(
        row=2,
        column=0,
        **widgetpack
    )
    a3constraintlabel1.grid(
        row=3,
        column=0,
        **widgetpack
    )
    a4constraintlabel1.grid(
        row=4,
        column=0,
        **widgetpack
    )
    a5constraintlabel1.grid(
        row=5,
        column=0,
        **widgetpack
    )
    a6constraintlabel1.grid(
        row=6,
        column=0,
        **widgetpack
    )
    a1constraintpositive.grid(
        row=1,
        column=4,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a2constraintpositive.grid(
        row=2,
        column=4,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a3constraintpositive.grid(
        row=3,
        column=4,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a4constraintpositive.grid(
        row=4,
        column=4,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a5constraintpositive.grid(
        row=5,
        column=4,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a6constraintpositive.grid(
        row=6,
        column=4,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a1constraintlabel2.grid(
        row=1,
        column=3,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a2constraintlabel2.grid(
        row=2,
        column=3,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a3constraintlabel2.grid(
        row=3,
        column=3,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a4constraintlabel2.grid(
        row=4,
        column=3,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a5constraintlabel2.grid(
        row=5,
        column=3,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a6constraintlabel2.grid(
        row=6,
        column=3,
        padx=0,
        pady=8,
        sticky=N + S + E + W
    )
    a1constraintnegative.grid(
        row=1,
        column=1,
        padx=(8, 0),
        pady=8,
        columnspan=2,
        sticky=N + S + E + W
    )
    a2constraintnegative.grid(
        row=2,
        column=1,
        padx=(8, 0),
        pady=8,
        columnspan=2,
        sticky=N + S + E + W
    )
    a3constraintnegative.grid(
        row=3,
        column=1,
        padx=(8, 0),
        pady=8,
        columnspan=2,
        sticky=N + S + E + W
    )
    a4constraintnegative.grid(
        row=4,
        column=1,
        padx=(8, 0),
        pady=8,
        columnspan=2,
        sticky=N + S + E + W
    )
    a5constraintnegative.grid(
        row=5,
        column=1,
        padx=(8, 0),
        pady=8,
        columnspan=2,
        sticky=N + S + E + W
    )
    a6constraintnegative.grid(
        row=6,
        column=1,
        padx=(8, 0),
        pady=8,
        columnspan=2,
        sticky=N + S + E + W
    )
    a1constraintsunit.grid(
        row=1,
        column=5,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a2constraintsunit.grid(
        row=2,
        column=5,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a3constraintsunit.grid(
        row=3,
        column=5,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a4constraintsunit.grid(
        row=4,
        column=5,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a5constraintsunit.grid(
        row=5,
        column=5,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )
    a6constraintsunit.grid(
        row=6,
        column=5,
        padx=(0, 8),
        pady=8,
        sticky=N + S + E + W
    )

    # pack preferences widgets
    preferenceslabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    markersizelabel.grid(
        row=1,
        column=0,
        columnspan=2,
        **widgetpack
    )
    markersize.grid(
        row=1,
        column=2,
        padx=8,
        pady=12,
        sticky=N + S + E + W
    )
    linewidthlabel.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    linewidth.grid(
        row=2,
        column=2,
        padx=8,
        pady=12,
        sticky=N + S + E + W
    )
    showoriginlabel.grid(
        row=3,
        column=0,
        columnspan=2,
        **widgetpack
    )
    showorigin.grid(
        row=3,
        column=2,
        **widgetpack
    )
    originsizelabel.grid(
        row=4,
        column=0,
        columnspan=2,
        **widgetpack
    )
    originsize.grid(
        row=4,
        column=2,
        padx=8,
        pady=12,
        sticky=N + S + E + W
    )
    automatelabel.grid(
        row=5,
        column=0,
        columnspan=2,
        **widgetpack
    )
    automate.grid(
        row=5,
        column=2,
        **widgetpack
    )
    updatelabel.grid(
        row=6,
        column=0,
        columnspan=2,
        **widgetpack
    )
    update.grid(
        row=6,
        column=2,
        **widgetpack
    )
    cornerradiuslabel.grid(
        row=7,
        column=0,
        columnspan=2,
        **widgetpack
    )
    cornerradius.grid(
        row=7,
        column=2,
        padx=8,
        pady=12,
        sticky=N + S + E + W
    )
    showghostlabel.grid(
        row=8,
        column=0,
        columnspan=2,
        **widgetpack
    )
    showghost.grid(
        row=8,
        column=2,
        **widgetpack
    )
    ghostopacitylabel.grid(
        row=9,
        column=0,
        columnspan=2,
        **widgetpack
    )
    ghostopacity.grid(
        row=9,
        column=2,
        padx=8,
        pady=12,
        sticky=N + S + E + W
    )
    lerpresolutionlabel.grid(
        row=10,
        column=0,
        columnspan=2,
        **widgetpack
    )
    lerpresolution.grid(
        row=10,
        column=2,
        padx=8,
        pady=12,
        sticky=N + S + E + W
    )

    # pack saving widgets
    savebutton.grid(
        row=0,
        column=0,
        **widgetpack
    )
    loadsavebutton.grid(
        row=0,
        column=1,
        **widgetpack
    )

    # pack colours widgets
    colourslabel.grid(
        row=0,
        column=0,
        **widgetpack
    )
    opencoloursbutton.grid(
        row=0,
        column=2,
        **widgetpack
    )

    # pack presets widgets
    presetlabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    presetselector.grid(
        row=1,
        column=0,
        padx=8,
        pady=(8, 0),
        columnspan=2,
        sticky=N + S + E + W
    )
    presetimage.grid(
        row=2,
        column=0,
        padx=8,
        pady=(0, 8),
        columnspan=2,
        sticky=N + S + E + W
    )
    selectbutton.grid(
        row=3,
        column=0,
        columnspan=2,
        **widgetpack
    )

    # finish
    loadSettings()
    settingswindow.focus_set()


# open colour settings window
def openColoursWindow():
    global colourswindow
    global previewxcode
    global previewycode
    global previewzcode
    global preview1code
    global preview2code
    global preview3code
    global preview4code
    global preview5code
    global preview6code
    global previewecode
    global previewfgcode
    global previewbgcode
    global previewlncode
    global buttonsfgcode
    global buttons1code
    global buttons2code
    global buttons3code
    global buttons4code
    global buttons5code
    global buttons6code
    global buttonsdimmer
    global buttonsestpfgcode
    global buttonsestpbgcode
    global buttonssccode
    global samplexaxs
    global sampleyaxs
    global samplezaxs
    global sample1plt
    global sample2plt
    global sample3plt
    global sample4plt
    global sample5plt
    global sample6plt
    global sampleeplt
    global samplefgplt
    global samplebgplt
    global samplelnplt
    global samplefgbtn
    global sample1btnl
    global sample1btnd
    global sample2btnl
    global sample2btnd
    global sample3btnl
    global sample3btnd
    global sample4btnl
    global sample4btnd
    global sample5btnl
    global sample5btnd
    global sample6btnl
    global sample6btnd
    global sampleestpfg
    global sampleestpbg
    global samplesc
    global sampleschover
    global colourblindselector
    global colourblindimg
    global themeselector
    global themeimg

    # unminimise and focus window if already exists
    try:
        colourswindow.deiconify()
        return
    except:
        pass

    # create settings window
    colourswindow = Toplevel(settingswindow)
    colourswindow.title("Colours")
    colourswindow.wm_iconphoto(
        False,
        ImageTk.PhotoImage(
            Image.open("img/icon colours.png")
        )
    )
    colourswindow.resizable(width=0, height=0)
    colourswindow.geometry("+100+210")
    colourswindow.configure(
        bg="#323232" if ctk.get_appearance_mode() == "Dark" else "#ececec"
    )

    # create kwargs that will be used frequently
    kwargs = {
        "corner_radius": radius,
        "fg_color": ("#d4d5d6", "#3f3f3f"),
        "justify": LEFT,
    }
    samplekwargs = {
        "corner_radius": radius,
        "width": 120
    }
    widgetpack = {
        "padx": 8,
        "pady": 8,
        "sticky": N + S + E + W
    }

    # create colour frames
    previewframe = ctk.CTkFrame(
        master=colourswindow,
        width=308,
        height=200,
        corner_radius=radius
    )
    buttonsframe = ctk.CTkFrame(
        master=colourswindow,
        width=308,
        height=495,
        corner_radius=radius
    )
    actionsframe = ctk.CTkFrame(
        master=colourswindow,
        width=308,
        height=48,
        corner_radius=radius
    )
    colourblindframe = ctk.CTkFrame(
        master=colourswindow,
        width=308,
        height=155,
        corner_radius=radius
    )
    themesframe = ctk.CTkFrame(
        master=colourswindow,
        width=308,
        height=60,
        corner_radius=radius
    )
    sampleframe = ctk.CTkFrame(
        master=colourswindow,
        width=232,
        height=400,
        corner_radius=radius
    )

    # create preview widgets
    previewlabel = ctk.CTkLabel(
        master=previewframe,
        text="Preview",
        width=60,
        **kwargs
    )
    previewxlabel = ctk.CTkLabel(
        master=previewframe,
        text="X-axis",
        width=130,
        **kwargs
    )
    previewylabel = ctk.CTkLabel(
        master=previewframe,
        text="Y-axis",
        width=130,
        **kwargs
    )
    previewzlabel = ctk.CTkLabel(
        master=previewframe,
        text="Z-axis",
        width=130,
        **kwargs
    )
    preview1label = ctk.CTkLabel(
        master=previewframe,
        text="Colour 1",
        width=130,
        **kwargs
    )
    preview2label = ctk.CTkLabel(
        master=previewframe,
        text="Colour 2",
        width=130,
        **kwargs
    )
    preview3label = ctk.CTkLabel(
        master=previewframe,
        text="Colour 3",
        width=130,
        **kwargs
    )
    preview4label = ctk.CTkLabel(
        master=previewframe,
        text="Colour 4",
        width=130,
        **kwargs
    )
    preview5label = ctk.CTkLabel(
        master=previewframe,
        text="Colour 5",
        width=130,
        **kwargs
    )
    preview6label = ctk.CTkLabel(
        master=previewframe,
        text="Colour 6",
        width=130,
        **kwargs
    )
    previewxcode = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Colx_axs"),
        width=60,
        **kwargs
    )
    previewycode = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Coly_axs"),
        width=60,
        **kwargs
    )
    previewzcode = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Colz_axs"),
        width=60,
        **kwargs
    )
    preview1code = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Col1_plt"),
        width=60,
        **kwargs
    )
    preview2code = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Col2_plt"),
        width=60,
        **kwargs
    )
    preview3code = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Col3_plt"),
        width=60,
        **kwargs
    )
    preview4code = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Col4_plt"),
        width=60,
        **kwargs
    )
    preview5code = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Col5_plt"),
        width=60,
        **kwargs
    )
    preview6code = ctk.CTkLabel(
        master=previewframe,
        text=getHex("Col6_plt"),
        width=60,
        **kwargs
    )
    previewxbutton = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Colx_axs"),
        width=70,
        corner_radius=radius
    )
    previewybutton = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Coly_axs"),
        width=70,
        corner_radius=radius
    )
    previewzbutton = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Colz_axs"),
        width=70,
        corner_radius=radius
    )
    preview1button = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Col1_plt"),
        width=70,
        corner_radius=radius
    )
    preview2button = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Col2_plt"),
        width=70,
        corner_radius=radius
    )
    preview3button = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Col3_plt"),
        width=70,
        corner_radius=radius
    )
    preview4button = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Col4_plt"),
        width=70,
        corner_radius=radius
    )
    preview5button = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Col5_plt"),
        width=70,
        corner_radius=radius
    )
    preview6button = ctk.CTkButton(
        master=previewframe,
        text="Change",
        command=lambda: selectColour("Col6_plt"),
        width=70,
        corner_radius=radius
    )

    # create buttons widgets
    buttonslabel = ctk.CTkLabel(
        master=buttonsframe,
        text="Buttons",
        width=60,
        **kwargs
    )
    buttonsfglabel = ctk.CTkLabel(
        master=buttonsframe,
        text="Foreground",
        width=130,
        **kwargs
    )
    buttons1label = ctk.CTkLabel(
        master=buttonsframe,
        text="Colour 1",
        width=130,
        **kwargs
    )
    buttons2label = ctk.CTkLabel(
        master=buttonsframe,
        text="Colour 2",
        width=130,
        **kwargs
    )
    buttons3label = ctk.CTkLabel(
        master=buttonsframe,
        text="Colour 3",
        width=130,
        **kwargs
    )
    buttons4label = ctk.CTkLabel(
        master=buttonsframe,
        text="Colour 4",
        width=130,
        **kwargs
    )
    buttons5label = ctk.CTkLabel(
        master=buttonsframe,
        text="Colour 5",
        width=130,
        **kwargs
    )
    buttons6label = ctk.CTkLabel(
        master=buttonsframe,
        text="Colour 6",
        width=130,
        **kwargs
    )
    buttonsestpfglabel = ctk.CTkLabel(
        master=buttonsframe,
        text="E-Stop foreground",
        width=130,
        **kwargs
    )
    buttonsestpbglabel = ctk.CTkLabel(
        master=buttonsframe,
        text="E-Stop background",
        width=130,
        **kwargs
    )
    buttonssclabel = ctk.CTkLabel(
        master=buttonsframe,
        text="Secondary",
        width=130,
        **kwargs
    )
    buttonsdimmerlabel = ctk.CTkLabel(
        master=buttonsframe,
        text="Dimming (% of original)",
        width=150,
        **kwargs
    )
    buttonsfgcode = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col_btn_fg"),
        width=60,
        **kwargs
    )
    buttons1code = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col1_btn"),
        width=60,
        **kwargs
    )
    buttons2code = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col2_btn"),
        width=60,
        **kwargs
    )
    buttons3code = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col3_btn"),
        width=60,
        **kwargs
    )
    buttons4code = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col4_btn"),
        width=60,
        **kwargs
    )
    buttons5code = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col5_btn"),
        width=60,
        **kwargs
    )
    buttons6code = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col6_btn"),
        width=60,
        **kwargs
    )
    buttonsestpfgcode = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col_estp_fg"),
        width=60,
        **kwargs
    )
    buttonsestpbgcode = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col_estp_bg"),
        width=60,
        **kwargs
    )
    buttonssccode = ctk.CTkLabel(
        master=buttonsframe,
        text=getHex("Col_btn_sc"),
        width=60,
        **kwargs
    )
    buttonsdimmer = ctk.CTkSlider(
        master=buttonsframe,
        from_=0,
        to=100,
        number_of_steps=10,
        width=308
    )
    buttonsdimmer.set(globalsettings["Dimming"])
    buttonsfgbutton = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col_btn_fg"),
        width=70,
        corner_radius=radius
    )
    buttons1button = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col1_btn"),
        width=70,
        corner_radius=radius
    )
    buttons2button = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col2_btn"),
        width=70,
        corner_radius=radius
    )
    buttons3button = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col3_btn"),
        width=70,
        corner_radius=radius
    )
    buttons4button = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col4_btn"),
        width=70,
        corner_radius=radius
    )
    buttons5button = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col5_btn"),
        width=70,
        corner_radius=radius
    )
    buttons6button = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col6_btn"),
        width=70,
        corner_radius=radius
    )
    buttonsestpfgbutton = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col_estp_fg"),
        width=70,
        corner_radius=radius
    )
    buttonsestpbgbutton = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col_estp_bg"),
        width=70,
        corner_radius=radius
    )
    buttonsscbutton = ctk.CTkButton(
        master=buttonsframe,
        text="Change",
        command=lambda: selectColour("Col_btn_sc"),
        width=70,
        corner_radius=radius
    )

    # create colourblind widgets
    colourblindlabel = ctk.CTkLabel(
        master=colourblindframe,
        text="Colourblind mode",
        width=120,
        **kwargs
    )
    colourblindselector = ctk.CTkSlider(
        master=colourblindframe,
        from_=1,
        to=4,
        number_of_steps=3,
        width=308
    )
    colourblindselector.set(globalsettings["Colour_blind"])
    colourblindimg = ImageTk.PhotoImage(
        Image.open(f"img/img colourblind {ctk.get_appearance_mode().lower()}.png")
    )
    colourblindimage = ctk.CTkLabel(
        master=colourblindframe,
        image=colourblindimg,
        height=60,
        **kwargs
    )
    colourblindselectbutton = ctk.CTkButton(
        master=colourblindframe,
        text="Load and Save",
        command=lambda: loadSettings(
            parent="colourswindow",
            tell=True,
            save=True,
            colourblind=colourblindselector.get(),
        ),
        width=60,
        corner_radius=radius
    )

    # create themes widgets
    themelabel = ctk.CTkLabel(
        master=themesframe,
        text="Themes",
        width=50,
        **kwargs
    )
    themeselector = ctk.CTkSlider(
        master=themesframe,
        from_=1,
        to=3,
        number_of_steps=2,
        width=308
    )
    themeselector.set(globalsettings["Theme"])
    themeimg = ImageTk.PhotoImage(
        Image.open(f"img/img theme {ctk.get_appearance_mode().lower()}.png")
    )
    themeimage = ctk.CTkLabel(
        master=themesframe,
        image=themeimg,
        height=46,
        **kwargs
    )
    themeselectbutton = ctk.CTkButton(
        master=themesframe,
        text="Load Theme and Save",
        command=lambda: loadSettings(
            parent="colourswindow", tell=True, save=True, theme=themeselector.get()
        ),
        width=60,
        corner_radius=radius
    )

    # create actions widgets
    actionssave = ctk.CTkButton(
        master=actionsframe,
        text="Save",
        command=lambda: saveSettings(parent="colourswindow", tell=True),
        width=60,
        corner_radius=radius
    )
    actionsload = ctk.CTkButton(
        master=actionsframe,
        text="Load Saved",
        command=lambda: loadSettings(parent="colourswindow", tell=True),
        width=60,
        corner_radius=radius
    )
    actionsrefresh = ctk.CTkButton(
        master=actionsframe,
        text="Refresh",
        command=updateSample,
        width=60,
        corner_radius=radius
    )

    # create sample widgets
    samplexaxs = ctk.CTkLabel(
        master=sampleframe,
        text="X-axis",
        fg_color=getHex("Colx_axs"),
        text_color=oppositeColour(getHex("Colx_axs")),
        corner_radius=radius
    )
    sampleyaxs = ctk.CTkLabel(
        master=sampleframe,
        text="Y-axis",
        fg_color=getHex("Coly_axs"),
        text_color=oppositeColour(getHex("Coly_axs")),
        corner_radius=radius
    )
    samplezaxs = ctk.CTkLabel(
        master=sampleframe,
        text="Z-axis",
        fg_color=getHex("Colz_axs"),
        text_color=oppositeColour(getHex("Colz_axs")),
        corner_radius=radius
    )
    sample1plt = ctk.CTkLabel(
        master=sampleframe,
        text="Preview 1",
        fg_color=getHex("Col1_plt"),
        text_color=oppositeColour(getHex("Col1_plt")),
        corner_radius=radius
    )
    sample2plt = ctk.CTkLabel(
        master=sampleframe,
        text="Preview 2",
        fg_color=getHex("Col2_plt"),
        text_color=oppositeColour(getHex("Col2_plt")),
        corner_radius=radius
    )
    sample3plt = ctk.CTkLabel(
        master=sampleframe,
        text="Preview 3",
        fg_color=getHex("Col3_plt"),
        text_color=oppositeColour(getHex("Col3_plt")),
        corner_radius=radius
    )
    sample4plt = ctk.CTkLabel(
        master=sampleframe,
        text="Preview 4",
        fg_color=getHex("Col4_plt"),
        text_color=oppositeColour(getHex("Col4_plt")),
        corner_radius=radius
    )
    sample5plt = ctk.CTkLabel(
        master=sampleframe,
        text="Preview 5",
        fg_color=getHex("Col5_plt"),
        text_color=oppositeColour(getHex("Col5_plt")),
        corner_radius=radius
    )
    sample6plt = ctk.CTkLabel(
        master=sampleframe,
        text="Preview 6",
        fg_color=getHex("Col6_plt"),
        text_color=oppositeColour(getHex("Col6_plt")),
        corner_radius=radius
    )
    samplefgbtn = ctk.CTkLabel(
        master=sampleframe,
        text="Button fg",
        fg_color=getHex("Col_btn_fg"),
        text_color=oppositeColour(getHex("Col_btn_fg")),
        corner_radius=radius
    )
    sample1btnl = ctk.CTkLabel(
        master=sampleframe,
        text="Button 1 light",
        fg_color=getHex("Col1_btn"),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample1btnd = ctk.CTkLabel(
        master=sampleframe,
        text="Button 1 dark",
        fg_color=getHex("Col1_btn", dim=True),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample2btnl = ctk.CTkLabel(
        master=sampleframe,
        text="Button 2 light",
        fg_color=getHex("Col2_btn"),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample2btnd = ctk.CTkLabel(
        master=sampleframe,
        text="Button 2 dark",
        fg_color=getHex("Col2_btn", dim=True),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample3btnl = ctk.CTkLabel(
        master=sampleframe,
        text="Button 3 light",
        fg_color=getHex("Col3_btn"),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample3btnd = ctk.CTkLabel(
        master=sampleframe,
        text="Button 3 dark",
        fg_color=getHex("Col3_btn", dim=True),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample4btnl = ctk.CTkLabel(
        master=sampleframe,
        text="Button 4 light",
        fg_color=getHex("Col4_btn"),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample4btnd = ctk.CTkLabel(
        master=sampleframe,
        text="Button 4 dark",
        fg_color=getHex("Col4_btn", dim=True),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample5btnl = ctk.CTkLabel(
        master=sampleframe,
        text="Button 5 light",
        fg_color=getHex("Col5_btn"),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample5btnd = ctk.CTkLabel(
        master=sampleframe,
        text="Button 5 dark",
        fg_color=getHex("Col5_btn", dim=True),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample6btnl = ctk.CTkLabel(
        master=sampleframe,
        text="Button 6 light",
        fg_color=getHex("Col6_btn"),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sample6btnd = ctk.CTkLabel(
        master=sampleframe,
        text="Button 6 dark",
        fg_color=getHex("Col6_btn", dim=True),
        text_color=getHex("Col_btn_fg"),
        corner_radius=radius
    )
    sampleestpfg = ctk.CTkLabel(
        master=sampleframe,
        text="E-Stop fg",
        fg_color=getHex("Col_estp_fg"),
        text_color=oppositeColour(getHex("Col_estp_fg")),
        corner_radius=radius
    )
    sampleestpbg = ctk.CTkLabel(
        master=sampleframe,
        text="E-Stop bg",
        fg_color=getHex("Col_estp_bg"),
        text_color=oppositeColour(getHex("Col_estp_bg")),
        corner_radius=radius
    )
    samplesc = ctk.CTkLabel(
        master=sampleframe,
        text="Secondary",
        fg_color=getHex("Col_btn_sc"),
        text_color=oppositeColour(getHex("Col_btn_sc")),
        corner_radius=radius
    )
    sampleschover = ctk.CTkLabel(
        master=sampleframe,
        text="Secondary hover",
        fg_color=getHex("Col_btn_sc", light=True),
        text_color=oppositeColour(getHex("Col_btn_sc")),
        corner_radius=radius
    )

    # configure colours grid system
    colourswindow.grid_columnconfigure(0, weight=0)
    colourswindow.grid_columnconfigure(1, weight=0)
    colourswindow.grid_columnconfigure(2, weight=0)
    colourswindow.grid_rowconfigure(0, weight=0)
    colourswindow.grid_rowconfigure(1, weight=0)
    colourswindow.grid_rowconfigure(2, weight=0)
    colourswindow.grid_rowconfigure(3, weight=1)

    # configure preview frame grid system
    previewframe.grid_columnconfigure(0, weight=0)
    previewframe.grid_columnconfigure(1, weight=1)
    previewframe.grid_columnconfigure(2, weight=0)
    previewframe.grid_columnconfigure(3, weight=0)

    # configure buttons frame grid system
    buttonsframe.grid_columnconfigure(0, weight=0)
    buttonsframe.grid_columnconfigure(1, weight=1)
    buttonsframe.grid_columnconfigure(2, weight=0)
    buttonsframe.grid_columnconfigure(3, weight=0)

    # configure colourblind frame grid system
    colourblindframe.grid_columnconfigure(0, weight=0)
    colourblindframe.grid_columnconfigure(1, weight=1)

    # configure themes frame system
    themesframe.grid_columnconfigure(0, weight=0)
    themesframe.grid_columnconfigure(1, weight=1)

    # configure actions frame grid system
    actionsframe.grid_columnconfigure(0, weight=1)
    actionsframe.grid_columnconfigure(1, weight=1)
    actionsframe.grid_columnconfigure(2, weight=1)
    actionsframe.grid_rowconfigure(0, weight=1)

    # configure sample frame grid system

    sampleframe.grid_columnconfigure(0, weight=1)
    sampleframe.grid_columnconfigure(1, weight=1)
    sampleframe.grid_rowconfigure(0, weight=1)
    sampleframe.grid_rowconfigure(1, weight=1)
    sampleframe.grid_rowconfigure(2, weight=1)
    sampleframe.grid_rowconfigure(3, weight=1)
    sampleframe.grid_rowconfigure(4, weight=1)
    sampleframe.grid_rowconfigure(5, weight=1)
    sampleframe.grid_rowconfigure(6, weight=1)
    sampleframe.grid_rowconfigure(7, weight=1)
    sampleframe.grid_rowconfigure(8, weight=1)
    sampleframe.grid_rowconfigure(9, weight=1)
    sampleframe.grid_rowconfigure(10, weight=1)
    sampleframe.grid_rowconfigure(11, weight=1)
    sampleframe.grid_rowconfigure(12, weight=1)

    # pack colours frames
    previewframe.grid(
        row=0,
        column=0,
        **widgetpack
    )
    buttonsframe.grid(
        row=0,
        column=1,
        rowspan=2,
        **widgetpack
    )
    colourblindframe.grid(
        row=1,
        column=0,
        rowspan=2,
        **widgetpack
    )
    themesframe.grid(
        row=2,
        column=1,
        rowspan=2,
        **widgetpack
    )
    actionsframe.grid(
        row=3,
        column=0,
        **widgetpack
    )
    sampleframe.grid(
        row=0,
        column=2,
        rowspan=4,
        **widgetpack
    )

    # pack preview widgets
    previewlabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    previewxlabel.grid(
        row=1,
        column=0,
        columnspan=2,
        **widgetpack
    )
    previewylabel.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    previewzlabel.grid(
        row=3,
        column=0,
        columnspan=2,
        **widgetpack
    )
    preview1label.grid(
        row=4,
        column=0,
        columnspan=2,
        **widgetpack
    )
    preview2label.grid(
        row=5,
        column=0,
        columnspan=2,
        **widgetpack
    )
    preview3label.grid(
        row=6,
        column=0,
        columnspan=2,
        **widgetpack
    )
    preview4label.grid(
        row=7,
        column=0,
        columnspan=2,
        **widgetpack
    )
    preview5label.grid(
        row=8,
        column=0,
        columnspan=2,
        **widgetpack
    )
    preview6label.grid(
        row=9,
        column=0,
        columnspan=2,
        **widgetpack
    )
    previewxcode.grid(
        row=1,
        column=2,
        **widgetpack
    )
    previewycode.grid(
        row=2,
        column=2,
        **widgetpack
    )
    previewzcode.grid(
        row=3,
        column=2,
        **widgetpack
    )
    preview1code.grid(
        row=4,
        column=2,
        **widgetpack
    )
    preview2code.grid(
        row=5,
        column=2,
        **widgetpack
    )
    preview3code.grid(
        row=6,
        column=2,
        **widgetpack
    )
    preview4code.grid(
        row=7,
        column=2,
        **widgetpack
    )
    preview5code.grid(
        row=8,
        column=2,
        **widgetpack
    )
    preview6code.grid(
        row=9,
        column=2,
        **widgetpack
    )
    previewxbutton.grid(
        row=1,
        column=3,
        **widgetpack
    )
    previewybutton.grid(
        row=2,
        column=3,
        **widgetpack
    )
    previewzbutton.grid(
        row=3,
        column=3,
        **widgetpack
    )
    preview1button.grid(
        row=4,
        column=3,
        **widgetpack
    )
    preview2button.grid(
        row=5,
        column=3,
        **widgetpack
    )
    preview3button.grid(
        row=6,
        column=3,
        **widgetpack
    )
    preview4button.grid(
        row=7,
        column=3,
        **widgetpack
    )
    preview5button.grid(
        row=8,
        column=3,
        **widgetpack
    )
    preview6button.grid(
        row=9,
        column=3,
        **widgetpack
    )

    # pack buttons widgets
    buttonslabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    buttonsfglabel.grid(
        row=1,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttons1label.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttons2label.grid(
        row=3,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttons3label.grid(
        row=4,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttons4label.grid(
        row=5,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttons5label.grid(
        row=6,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttons6label.grid(
        row=7,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttonsestpfglabel.grid(
        row=8,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttonsestpbglabel.grid(
        row=9,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttonssclabel.grid(
        row=10,
        column=0,
        columnspan=2,
        **widgetpack
    )
    buttonsfgcode.grid(
        row=1,
        column=2,
        **widgetpack
    )
    buttons1code.grid(
        row=2,
        column=2,
        **widgetpack
    )
    buttons2code.grid(
        row=3,
        column=2,
        **widgetpack
    )
    buttons3code.grid(
        row=4,
        column=2,
        **widgetpack
    )
    buttons4code.grid(
        row=5,
        column=2,
        **widgetpack
    )
    buttons5code.grid(
        row=6,
        column=2,
        **widgetpack
    )
    buttons6code.grid(
        row=7,
        column=2,
        **widgetpack
    )
    buttonsestpfgcode.grid(
        row=8,
        column=2,
        **widgetpack
    )
    buttonsestpbgcode.grid(
        row=9,
        column=2,
        **widgetpack
    )
    buttonssccode.grid(
        row=10,
        column=2,
        **widgetpack
    )
    buttonsfgbutton.grid(
        row=1,
        column=3,
        **widgetpack
    )
    buttons1button.grid(
        row=2,
        column=3,
        **widgetpack
    )
    buttons2button.grid(
        row=3,
        column=3,
        **widgetpack
    )
    buttons3button.grid(
        row=4,
        column=3,
        **widgetpack
    )
    buttons4button.grid(
        row=5,
        column=3,
        **widgetpack
    )
    buttons5button.grid(
        row=6,
        column=3,
        **widgetpack
    )
    buttons6button.grid(
        row=7,
        column=3,
        **widgetpack
    )
    buttonsestpfgbutton.grid(
        row=8,
        column=3,
        **widgetpack
    )
    buttonsestpbgbutton.grid(
        row=9,
        column=3,
        **widgetpack
    )
    buttonsscbutton.grid(
        row=10,
        column=3,
        **widgetpack
    )
    buttonsdimmerlabel.grid(
        row=11,
        column=0,
        padx=8,
        pady=(8, 0),
        columnspan=4,
        sticky=N + S + E + W
    )
    buttonsdimmer.grid(
        row=12,
        column=0,
        padx=8,
        pady=(0, 8),
        columnspan=4,
        sticky=N + S + E + W
    )

    # pack colourblind widgets
    colourblindlabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    colourblindselector.grid(
        row=1,
        column=0,
        padx=8,
        pady=(8, 0),
        columnspan=2,
        sticky=N + S + E + W
    )
    colourblindimage.grid(
        row=2,
        column=0,
        padx=8,
        pady=(0, 8),
        columnspan=2,
        sticky=N + S + E + W
    )
    colourblindselectbutton.grid(
        row=3,
        column=0,
        columnspan=2,
        **widgetpack
    )

    # pack theme widgets
    themelabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    themeselector.grid(
        row=1,
        column=0,
        padx=8,
        pady=(8, 0),
        columnspan=2,
        sticky=N + S + E + W
    )
    themeimage.grid(
        row=2,
        column=0,
        padx=8,
        pady=(0, 8),
        columnspan=2,
        sticky=N + S + E + W
    )
    themeselectbutton.grid(
        row=3,
        column=0,
        columnspan=2,
        **widgetpack
    )

    # pack actions widgets
    actionssave.grid(
        row=0,
        column=0,
        **widgetpack
    )
    actionsload.grid(
        row=0,
        column=1,
        **widgetpack
    )
    actionsrefresh.grid(
        row=0,
        column=2,
        **widgetpack
    )

    # pack sample widgets
    samplexaxs.grid(
        row=0,
        column=0,
        **widgetpack
    )
    sampleyaxs.grid(
        row=0,
        column=1,
        **widgetpack
    )
    samplezaxs.grid(
        row=1,
        column=0,
        **widgetpack
    )
    samplefgbtn.grid(
        row=1,
        column=1,
        **widgetpack
    )
    sample1plt.grid(
        row=2,
        column=0,
        **widgetpack
    )
    sample2plt.grid(
        row=2,
        column=1,
        **widgetpack
    )
    sample3plt.grid(
        row=3,
        column=0,
        **widgetpack
    )
    sample4plt.grid(
        row=3,
        column=1,
        **widgetpack
    )
    sample5plt.grid(
        row=4,
        column=0,
        **widgetpack
    )
    sample6plt.grid(
        row=4,
        column=1,
        **widgetpack
    )
    sample1btnl.grid(
        row=5,
        column=0,
        **widgetpack
    )
    sample1btnd.grid(
        row=5,
        column=1,
        **widgetpack
    )
    sample2btnl.grid(
        row=6,
        column=0,
        **widgetpack
    )
    sample2btnd.grid(
        row=6,
        column=1,
        **widgetpack
    )
    sample3btnl.grid(
        row=7,
        column=0,
        **widgetpack
    )
    sample3btnd.grid(
        row=7,
        column=1,
        **widgetpack
    )
    sample4btnl.grid(
        row=8,
        column=0,
        **widgetpack
    )
    sample4btnd.grid(
        row=8,
        column=1,
        **widgetpack
    )
    sample5btnl.grid(
        row=9,
        column=0,
        **widgetpack
    )
    sample5btnd.grid(
        row=9,
        column=1,
        **widgetpack
    )
    sample6btnl.grid(
        row=10,
        column=0,
        **widgetpack
    )
    sample6btnd.grid(
        row=10,
        column=1,
        **widgetpack
    )
    sampleestpfg.grid(
        row=11,
        column=0,
        **widgetpack
    )
    sampleestpbg.grid(
        row=11,
        column=1,
        **widgetpack
    )
    samplesc.grid(
        row=12,
        column=0,
        **widgetpack
    )
    sampleschover.grid(
        row=12,
        column=1,
        **widgetpack
    )

    # finish
    colourswindow.focus_set()


def updateSample():
    # set the dimming factor
    globalsettings["Dimming"] = buttonsdimmer.get()

    # update the hex colour codes
    previewxcode.config(text=getHex("Colx_axs"))
    previewycode.config(text=getHex("Coly_axs"))
    previewzcode.config(text=getHex("Colz_axs"))
    preview1code.config(text=getHex("Col1_plt"))
    preview2code.config(text=getHex("Col2_plt"))
    preview3code.config(text=getHex("Col3_plt"))
    preview4code.config(text=getHex("Col4_plt"))
    preview5code.config(text=getHex("Col5_plt"))
    preview6code.config(text=getHex("Col6_plt"))
    buttonsfgcode.config(text=getHex("Col_btn_fg"))
    buttons1code.config(text=getHex("Col1_btn"))
    buttons2code.config(text=getHex("Col2_btn"))
    buttons3code.config(text=getHex("Col3_btn"))
    buttons4code.config(text=getHex("Col4_btn"))
    buttons5code.config(text=getHex("Col5_btn"))
    buttons6code.config(text=getHex("Col6_btn"))
    buttonsestpfgcode.config(text=getHex("Col_estp_fg"))
    buttonsestpbgcode.config(text=getHex("Col_estp_bg"))
    buttonssccode.config(text=getHex("Col_btn_sc"))

    # update the samples
    samplexaxs.config(
        fg_color=getHex("Colx_axs"),
        text_color=oppositeColour(getHex("Colx_axs"))
    )
    sampleyaxs.config(
        fg_color=getHex("Coly_axs"),
        text_color=oppositeColour(getHex("Coly_axs"))
    )
    samplezaxs.config(
        fg_color=getHex("Colz_axs"),
        text_color=oppositeColour(getHex("Colz_axs"))
    )
    sample1plt.config(
        fg_color=getHex("Col1_plt"),
        text_color=oppositeColour(getHex("Col1_plt"))
    )
    sample2plt.config(
        fg_color=getHex("Col2_plt"),
        text_color=oppositeColour(getHex("Col2_plt"))
    )
    sample3plt.config(
        fg_color=getHex("Col3_plt"),
        text_color=oppositeColour(getHex("Col3_plt"))
    )
    sample4plt.config(
        fg_color=getHex("Col4_plt"),
        text_color=oppositeColour(getHex("Col4_plt"))
    )
    sample5plt.config(
        fg_color=getHex("Col5_plt"),
        text_color=oppositeColour(getHex("Col5_plt"))
    )
    sample6plt.config(
        fg_color=getHex("Col6_plt"),
        text_color=oppositeColour(getHex("Col6_plt"))
    )
    samplefgbtn.config(
        fg_color=getHex("Col_btn_fg"),
        text_color=oppositeColour(getHex("Col_btn_fg"))
    )
    sample1btnl.config(
        fg_color=getHex("Col1_btn"),
        text_color=getHex("Col_btn_fg")
    )
    sample1btnd.config(
        fg_color=getHex("Col1_btn", dim=True),
        text_color=getHex("Col_btn_fg")
    )
    sample2btnl.config(
        fg_color=getHex("Col2_btn"),
        text_color=getHex("Col_btn_fg")
    )
    sample2btnd.config(
        fg_color=getHex("Col2_btn", dim=True),
        text_color=getHex("Col_btn_fg")
    )
    sample3btnl.config(
        fg_color=getHex("Col3_btn"),
        text_color=getHex("Col_btn_fg")
    )
    sample3btnd.config(
        fg_color=getHex("Col3_btn", dim=True),
        text_color=getHex("Col_btn_fg")
    )
    sample4btnl.config(
        fg_color=getHex("Col4_btn"),
        text_color=getHex("Col_btn_fg")
    )
    sample4btnd.config(
        fg_color=getHex("Col4_btn", dim=True),
        text_color=getHex("Col_btn_fg")
    )
    sample5btnl.config(
        fg_color=getHex("Col5_btn"),
        text_color=getHex("Col_btn_fg")
    )
    sample5btnd.config(
        fg_color=getHex("Col5_btn", dim=True),
        text_color=getHex("Col_btn_fg")
    )
    sample6btnl.config(
        fg_color=getHex("Col6_btn"),
        text_color=getHex("Col_btn_fg")
    )
    sample6btnd.config(
        fg_color=getHex("Col6_btn", dim=True),
        text_color=getHex("Col_btn_fg")
    )
    sampleestpfg.config(
        fg_color=getHex("Col_estp_fg"),
        text_color=oppositeColour(getHex("Col_estp_fg"))
    )
    sampleestpbg.config(
        fg_color=getHex("Col_estp_bg"),
        text_color=oppositeColour(getHex("Col_estp_bg"))
    )
    samplesc.config(
        fg_color=getHex("Col_btn_sc"),
        text_color=oppositeColour(getHex("Col_btn_sc"))
    )
    sampleschover.config(
        fg_color=getHex("Col_btn_sc", light=True),
        text_color=oppositeColour(getHex("Col_btn_sc"))
    )


# get the hex value of a widget from settings
def getHex(setting, dim=False, light=False):
    rgb = (
        globalsettings[f"{setting}_r"],
        globalsettings[f"{setting}_g"],
        globalsettings[f"{setting}_b"],
    )
    r, g, b = map(lambda x: x / 255.0, rgb)
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    if dim:
        v = v * globalsettings["Dimming"] / 100
    if light:
        s = s * 0.5
    newrgb = list(map(lambda x: round(x * 255), colorsys.hsv_to_rgb(h, s, v)))
    return f"#{hex(newrgb[0])[2:]:02}{hex(newrgb[1])[2:]:02}{hex(newrgb[2])[2:]:02}"


# get the RGBA value of a widget from settings
def getRGBA(setting):
    red = globalsettings[f"{setting}_r"] / 255
    green = globalsettings[f"{setting}_g"] / 255
    blue = globalsettings[f"{setting}_b"] / 255
    return (red, green, blue, 1.0)


# let the user select a colour
def selectColour(setting):
    colour = colorchooser.askcolor(
        color=getHex(setting),
        parent=colourswindow
    )[1]
    if colour:
        red = int(f"0x{colour[1:3]}", 0)
        green = int(f"0x{colour[3:5]}", 0)
        blue = int(f"0x{colour[5:7]}", 0)
        globalsettings[f"{setting}_r"] = red
        globalsettings[f"{setting}_g"] = green
        globalsettings[f"{setting}_b"] = blue
    if bool(globalsettings["Update"]):
        updateSample()
    colourswindow.focus_set()


# return the opposite colour
def oppositeColour(hexval):
    rgb = (
        int(f"0x{hexval[1:3]}", 0),
        int(f"0x{hexval[3:5]}", 0),
        int(f"0x{hexval[5:7]}", 0),
    )
    r, g, b = map(lambda x: x / 255.0, rgb)
    h, l, s = colorsys.rgb_to_hls(r, g, b)
    opph = h + 0.5
    oppl = 1 - l
    s = s
    opprgb = list(map(lambda x: round(x * 255), colorsys.hls_to_rgb(opph, oppl, s)))
    return f"#{hex(opprgb[0])[2:]:02}{hex(opprgb[1])[2:]:02}{hex(opprgb[2])[2:]:02}"


# update log box with new information
def updateLog(message=""):
    # define log buffer size
    length = 18

    if "log" not in globals():
        # create dummy data on startup
        global log
        log = []
        for i in range(length):
            log.append("(dd/mm/yy @ hh:mm:ss):")
    else:
        # queue new message
        if log[-1][23:] != message:
            currenttime = datetime.now().strftime("(%d/%m/%y @ %H:%M:%S): ")
            log.append(f"{currenttime}{message}")
            log.pop(0)

    # print queue to readout
    text = ""
    for i in range(length):
        text += f"{log[i]}" + ("\n" if i < (length - 1) else "")
    logtext.config(text=text, width=576)


# get inverse translation step size
def getInverseStepSizeT():
    try:
        step = float(inverseconfigstepsizet.get())
        return step
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Configure Tx, Ty and Tz' must be float"
        )


# get inverse rotation step size
def getInverseStepSizeR():
    try:
        step = float(inverseconfigstepsizer.get())
        return step
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Configure Rx, Ry and Rz' must be float"
        )


# get forward rotation step size
def getForwardStepSize():
    try:
        step = float(forwardconfigstepsize.get())
        return step
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Configure A1, A2, A3, A4, A5 and A6' must be float"
        )


# change x translation by current step
def updateTx(scalar):
    try:
        current = float(tx.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'X Translation' must be float"
        )
    step = getInverseStepSizeT()
    try:
        new = float(current + (step * scalar))
        tx.delete(0, END)
        tx.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change y translation by current step
def updateTy(scalar):
    try:
        current = float(ty.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Y Translation' must be float"
        )
    step = getInverseStepSizeT()
    try:
        new = float(current + (step * scalar))
        ty.delete(0, END)
        ty.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change z translation by current step
def updateTz(scalar):
    try:
        current = float(tz.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Z Translation' must be float"
        )
    step = getInverseStepSizeT()
    try:
        new = float(current + (step * scalar))
        tz.delete(0, END)
        tz.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change x rotation by current step
def updateRx(scalar):
    try:
        current = float(rx.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'X Rotation' must be float"
        )
    step = getInverseStepSizeR()
    try:
        new = float(current + (step * scalar))
        rx.delete(0, END)
        rx.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change y rotation by current step
def updateRy(scalar):
    try:
        current = float(ry.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Y Rotation' must be float"
        )
    step = getInverseStepSizeR()
    try:
        new = float(current + (step * scalar))
        ry.delete(0, END)
        ry.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change z rotation by current step
def updateRz(scalar):
    try:
        current = float(rz.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Z Rotation' must be float"
        )
    step = getInverseStepSizeR()
    try:
        new = float(current + (step * scalar))
        rz.delete(0, END)
        rz.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change axis 1 angle by current step
def updateA1(scalar):
    try:
        current = float(a1.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Axis 1' must be float"
        )
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a1.delete(0, END)
        a1.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change axis 2 angle by current step
def updateA2(scalar):
    try:
        current = float(a2.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Axis 2' must be float"
        )
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a2.delete(0, END)
        a2.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change axis 3 angle by current step
def updateA3(scalar):
    try:
        current = float(a3.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Axis 3' must be float"
        )
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a3.delete(0, END)
        a3.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change axis 4 angle by current step
def updateA4(scalar):
    try:
        current = float(a4.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Axis 4' must be float"
        )
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a4.delete(0, END)
        a4.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change axis 5 angle by current step
def updateA5(scalar):
    try:
        current = float(a5.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Axis 5' must be float"
        )
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a5.delete(0, END)
        a5.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# change axis 6 angle by current step
def updateA6(scalar):
    try:
        current = float(a6.get())
    except:
        messagebox.showerror(
            title="An error occurred",
            message="'Axis 6' must be float"
        )
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a6.delete(0, END)
        a6.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings["Update"]):
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()


# configure window based on settings
def updateWindow():
    if bool(globalsettings["Automate"]):
        directposbutton.configure(state=DISABLED)
    else:
        directposbutton.configure(state=NORMAL)
    if bool(globalsettings["Update"]):
        updateplotbutton.configure(state=DISABLED)
        if globalsettings["Ghost"] == 1.0:
            plotData(True)
        plotData()
    else:
        updateplotbutton.configure(state=NORMAL)


# open the root window
def openMainWindow():
    global root
    global canvas
    global ax
    global inverseconfigstepsizet
    global inverseconfigstepsizer
    global forwardconfigstepsize
    global tx
    global ty
    global tz
    global rx
    global ry
    global rz
    global a1
    global a2
    global a3
    global a4
    global a5
    global a6
    global endeffectorslider
    global directposbutton
    global updateplotbutton
    global logtext
    global radius

    global directTravel
    global lerpTravel
    global eStop

    # only create placeholder function if user has not defined their own
    if "directTravel" not in locals():

        def directTravel(data):
            print("'directTravel' function call")
            print(data)

    # only create placeholder function is user has not defined their own
    if "lerpTravel" not in locals():

        def lerpTravel(path):
            print("'lerpTravel' function call")
            print(path)

    # only create placeholder function if user has not defined their own
    if "eStop" not in locals():

        def eStop():  # Emergency stop
            print("'eStop' function call")

    # load saved settings
    setSettings()

    # define aesthetic constants
    radius = globalsettings["Corner_radius"]
    fontcolour = getHex("Col_btn_fg")

    # set the theme
    ctk.set_appearance_mode(
        "Dark"
        if globalsettings["Theme"] == 3.0
        else "Light"
        if globalsettings["Theme"] == 2.0
        else "System"
    )
    ctk.CTkColorManager.MAIN = getHex("Col_btn_sc")
    ctk.CTkColorManager.MAIN_HOVER = getHex("Col_btn_sc", False, True)

    # create and configure window object
    root = ctk.CTk()
    root.title("InvKinGUI")
    root.wm_iconphoto(False, ImageTk.PhotoImage(Image.open("img/icon root.png")))
    root.geometry("+40+150")
    root.resizable(width=0, height=0)

    # create kwargs that will be used frequently
    kwargslight = {
        "corner_radius": radius,
        "fg_color": ("#bfbec1", "#505050"),
        "justify": LEFT
    }

    kwargsdark = {
        "corner_radius": radius,
        "fg_color": ("#d4d5d6", "#3f3f3f"),
        "justify": LEFT
    }

    widgetpack = {
        "padx": 8,
        "pady": 8,
        "sticky": N + S + E + W
    }

    # define graph frame object
    graphframe = ctk.CTkFrame(
        master=root,
        width=916,
        height=800,
        corner_radius=radius
    )

    # create inverse kinematics frames
    inverseframe = ctk.CTkFrame(
        master=root,
        width=912,
        height=234,
        corner_radius=radius
    )
    txinverseframe = ctk.CTkFrame(
        master=inverseframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    tyinverseframe = ctk.CTkFrame(
        master=inverseframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    tzinverseframe = ctk.CTkFrame(
        master=inverseframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    rxinverseframe = ctk.CTkFrame(
        master=inverseframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    ryinverseframe = ctk.CTkFrame(
        master=inverseframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    rzinverseframe = ctk.CTkFrame(
        master=inverseframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    inverseconfigtframe = ctk.CTkFrame(
        master=inverseframe,
        width=440,
        height=48,
        corner_radius=radius
    )
    inverseconfigrframe = ctk.CTkFrame(
        master=inverseframe,
        width=440,
        height=48,
        corner_radius=radius
    )

    # create inverse kinematics widgets
    inverseframelabel = ctk.CTkLabel(
        master=inverseframe,
        text="Inverse Kinematics",
        width=136,
        **kwargsdark
    )
    txinverseup = ctk.CTkButton(
        master=txinverseframe,
        text="Increment",
        fg_color=getHex("Col1_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col1_btn", False, True),
        command=lambda: updateTx(1),
        corner_radius=radius
    )
    tyinverseup = ctk.CTkButton(
        master=tyinverseframe,
        text="Increment",
        fg_color=getHex("Col2_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col2_btn", False, True),
        command=lambda: updateTy(1),
        corner_radius=radius
    )
    tzinverseup = ctk.CTkButton(
        master=tzinverseframe,
        text="Increment",
        fg_color=getHex("Col3_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col3_btn", False, True),
        command=lambda: updateTz(1),
        corner_radius=radius
    )
    rxinverseup = ctk.CTkButton(
        master=rxinverseframe,
        text="Increment",
        fg_color=getHex("Col4_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col4_btn", False, True),
        command=lambda: updateRx(1),
        corner_radius=radius
    )
    ryinverseup = ctk.CTkButton(
        master=ryinverseframe,
        text="Increment",
        fg_color=getHex("Col5_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col5_btn", False, True),
        command=lambda: updateRy(1),
        corner_radius=radius
    )
    rzinverseup = ctk.CTkButton(
        master=rzinverseframe,
        text="Increment",
        fg_color=getHex("Col6_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col6_btn", False, True),
        command=lambda: updateRz(1),
        corner_radius=radius
    )
    tx = ctk.CTkEntry(
        master=txinverseframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    ty = ctk.CTkEntry(
        master=tyinverseframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    tz = ctk.CTkEntry(
        master=tzinverseframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    rx = ctk.CTkEntry(
        master=rxinverseframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    ry = ctk.CTkEntry(
        master=ryinverseframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    rz = ctk.CTkEntry(
        master=rzinverseframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    txunit = ctk.CTkLabel(
        master=txinverseframe,
        text="mm",
        width=30,
        **kwargslight
    )
    tyunit = ctk.CTkLabel(
        master=tyinverseframe,
        text="mm",
        width=30,
        **kwargslight
    )
    tzunit = ctk.CTkLabel(
        master=tzinverseframe,
        text="mm",
        width=30,
        **kwargslight
    )
    rxunit = ctk.CTkLabel(
        master=rxinverseframe,
        text="°    ",
        width=30,
        **kwargslight
    )
    ryunit = ctk.CTkLabel(
        master=ryinverseframe,
        text="°    ",
        width=30,
        **kwargslight
    )
    rzunit = ctk.CTkLabel(
        master=rzinverseframe,
        text="°    ",
        width=30,
        **kwargslight
    )
    txinversedown = ctk.CTkButton(
        master=txinverseframe,
        text="Decrement",
        fg_color=getHex("Col1_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col1_btn", True, True),
        command=lambda: updateTx(-1),
        corner_radius=radius
    )
    tyinversedown = ctk.CTkButton(
        master=tyinverseframe,
        text="Decrement",
        fg_color=getHex("Col2_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col2_btn", True, True),
        command=lambda: updateTy(-1),
        corner_radius=radius
    )
    tzinversedown = ctk.CTkButton(
        master=tzinverseframe,
        text="Decrement",
        fg_color=getHex("Col3_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col3_btn", True, True),
        command=lambda: updateTz(-1),
        corner_radius=radius
    )
    rxinversedown = ctk.CTkButton(
        master=rxinverseframe,
        text="Decrement",
        fg_color=getHex("Col4_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col4_btn", True, True),
        command=lambda: updateRx(-1),
        corner_radius=radius
    )
    ryinversedown = ctk.CTkButton(
        master=ryinverseframe,
        text="Decrement",
        fg_color=getHex("Col5_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col5_btn", True, True),
        command=lambda: updateRy(-1),
        corner_radius=radius
    )
    rzinversedown = ctk.CTkButton(
        master=rzinverseframe,
        text="Decrement",
        fg_color=getHex("Col6_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col6_btn", True, True),
        command=lambda: updateRz(-1),
        corner_radius=radius
    )
    inverseconfigtlabel1 = ctk.CTkLabel(
        master=inverseconfigtframe,
        text="Increment / Decrement in",
        width=180,
        **kwargslight
    )
    inverseconfigstepsizet = ctk.CTkEntry(
        master=inverseconfigtframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    inverseconfigstepsizet.insert(END, globalsettings["Inv_t_step"])
    inverseconfigtlabel2 = ctk.CTkLabel(
        master=inverseconfigtframe,
        text="mm steps",
        width=70,
        **kwargslight
    )
    inverseconfigsavet = ctk.CTkButton(
        master=inverseconfigtframe,
        text="Save",
        fg_color=getHex("Col_btn_sc"),
        hover_color=getHex("Col_btn_sc", False, True),
        command=lambda: saveSettings(tell=True),
        width=10,
        corner_radius=radius
    )
    inverseconfigrlabel1 = ctk.CTkLabel(
        master=inverseconfigrframe,
        text="Increment / Decrement in",
        width=180,
        **kwargslight
    )
    inverseconfigstepsizer = ctk.CTkEntry(
        master=inverseconfigrframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    inverseconfigstepsizer.insert( END, globalsettings["Inv_r_step"])
    inverseconfigrlabel2 = ctk.CTkLabel(
        master=inverseconfigrframe,
        text="°  steps",
        width=55,
        **kwargslight
    )
    inverseconfigsaver = ctk.CTkButton(
        master=inverseconfigrframe,
        text="Save",
        fg_color=getHex("Col_btn_sc"),
        hover_color=getHex("Col_btn_sc", False, True),
        command=lambda: saveSettings(tell=True),
        width=10,
        corner_radius=radius
    )

    # assign inverse kinematics defaults
    tx.insert(0, 0.0)
    ty.insert(0, 0.0)
    tz.insert(0, 0.0)
    rx.insert(0, 0.0)
    ry.insert(0, 0.0)
    rz.insert(0, 0.0)

    # create forward kinematics frames
    forwardframe = ctk.CTkFrame(
        master=root,
        width=912,
        height=234,
        corner_radius=radius
    )
    forwardframe1 = ctk.CTkFrame(
        master=forwardframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    forwardframe2 = ctk.CTkFrame(
        master=forwardframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    forwardframe3 = ctk.CTkFrame(
        master=forwardframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    forwardframe4 = ctk.CTkFrame(
        master=forwardframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    forwardframe5 = ctk.CTkFrame(
        master=forwardframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    forwardframe6 = ctk.CTkFrame(
        master=forwardframe,
        width=136,
        height=122,
        corner_radius=radius
    )
    forwardconfigframe = ctk.CTkFrame(
        master=forwardframe,
        width=440,
        height=48,
        corner_radius=radius
    )

    # create forward kinematics widgets
    forwardframelabel = ctk.CTkLabel(
        master=forwardframe,
        text="Forward Kinematics",
        width=136,
        **kwargsdark
    )
    forwardup1 = ctk.CTkButton(
        master=forwardframe1,
        text="Increment",
        fg_color=getHex("Col1_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col1_btn", False, True),
        command=lambda: updateA1(1),
        corner_radius=radius
    )
    forwardup2 = ctk.CTkButton(
        master=forwardframe2,
        text="Increment",
        fg_color=getHex("Col2_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col2_btn", False, True),
        command=lambda: updateA2(1),
        corner_radius=radius
    )
    forwardup3 = ctk.CTkButton(
        master=forwardframe3,
        text="Increment",
        fg_color=getHex("Col3_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col3_btn", False, True),
        command=lambda: updateA3(1),
        corner_radius=radius
    )
    forwardup4 = ctk.CTkButton(
        master=forwardframe4,
        text="Increment",
        fg_color=getHex("Col4_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col4_btn", False, True),
        command=lambda: updateA4(1),
        corner_radius=radius
    )
    forwardup5 = ctk.CTkButton(
        master=forwardframe5,
        text="Increment",
        fg_color=getHex("Col5_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col5_btn", False, True),
        command=lambda: updateA5(1),
        corner_radius=radius
    )
    forwardup6 = ctk.CTkButton(
        master=forwardframe6,
        text="Increment",
        fg_color=getHex("Col6_btn"),
        text_color=fontcolour,
        hover_color=getHex("Col6_btn", False, True),
        command=lambda: updateA6(1),
        corner_radius=radius
    )
    a1 = ctk.CTkEntry(
        master=forwardframe1,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    a2 = ctk.CTkEntry(
        master=forwardframe2,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    a3 = ctk.CTkEntry(
        master=forwardframe3,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    a4 = ctk.CTkEntry(
        master=forwardframe4,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    a5 = ctk.CTkEntry(
        master=forwardframe5,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    a6 = ctk.CTkEntry(
        master=forwardframe6,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    a1unit = ctk.CTkLabel(
        master=forwardframe1,
        text="°    ",
        width=30,
        **kwargslight
    )
    a2unit = ctk.CTkLabel(
        master=forwardframe2,
        text="°    ",
        width=30,
        **kwargslight
    )
    a3unit = ctk.CTkLabel(
        master=forwardframe3,
        text="°    ",
        width=30,
        **kwargslight
    )
    a4unit = ctk.CTkLabel(
        master=forwardframe4,
        text="°    ",
        width=30,
        **kwargslight
    )
    a5unit = ctk.CTkLabel(
        master=forwardframe5,
        text="°    ",
        width=30,
        **kwargslight
    )
    a6unit = ctk.CTkLabel(
        master=forwardframe6,
        text="°    ",
        width=30,
        **kwargslight
    )
    forwarddown1 = ctk.CTkButton(
        master=forwardframe1,
        text="Decrement",
        fg_color=getHex("Col1_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col1_btn", True, True),
        command=lambda: updateA1(-1),
        corner_radius=radius
    )
    forwarddown2 = ctk.CTkButton(
        master=forwardframe2,
        text="Decrement",
        fg_color=getHex("Col2_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col2_btn", True, True),
        command=lambda: updateA2(-1),
        corner_radius=radius
    )
    forwarddown3 = ctk.CTkButton(
        master=forwardframe3,
        text="Decrement",
        fg_color=getHex("Col3_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col3_btn", True, True),
        command=lambda: updateA3(-1),
        corner_radius=radius
    )
    forwarddown4 = ctk.CTkButton(
        master=forwardframe4,
        text="Decrement",
        fg_color=getHex("Col4_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col4_btn", True, True),
        command=lambda: updateA4(-1),
        corner_radius=radius
    )
    forwarddown5 = ctk.CTkButton(
        master=forwardframe5,
        text="Decrement",
        fg_color=getHex("Col5_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col5_btn", True, True),
        command=lambda: updateA5(-1),
        corner_radius=radius
    )
    forwarddown6 = ctk.CTkButton(
        master=forwardframe6,
        text="Decrement",
        fg_color=getHex("Col6_btn", True),
        text_color=fontcolour,
        hover_color=getHex("Col6_btn", True, True),
        command=lambda: updateA6(-1),
        corner_radius=radius
    )
    forwardconfiglabel1 = ctk.CTkLabel(
        master=forwardconfigframe,
        text="Increment / Decrement in",
        width=180,
        **kwargslight
    )
    forwardconfigstepsize = ctk.CTkEntry(
        master=forwardconfigframe,
        justify=RIGHT,
        width=90,
        corner_radius=radius
    )
    forwardconfigstepsize.insert(END, globalsettings["Fwd_step"])
    forwardconfiglabel2 = ctk.CTkLabel(
        master=forwardconfigframe,
        text="°  steps",
        width=55,
        **kwargslight
    )
    forwardconfigsave = ctk.CTkButton(
        master=forwardconfigframe,
        text="Save",
        fg_color=getHex("Col_btn_sc"),
        hover_color=getHex("Col_btn_sc", False, True),
        command=lambda: saveSettings(tell=True),
        width=10,
        corner_radius=radius
    )

    # assign forwards kinematics defaults
    a1.insert(0, 0.0)
    a2.insert(0, 0.0)
    a3.insert(0, 0.0)
    a4.insert(0, 0.0)
    a5.insert(0, 0.0)
    a6.insert(0, 0.0)

    # create controls frame
    controlsframe = ctk.CTkFrame(
        master=root,
        width=304,
        height=340, 
        corner_radius=radius
    )
    travelframe = ctk.CTkFrame(
        master=controlsframe,
        width=100,
        height=125,
        corner_radius=radius
    )

    # create controls widgets
    endeffectorlabel = ctk.CTkLabel(
        master=controlsframe,
        text="End Effector",
        width=100,
        **kwargsdark
    )
    endeffectorslider = ctk.CTkSlider(
        master=controlsframe,
        from_=0,
        to=100,
        width=253
    )
    calculateinversebutton = ctk.CTkButton(
        master=controlsframe,
        text="Calculate Inverse Kinematics",
        command=calcInverseKinematics,
        width=288,
        corner_radius=radius
    )
    calculateforwardbutton = ctk.CTkButton(
        master=controlsframe,
        text="Calculate Forward Kinematics",
        command=calcForwardKinematics,
        width=288,
        corner_radius=radius
    )
    estopbutton = ctk.CTkButton(
        master=controlsframe,
        text="E - STOP",
        text_font="roboto 20 bold",
        fg_color=getHex("Col_estp_bg"),
        text_color=getHex("Col_estp_fg"),
        hover_color=getHex("Col_estp_bg", False, True),
        command=eStopFunc,
        width=120,
        height=125,
        corner_radius=radius
    )
    homebutton = ctk.CTkButton(
        master=controlsframe,
        text="Home",
        command=goHome,
        width=100,
        corner_radius=radius
    )
    travellabel = ctk.CTkLabel(
        master=travelframe,
        text='Travel',
        width=40,
        **kwargslight
    )
    directposbutton = ctk.CTkButton(
        master=travelframe,
        text="Direct",
        command=directTravelFunc,
        width=84,
        corner_radius=radius
    )
    lerpposbutton = ctk.CTkButton(
        master=travelframe,
        text='Lerp',
        command=lerpTravelFunc,
        width=84,
        corner_radius=radius
    )
    updateplotbutton = ctk.CTkButton(
        master=controlsframe,
        text="Update Preview",
        command=plotData,
        width=100,
        corner_radius=radius
    )
    opensettingsbutton = ctk.CTkButton(
        master=controlsframe,
        text="Settings",
        command=openSettingsWindow,
        width=100,
        corner_radius=radius
    )

    # configure log frame
    logframe = ctk.CTkFrame(
        master=root,
        width=510,
        height=320,
        corner_radius=radius
    )

    # configure log widget
    logtext = ctk.CTkLabel(
        master=logframe,
        width=494,
        height=304,
        **kwargsdark
    )

    # create graph object
    fig = Figure(
        figsize=(8.5, 8),
        dpi=100,
        facecolor="#3f3f3f" if ctk.get_appearance_mode() == "Dark" else "#d4d5d6"
    )
    fig.subplots_adjust(
        left=0.0,
        bottom=0.03,
        right=0.97,
        top=0.97
    )
    canvas = FigureCanvasTkAgg(fig, master=graphframe)

    # configure graph object
    ax = fig.add_subplot(111, projection="3d")
    ax.w_xaxis.line.set_color(getHex("Colx_axs"))
    ax.w_yaxis.line.set_color(getHex("Coly_axs"))
    ax.w_zaxis.line.set_color(getHex("Colz_axs"))
    ax.w_xaxis.set_pane_color(
        (0.24706, 0.24706, 0.24706, 1.0)
        if ctk.get_appearance_mode() == "Dark"
        else (0.83137, 0.83529, 0.84314, 1.0)
    )
    ax.w_yaxis.set_pane_color(
        (0.24706, 0.24706, 0.24706, 1.0)
        if ctk.get_appearance_mode() == "Dark"
        else (0.83137, 0.83529, 0.84314, 1.0)
    )
    ax.w_zaxis.set_pane_color(
        (0.24706, 0.24706, 0.24706, 1.0)
        if ctk.get_appearance_mode() == "Dark"
        else (0.83137, 0.83529, 0.84314, 1.0)
    )
    ax.xaxis._axinfo["grid"].update(
        {
            "color": (0.8, 0.8, 0.8, 1.0)
            if ctk.get_appearance_mode() == "Dark"
            else (0.5, 0.5, 0.5, 1.0)
        }
    )
    ax.yaxis._axinfo["grid"].update(
        {
            "color": (0.8, 0.8, 0.8, 1.0)
            if ctk.get_appearance_mode() == "Dark"
            else (0.5, 0.5, 0.5, 1.0)
        }
    )
    ax.zaxis._axinfo["grid"].update(
        {
            "color": (0.8, 0.8, 0.8, 1.0)
            if ctk.get_appearance_mode() == "Dark"
            else (0.5, 0.5, 0.5, 1.0)
        }
    )
    ax.view_init(elev=30, azim=-135)
    ax.tick_params(
        axis="x", colors="white" if ctk.get_appearance_mode() == "Dark" else "black"
    )
    ax.tick_params(
        axis="y", colors="white" if ctk.get_appearance_mode() == "Dark" else "black"
    )
    ax.tick_params(
        axis="z", colors="white" if ctk.get_appearance_mode() == "Dark" else "black"
    )
    ax.set_facecolor(
        (0.24706, 0.24706, 0.24706, 1.0)
        if ctk.get_appearance_mode() == "Dark"
        else (0.83137, 0.83529, 0.84314, 1.0)
    )

    # configure root grid system
    root.grid_rowconfigure(0, weight=0)
    root.grid_rowconfigure(1, weight=0)
    root.grid_rowconfigure(2, weight=1)
    root.grid_columnconfigure(0, weight=0)
    root.grid_columnconfigure(1, weight=0)
    root.grid_columnconfigure(2, weight=1)

    # configure inverse kinematics grid system
    inverseframe.grid_rowconfigure(0, weight=0)
    inverseframe.grid_rowconfigure(1, weight=0)
    inverseframe.grid_rowconfigure(2, weight=0)
    txinverseframe.grid_columnconfigure(0, weight=4)
    txinverseframe.grid_columnconfigure(1, weight=1)
    tyinverseframe.grid_columnconfigure(0, weight=4)
    tyinverseframe.grid_columnconfigure(1, weight=1)
    tzinverseframe.grid_columnconfigure(0, weight=4)
    tzinverseframe.grid_columnconfigure(1, weight=1)
    rxinverseframe.grid_columnconfigure(0, weight=4)
    rxinverseframe.grid_columnconfigure(1, weight=1)
    ryinverseframe.grid_columnconfigure(0, weight=4)
    ryinverseframe.grid_columnconfigure(1, weight=1)
    rzinverseframe.grid_columnconfigure(0, weight=4)
    rzinverseframe.grid_columnconfigure(1, weight=1)

    # configure forward kinematics grid system
    forwardframe.grid_rowconfigure(0, weight=3)
    forwardframe.grid_rowconfigure(1, weight=2)
    forwardframe1.grid_columnconfigure(0, weight=4)
    forwardframe1.grid_columnconfigure(1, weight=1)
    forwardframe2.grid_columnconfigure(0, weight=4)
    forwardframe2.grid_columnconfigure(1, weight=1)
    forwardframe3.grid_columnconfigure(0, weight=4)
    forwardframe3.grid_columnconfigure(1, weight=1)
    forwardframe4.grid_columnconfigure(0, weight=4)
    forwardframe4.grid_columnconfigure(1, weight=1)
    forwardframe5.grid_columnconfigure(0, weight=4)
    forwardframe5.grid_columnconfigure(1, weight=1)
    forwardframe6.grid_columnconfigure(0, weight=4)
    forwardframe6.grid_columnconfigure(1, weight=1)

    # configure controls grid system
    controlsframe.grid_columnconfigure(0, weight=0)
    controlsframe.grid_columnconfigure(1, weight=1)
    controlsframe.grid_columnconfigure(2, weight=0)
    controlsframe.grid_columnconfigure(3, weight=0)
    controlsframe.grid_rowconfigure(0, weight=0)
    controlsframe.grid_rowconfigure(1, weight=0)
    controlsframe.grid_rowconfigure(2, weight=0)
    controlsframe.grid_rowconfigure(3, weight=0)
    controlsframe.grid_rowconfigure(4, weight=0)
    controlsframe.grid_rowconfigure(5, weight=0)
    travelframe.grid_columnconfigure(0, weight=0)
    travelframe.grid_columnconfigure(1, weight=1)

    # configure log grid system
    logframe.grid_columnconfigure(0, weight=1)

    # pack graph frame
    graphframe.grid(
        row=0,
        column=0,
        rowspan=3,
        **widgetpack
    )

    # pack graph widget
    canvas.get_tk_widget().pack(side=TOP, padx=8, pady=8)

    # pack inverse kinematics frames
    inverseframe.grid(
        row=0,
        column=1,
        columnspan=2,
        **widgetpack
    )
    inverseframelabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    txinverseframe.grid(
        row=1,
        column=0,
        **widgetpack
    )
    tyinverseframe.grid(
        row=1,
        column=1,
        **widgetpack
    )
    tzinverseframe.grid(
        row=1,
        column=2,
        **widgetpack
    )
    rxinverseframe.grid(
        row=1,
        column=3,
        **widgetpack
    )
    ryinverseframe.grid(
        row=1,
        column=4,
        **widgetpack
    )
    rzinverseframe.grid(
        row=1,
        column=5,
        **widgetpack
    )
    inverseconfigtframe.grid(
        row=2,
        column=0,
        columnspan=3,
        **widgetpack
    )
    inverseconfigrframe.grid(
        row=2,
        column=3,
        columnspan=3,
        **widgetpack
    )

    # pack inverse kinematics widgets
    txinverseup.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    tyinverseup.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    tzinverseup.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    rxinverseup.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    ryinverseup.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    rzinverseup.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    tx.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    ty.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    tz.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    rx.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    ry.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    rz.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    txunit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    tyunit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    tzunit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    rxunit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    ryunit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    rzunit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    txinversedown.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    tyinversedown.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    tzinversedown.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    rxinversedown.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    ryinversedown.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    rzinversedown.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    inverseconfigtlabel1.pack(
        side=LEFT,
        padx=(8, 0)
    )
    inverseconfigstepsizet.pack(
        side=LEFT
    )
    inverseconfigtlabel2.pack(
        side=LEFT
    )
    inverseconfigsavet.pack(
        side=RIGHT,
        padx=8,
        pady=8
    )
    inverseconfigrlabel1.pack(
        side=LEFT,
        padx=(8, 0)
    )
    inverseconfigstepsizer.pack(
        side=LEFT
    )
    inverseconfigrlabel2.pack(
        side=LEFT
    )
    inverseconfigsaver.pack(
        side=RIGHT,
        padx=8,
        pady=8
    )

    # pack forward kinematics frames
    forwardframe.grid(
        row=1,
        column=1,
        columnspan=2,
        **widgetpack
    )
    forwardframelabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8, 0),
        sticky=N + S + E + W
    )
    forwardframe1.grid(
        row=1,
        column=0,
        **widgetpack
    )
    forwardframe2.grid(
        row=1,
        column=1,
        **widgetpack
    )
    forwardframe3.grid(
        row=1,
        column=2,
        **widgetpack
    )
    forwardframe4.grid(
        row=1,
        column=3,
        **widgetpack
    )
    forwardframe5.grid(
        row=1,
        column=4,
        **widgetpack
    )
    forwardframe6.grid(
        row=1,
        column=5,
        **widgetpack
    )
    forwardconfigframe.grid(
        row=2,
        column=0,
        **widgetpack, columnspan=3
    )

    # pack forward kinematics widgets
    forwardup1.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwardup2.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwardup3.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwardup4.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwardup5.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwardup6.grid(
        row=0,
        column=0,
        columnspan=2,
        **widgetpack
    )
    a1.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    a2.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    a3.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    a4.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    a5.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    a6.grid(
        row=1,
        column=0,
        padx=(8, 0),
        pady=0,
        sticky=N + S + E + W
    )
    a1unit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    a2unit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    a3unit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    a4unit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    a5unit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    a6unit.grid(
        row=1,
        column=1,
        padx=(0, 8),
        pady=0,
        sticky=N + S + E + W
    )
    forwarddown1.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwarddown2.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwarddown3.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwarddown4.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwarddown5.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwarddown6.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )
    forwardconfiglabel1.pack(
        side=LEFT,
        padx=(8, 0),
        pady=4
    )
    forwardconfigstepsize.pack(
        side=LEFT,
        padx=0,
        pady=4
    )
    forwardconfiglabel2.pack(
        side=LEFT,
        padx=(0, 8),
        pady=4
    )
    forwardconfigsave.pack(
        side=RIGHT,
        padx=8,
        pady=8
    )

    # pack controls frame
    controlsframe.grid(
        row=2,
        column=1,
        **widgetpack
    )
    travelframe.grid(
        row=1,
        column=3,
        **widgetpack
    )

    # pack controls widgets
    endeffectorlabel.grid(
        row=0,
        column=0,
        **widgetpack
    )
    endeffectorslider.grid(
        row=0,
        column=1,
        padx=8,
        pady=12,
        sticky=N + S + E + W,
        columnspan=3
    )
    calculateinversebutton.grid(
        row=2,
        column=0,
        columnspan=4,
        **widgetpack
    )
    calculateforwardbutton.grid(
        row=3,
        column=0,
        columnspan=4,
        **widgetpack
    )
    estopbutton.grid(
        row=1,
        column=0,
        columnspan=3,
        **widgetpack
    )
    homebutton.grid(
        row=4,
        column=2,
        **widgetpack
    )
    updateplotbutton.grid(
        row=4,
        column=0,
        columnspan=2,
        **widgetpack
    )
    opensettingsbutton.grid(
        row=4,
        column=3,
        **widgetpack
    )
    travellabel.grid(
        row=0,
        column=0,
        padx=8,
        pady=(8,0),
        sticky=N + S + E + W
    )
    directposbutton.grid(
        row=1,
        column=0,
        columnspan=2,
        **widgetpack
    )
    lerpposbutton.grid(
        row=2,
        column=0,
        columnspan=2,
        **widgetpack
    )

    # pack log frame
    logframe.grid(
        row=2,
        column=2,
        **widgetpack
    )

    # pack log widget
    logtext.grid(
        row=0,
        column=0,
        **widgetpack
    )

    # home robot
    goHome()
    plotData()

    # load data into widgets
    root.after_idle(updateWindow)

    # start window
    root.mainloop()


if __name__ == "__main__":
    openMainWindow()
