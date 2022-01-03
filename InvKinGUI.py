import numpy as np
from tkinter import *
from tkinter import messagebox
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from datetime import datetime
from time import perf_counter
from PIL import Image, ImageTk
import InvKin
import FwdKin

#calculate inverse kinematics
def calcInverseKinematics():
    global xdata
    global ydata
    global zdata
    
    updateLog('Calculating inverse kinematics')
    start = perf_counter()

    #collect joint lengths
    jointlengths = np.zeros(6)
    for i in range(6):
        jointlengths[i] = globalsettings[f'A{i+1}_length']

    #collect target data
    position = np.array([float(tx.get()),
                         float(ty.get()),
                         float(tz.get()),
                         float(rx.get()),
                         float(ry.get()),
                         float(rz.get())])
    
    #calculation
    raw = InvKin.invKin(jointlengths, position)

    if raw:
        angles = raw[0]
        xdata = raw[1]
        ydata = raw[2]
        zdata = raw[3]

        #prepare new data
        a1angle = round(np.degrees(angles[0]), 2)
        a2angle = round(np.degrees(angles[1]), 2)
        a3angle = round(np.degrees(angles[2]), 2)
        a4angle = round(np.degrees(angles[3]), 2)
        a5angle = round(np.degrees(angles[4]), 2)
        a6angle = round(np.degrees(angles[5]), 2)

        #validata new data
        error = False
        if not (globalsettings['A1_constr_neg'] < a1angle < globalsettings['A1_constr_pos']):
            error = True
        elif not (globalsettings['A2_constr_neg'] < a2angle < globalsettings['A2_constr_pos']):
            error = True
        elif not (globalsettings['A3_constr_neg'] < a3angle < globalsettings['A3_constr_pos']):
            error = True
        elif not (globalsettings['A4_constr_neg'] < a4angle < globalsettings['A4_constr_pos']):
            error = True
        elif not (globalsettings['A5_constr_neg'] < a5angle < globalsettings['A5_constr_pos']):
            error = True
        elif not (globalsettings['A6_constr_neg'] < a6angle < globalsettings['A6_constr_pos']):
            error = True

        if error:
            #finish
            end = perf_counter()
            updateLog(f'ERROR occurred in inverse kinematics (took {round((end-start)*1000, 2)}ms)')
            if bool(globalsettings['Update']):
                plotData()
            messagebox.showerror('An error occurred','Angles fall outside constraints')
        
        else:
            #erase old data
            a1.delete(0, END)
            a2.delete(0, END)
            a3.delete(0, END)
            a4.delete(0, END)
            a5.delete(0, END)
            a6.delete(0, END)

            #insert new data
            a1.insert(0, a1angle)
            a2.insert(0, a2angle)
            a3.insert(0, a3angle)
            a4.insert(0, a4angle)
            a5.insert(0, a5angle)
            a6.insert(0, a6angle)

            #finish
            end = perf_counter()
            updateLog(f'Inverse kinematics calculated (took {round((end-start)*1000, 2)}ms)')
            if bool(globalsettings['Update']):
                plotData()
            if bool(globalsettings['Automate']) and not error:
                goToFunc()
    
    else:
        #finish
        end = perf_counter()
        updateLog(f'ERROR occurred in inverse kinematics (took {round((end-start)*1000, 2)}ms)')
        if bool(globalsettings['Update']):
                plotData()
        messagebox.showerror('An error occurred','Destination is too far away')

#calculate forward kinematics
def calcForwardKinematics():
    global xdata
    global ydata
    global zdata
    
    updateLog('Calculating forward kinematics')
    start = perf_counter()

    #validate incoming data
    error = False
    if not (globalsettings['A1_constr_neg'] < globalsettings[f'A1_length'] < globalsettings['A1_constr_pos']):
        error = True
    elif not (globalsettings['A2_constr_neg'] < globalsettings[f'A2_length'] < globalsettings['A2_constr_pos']):
        error = True
    elif not (globalsettings['A3_constr_neg'] < globalsettings[f'A3_length'] < globalsettings['A3_constr_pos']):
        error = True
    elif not (globalsettings['A4_constr_neg'] < globalsettings[f'A4_length'] < globalsettings['A4_constr_pos']):
        error = True
    elif not (globalsettings['A5_constr_neg'] < globalsettings[f'A5_length'] < globalsettings['A5_constr_pos']):
        error = True
    elif not (globalsettings['A6_constr_neg'] < globalsettings[f'A6_length'] < globalsettings['A6_constr_pos']):
        error = True

    if error:
        #finish
        end = perf_counter()
        updateLog(f'ERROR occurred in forward kinematics (took {round((end-start)*1000, 2)}ms)')
        if bool(globalsettings['Update']):
            plotData()
        messagebox.showerror('An error occurred','Angles fall outside constraints')
    
    else:
        #collect joint lengths
        jointlengths = np.zeros(6)
        for i in range(6):
            jointlengths[i] = globalsettings[f'A{i+1}_length']

        #collect target data
        jointangles = np.array([float(a1.get()),
                                float(a2.get()),
                                float(a3.get()),
                                float(a4.get()),
                                float(a5.get()),
                                float(a6.get())])
        
        #calculation
        raw = FwdKin.fwdKin(jointlengths, np.radians(jointangles))

        positions = raw[0]
        xdata = raw[1]
        ydata = raw[2]
        zdata = raw[3]
        
        #erase old data
        tx.delete(0, END)
        ty.delete(0, END)
        tz.delete(0, END)
        rx.delete(0, END)
        ry.delete(0, END)
        rz.delete(0, END)

        #insert new data
        tx.insert(0, round(positions[0], 2))
        ty.insert(0, round(positions[1], 2))
        tz.insert(0, round(positions[2], 2))
        rx.insert(0, round(np.degrees(positions[3]), 2))
        ry.insert(0, round(np.degrees(positions[4]), 2))
        rz.insert(0, round(np.degrees(positions[5]), 2))
        
        #finish
        end = perf_counter()
        updateLog(f'Forward kinematics calculated (took {round((end-start)*1000, 2)}ms)')
        if bool(globalsettings['Update']):
            plotData()
        if bool(globalsettings['Automate']) and not error:
            goToFunc()

#set joint data to the home position
def goHome():
    global xdata
    global ydata
    global zdata

    #erase old data
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

    #insert new data
    tx.insert(0, str(globalsettings['A3_length']+
                     globalsettings['A4_length']+
                     globalsettings['A5_length']+
                     globalsettings['A6_length']))
    ty.insert(0, '0.0')
    tz.insert(0, str(globalsettings['A1_length']+
                     globalsettings['A2_length']))
    rx.insert(0, '0.0')
    ry.insert(0, '0.0')
    rz.insert(0, '0.0')
    a1.insert(0, '0.0')
    a2.insert(0, '0.0')
    a3.insert(0, '90.0')
    a4.insert(0, '0.0')
    a5.insert(0, '0.0')
    a6.insert(0, '0.0')

    #create plot data
    xpoint1 = globalsettings['A3_length']
    xpoint2 = xpoint1 + globalsettings['A4_length']
    xpoint3 = xpoint2 + globalsettings['A5_length']
    xpoint4 = xpoint3 + globalsettings['A6_length']
    zpoint1 = globalsettings['A1_length']
    zpoint2 = zpoint1 + globalsettings['A2_length']
    xdata = [0, 0, 0, xpoint1, xpoint2, xpoint3, xpoint4]
    ydata = [0, 0, 0, 0, 0, 0, 0]
    zdata = [0, zpoint1, zpoint2, zpoint2, zpoint2, zpoint2, zpoint2]
    
    #finish
    updateWindow()
    updateLog('Robot homed')
    if bool(globalsettings['Update']):
        plotData()
    if bool(globalsettings['Automate']):
        goToFunc()

#pass data to another program
def goToFunc():
    updateLog('Travelling to position')
    data = {'tx':float(tx.get()),
            'ty':float(ty.get()),
            'tz':float(tz.get()),
            'rx':float(rx.get()),
            'ry':float(ry.get()),
            'rz':float(rz.get()),
            'a1':float(a1.get()),
            'a2':float(a2.get()),
            'a3':float(a3.get()),
            'a4':float(a4.get()),
            'a5':float(a5.get()),
            'a6':float(a6.get()),
            'end':float(endeffectorslider.get())}
    goTo(data)

def eStopFunc(): #handle eStop button input
    updateLog('Emergency stop')
    eStop()

#plot data onto window
def plotData():
    global xdata
    global ydata
    global zdata
    
    #clear the current plot
    ax.clear()

    #collect joint data
    totallength = globalsettings['A1_length']
    totallength += globalsettings['A2_length']
    totallength += globalsettings['A3_length']
    totallength += globalsettings['A4_length']
    totallength += globalsettings['A5_length']
    totallength += globalsettings['A6_length']
    
    #scale plot to robot
    xytune = 0.65
    ztune = 1
    xylim = (-totallength*xytune, totallength*xytune)
    ax.set(xlim=xylim, ylim=xylim, zlim=(0,totallength*ztune))

    #plot data
    ax.scatter(xdata[0], ydata[0], zdata[0], c='#000000', linewidths=globalsettings['Marker_size'], marker='D')
    ax.scatter(xdata[1], ydata[1], zdata[1], c='#00ff00', linewidths=globalsettings['Marker_size'], marker='o')
    ax.scatter(xdata[2], ydata[2], zdata[2], c='#0000ff', linewidths=globalsettings['Marker_size'], marker='o')
    ax.scatter(xdata[3], ydata[3], zdata[3], c='#ff00ff', linewidths=globalsettings['Marker_size'], marker='o')
    ax.scatter(xdata[4], ydata[4], zdata[4], c='#ffff00', linewidths=globalsettings['Marker_size'], marker='o')
    ax.scatter(xdata[5], ydata[5], zdata[5], c='#00ffff', linewidths=globalsettings['Marker_size'], marker='o')
    ax.scatter(xdata[6], ydata[6], zdata[6], c='#000000', linewidths=globalsettings['Marker_size'], marker='D')
    ax.plot(xdata[0:2], ydata[0:2], zdata[0:2], c='#ff0000', linewidth=globalsettings['Line_width'])
    ax.plot(xdata[1:3], ydata[1:3], zdata[1:3], c='#00ff00', linewidth=globalsettings['Line_width'])
    ax.plot(xdata[2:4], ydata[2:4], zdata[2:4], c='#0000ff', linewidth=globalsettings['Line_width'])
    ax.plot(xdata[3:5], ydata[3:5], zdata[3:5], c='#ff00ff', linewidth=globalsettings['Line_width'])
    ax.plot(xdata[4:6], ydata[4:6], zdata[4:6], c='#ffff00', linewidth=globalsettings['Line_width'])
    ax.plot(xdata[5:7], ydata[5:7], zdata[5:7], c='#00ffff', linewidth=globalsettings['Line_width'])

    #plot origins 
    if globalsettings['Show_origins'] == 1:
        plotOrigin(0,
                   0,
                   0,
                   0,
                   0,
                   0,
                   totallength/50*globalsettings['Origin_size']*xytune)
        plotOrigin(float(tx.get()),
                   float(ty.get()),
                   float(tz.get()),
                   float(rx.get()),
                   float(ry.get()),
                   float(rz.get()),
                   totallength/50*globalsettings['Origin_size']*xytune)
    ax.set_xlabel('X-axis (mm)')
    ax.set_ylabel('Y-axis (mm)')
    ax.set_zlabel('Z-axis (mm)')
    canvas.draw()
    updateLog('Preview updated')

#plot axis origins onto point with rotation
def plotOrigin(tx, ty, tz, rx, ry, rz, length):
    #transform origin
    rx = np.radians(rx)
    ry = np.radians(ry)
    rz = np.radians(rz)
    rotx = ((1, 0, 0),
            (0, np.cos(rx), -np.sin(rx)),
            (0, np.sin(rx), np.cos(rx)))
    roty = ((np.cos(ry), 0, np.sin(ry)),
            (0, 1, 0),
            (-np.sin(ry), 0, np.cos(ry)))
    rotz = ((np.cos(rz), -np.sin(rz), 0),
            (np.sin(rz), np.cos(rz), 0),
            (0, 0, 1))
    rotall = np.matmul(np.matmul(rotz, roty), rotx)
    xplots = np.matmul(rotall, ((0,length), (0,0), (0,0)))
    yplots = np.matmul(rotall, ((0,0), (0,length), (0,0)))
    zplots = np.matmul(rotall, ((0,0), (0,0), (0,length)))
    xplots[0] += tx
    xplots[1] += ty
    xplots[2] += tz
    yplots[0] += tx
    yplots[1] += ty
    yplots[2] += tz
    zplots[0] += tx
    zplots[1] += ty
    zplots[2] += tz

    #plot origin
    ax.plot(*xplots, c='#ff0000', linewidth=globalsettings['Line_width'])
    ax.plot(*yplots, c='#00ff00', linewidth=globalsettings['Line_width'])
    ax.plot(*zplots, c='#0000ff', linewidth=globalsettings['Line_width'])    

#set settings saved in 'settings.txt'
def setSettings():
    global globalsettings
    file = open('settings.txt', 'r')
    settings = {}
    for i in file:
        isname = True
        issetting = False
        name = ''
        setting = ''
        for j in i:
            if j == ':':
                isname = False
                issetting = True
                continue
            if j == '\n':
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

#save current settings to 'settings.txt'
def saveSettings(tell=False, onlysteps=False):
    #collect all settings and perform data type verification
    error = False
    try:
        msg = 'Configure Tx, Ty and Tz'
        globalsettings['Inv_t_step'] = float(inverseconfigstepsizet.get())
        msg = 'Configure Rx, Ry and Rz'
        globalsettings['Inv_r_step'] = float(inverseconfigstepsizer.get())
        msg = 'Configure A1, A2, A3, A4, A5 and A6'
        globalsettings['Fwd_step'] = float(forwardconfigstepsize.get())
        if not onlysteps:
            try:
                msg = 'Axis 1 to Axis 2'
                globalsettings['A1_length'] = float(a1length.get())
                msg = 'Axis 2 to Axis 3'
                globalsettings['A2_length'] = float(a2length.get())
                msg = 'Axis 3 to Axis 4'
                globalsettings['A3_length'] = float(a3length.get())
                msg = 'Axis 4 to Axis 5'
                globalsettings['A4_length'] = float(a4length.get())
                msg = 'Axis 5 to Axis 6'
                globalsettings['A5_length'] = float(a5length.get())
                msg = 'Axis 6 to End'
                globalsettings['A6_length'] = float(a6length.get())

                try:
                    msg = 'Axis 1'
                    globalsettings['A1_constr_pos'] = float(a1constraintpositive.get())
                    globalsettings['A1_constr_neg'] = float(a1constraintnegative.get())
                    msg = 'Axis 2'
                    globalsettings['A2_constr_pos'] = float(a2constraintpositive.get())
                    globalsettings['A2_constr_neg'] = float(a2constraintnegative.get())
                    msg = 'Axis 3'
                    globalsettings['A3_constr_pos'] = float(a3constraintpositive.get())
                    globalsettings['A3_constr_neg'] = float(a3constraintnegative.get())
                    msg = 'Axis 4'
                    globalsettings['A4_constr_pos'] = float(a4constraintpositive.get())
                    globalsettings['A4_constr_neg'] = float(a4constraintnegative.get())
                    msg = 'Axis 5'
                    globalsettings['A5_constr_pos'] = float(a5constraintpositive.get())
                    globalsettings['A5_constr_neg'] = float(a5constraintnegative.get())
                    msg = 'Axis 6'
                    globalsettings['A6_constr_pos'] = float(a6constraintpositive.get())
                    globalsettings['A6_constr_neg'] = float(a6constraintnegative.get())
                except:
                    error = True
                    messagebox.showerror('An error occurred', f'Constraint "{msg}" must be float')
                    settingswindow.focus_set()
            except:
                error = True
                messagebox.showerror('An error occurred', f'Length "{msg}" must be float')
                settingswindow.focus_set()
    except:
        error = True
        messagebox.showerror('An error occurred', f'"{msg}" must be float')
        root.focus_set()

    if not onlysteps:
        globalsettings['Marker_size'] = int(markersize.get())
        globalsettings['Line_width'] = int(linewidth.get())
        globalsettings['Show_origins'] = int(showoriginstate.get())
        globalsettings['Origin_size'] = int(originsize.get())
        globalsettings['Automate'] = int(automatestate.get())
        globalsettings['Update'] = int(updatestate.get())
    
    #perform other validation
    steperror = None
    if globalsettings['Inv_t_step'] < 0 and not error:
        steperror = 'Tx, Ty and Tz'
    elif globalsettings['Inv_r_step'] < 0 and not error:
        steperror = 'Rx, Ry and Rz'
    elif globalsettings['Fwd_step'] < 0 and not error:
        steperror = 'A1, A2, A3, A4, A5 and A6'
    if steperror:
        error = True
        messagebox.showerror('An error occurred', f'"{steperror}" cannot be negative')

    if not onlysteps:
        negativeerror = None
        if globalsettings['A1_length'] < 0 and not error:
            negativeerror = 'Axis 1 to Axis 2'
        elif globalsettings['A2_length'] < 0 and not error:
            negativeerror = 'Axis 2 to Axis 3'
        elif globalsettings['A3_length'] < 0 and not error:
            negativeerror = 'Axis 3 to Axis 4'
        elif globalsettings['A4_length'] < 0 and not error:
            negativeerror = 'Axis 4 to Axis 5'
        elif globalsettings['A5_length'] < 0 and not error:
            negativeerror = 'Axis 5 to Axis 6'
        elif globalsettings['A6_length'] < 0 and not error:
            negativeerror = 'Axis 6 to End'
        if negativeerror:
            error = True
            messagebox.showerror('An error occurred', f'"{negativeerror}" cannot be negative')
            settingswindow.focus_set()

        ordererror = None
        if globalsettings['A1_constr_neg'] > globalsettings['A1_constr_pos'] and not error:
            ordererror = 'Axis 1'
        elif globalsettings['A2_constr_neg'] > globalsettings['A2_constr_pos'] and not error:
            ordererror = 'Axis 2'
        elif globalsettings['A3_constr_neg'] > globalsettings['A3_constr_pos'] and not error:
            ordererror = 'Axis 3'
        elif globalsettings['A4_constr_neg'] > globalsettings['A4_constr_pos'] and not error:
            ordererror = 'Axis 4'
        elif globalsettings['A5_constr_neg'] > globalsettings['A5_constr_pos'] and not error:
            ordererror = 'Axis 5'
        elif globalsettings['A6_constr_neg'] > globalsettings['A6_constr_pos'] and not error:
            ordererror = 'Axis 6'
        if ordererror:
            error = True
            messagebox.showerror('An error occurred', f'"{ordererror}" must have the smaller value to the left')
            settingswindow.focus_set()
    
    #save to file if no errors occurred
    if not error:
        file = open('settings.txt', 'w')
        for i in globalsettings:
            file.write(f'{i}:{globalsettings[i]}\n')
        file.close()

        #finish
        if tell:
            messagebox.showinfo('Success', 'Settings saved')
            try:
                settingswindow.focus_set()
            except:
                pass
        updateLog('Settings saved')
        updateWindow()
    else:
        updateLog('Settings not saved due to error')

#load settings from 'settings.txt'
def loadSettings(tell=False, preset=None, save=False):
    #load a preset if given name
    confirm = True
    if preset:
        preset = preset.get()
        performance = [2, 2, 0, 4, 0, 0]
        slim = [2, 2, 1, 16, 0, 1]
        normal = [4, 4, 1, 8, 0, 1]
        thick = [8, 8, 1, 8, 0, 1]
        plain = [3, 3, 0, 4, 0, 1]
        current = [globalsettings['Marker_size'],
                   globalsettings['Line_width'],
                   globalsettings['Show_origins'],
                   globalsettings['Origin_size'],
                   globalsettings['Automate'],
                   globalsettings['Update']]
        presets = {'Performance':performance,
                   'Slim':slim,
                   'Normal':normal,
                   'Thick': thick,
                   'Plain': plain}
        simplifier = ['Marker_size',
                      'Line_width',
                      'Show_origins',
                      'Origin_size',
                      'Automate',
                      'Update']
        confirmstring = 'This will override current settings'
        for i in range(6):
            confirmstring += f'\n{simplifier[i]}: {current[i]} -> {presets[preset][i]}'
        confirm = messagebox.askokcancel('Confirm', confirmstring)
        if confirm:
            for i in range(6):
                globalsettings[simplifier[i]] = presets[preset][i]
    
    #delete old values
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
    
    #insert new values
    inverseconfigstepsizet.insert(0, globalsettings['Inv_t_step'])
    inverseconfigstepsizer.insert(0, globalsettings['Inv_r_step'])
    forwardconfigstepsize.insert(0, globalsettings['Fwd_step'])
    a1length.insert(0, globalsettings['A1_length'])
    a2length.insert(0, globalsettings['A2_length'])
    a3length.insert(0, globalsettings['A3_length'])
    a4length.insert(0, globalsettings['A4_length'])
    a5length.insert(0, globalsettings['A5_length'])
    a6length.insert(0, globalsettings['A6_length'])
    a1constraintpositive.insert(0, globalsettings['A1_constr_pos'])
    a1constraintnegative.insert(0, globalsettings['A1_constr_neg'])
    a2constraintpositive.insert(0, globalsettings['A2_constr_pos'])
    a2constraintnegative.insert(0, globalsettings['A2_constr_neg'])
    a3constraintpositive.insert(0, globalsettings['A3_constr_pos'])
    a3constraintnegative.insert(0, globalsettings['A3_constr_neg'])
    a4constraintpositive.insert(0, globalsettings['A4_constr_pos'])
    a4constraintnegative.insert(0, globalsettings['A4_constr_neg'])
    a5constraintpositive.insert(0, globalsettings['A5_constr_pos'])
    a5constraintnegative.insert(0, globalsettings['A5_constr_neg'])
    a6constraintpositive.insert(0, globalsettings['A6_constr_pos'])
    a6constraintnegative.insert(0, globalsettings['A6_constr_neg'])
    markersize.set(globalsettings['Marker_size'])
    linewidth.set(globalsettings['Line_width'])
    showorigin.select() if globalsettings['Show_origins'] == 1 else showorigin.deselect()
    originsize.set(globalsettings['Origin_size'])
    automate.select() if globalsettings['Automate'] == 1 else automate.deselect()
    update.select() if globalsettings['Update'] == 1 else update.deselect()

    #finish
    if confirm:
        if tell:
            messagebox.showinfo('Success', f'{"Preset" if preset else "Settings"} loaded')
            try:
                settingswindow.focus_set()
            except:
                pass
        updateLog(f'{"Preset" if preset else "Settings"} {"(" + preset + ") " if preset else ""}loaded')
        if bool(globalsettings['Update']):
            plotData()
        updateWindow()
        if save:
            saveSettings()
    else:
        settingswindow.focus_set()

#open settings window
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
    
    #unminimise and focus window if already exists
    try:
        settingswindow.deiconify()
        return
    except:
        pass

    #create settings window
    settingswindow = Toplevel(root)
    settingswindow.title('Settings')
    settingswindow.wm_iconphoto(False, ImageTk.PhotoImage(Image.open('icon settings.ico')))
    settingswindow.resizable(width=0, height=0)
    
    #define tkinter variables
    automatestate = IntVar()
    updatestate = IntVar()
    showoriginstate = IntVar()
    presetname = StringVar()
    
    #create settings frames
    lengthsframe = LabelFrame(master=settingswindow, text='Lengths')
    constraintsframe = LabelFrame(master=settingswindow, text='Constraints')
    preferencesframe = LabelFrame(master=settingswindow, text='Preferences')
    settingsactionsframe = LabelFrame(master=settingswindow, text='Actions')
    presetsframe = LabelFrame(master=settingswindow, text='Presets')

    #create settings widgets
    a1lengthlabel = Label(master=lengthsframe, text='Axis 1 to Axis 2')
    a2lengthlabel = Label(master=lengthsframe, text='Axis 2 to Axis 3')
    a3lengthlabel = Label(master=lengthsframe, text='Axis 3 to Axis 4')
    a4lengthlabel = Label(master=lengthsframe, text='Axis 4 to Axis 5')
    a5lengthlabel = Label(master=lengthsframe, text='Axis 5 to Axis 6')
    a6lengthlabel = Label(master=lengthsframe, text='Axis 6 to end')
    a1length = Entry(master=lengthsframe, bg='#ffaaaa', width=10, justify=RIGHT)
    a2length = Entry(master=lengthsframe, bg='#aaffaa', width=10, justify=RIGHT)
    a3length = Entry(master=lengthsframe, bg='#aaaaff', width=10, justify=RIGHT)
    a4length = Entry(master=lengthsframe, bg='#ffaaff', width=10, justify=RIGHT)
    a5length = Entry(master=lengthsframe, bg='#ffffaa', width=10, justify=RIGHT)
    a6length = Entry(master=lengthsframe, bg='#aaffff', width=10, justify=RIGHT)
    a1lengthsunit = Label(master=lengthsframe, text='mm')
    a2lengthsunit = Label(master=lengthsframe, text='mm')
    a3lengthsunit = Label(master=lengthsframe, text='mm')
    a4lengthsunit = Label(master=lengthsframe, text='mm')
    a5lengthsunit = Label(master=lengthsframe, text='mm')
    a6lengthsunit = Label(master=lengthsframe, text='mm')

    a1constraintlabel = Label(master=constraintsframe, text='Axis 1')
    a2constraintlabel = Label(master=constraintsframe, text='Axis 2')
    a3constraintlabel = Label(master=constraintsframe, text='Axis 3')
    a4constraintlabel = Label(master=constraintsframe, text='Axis 4')
    a5constraintlabel = Label(master=constraintsframe, text='Axis 5')
    a6constraintlabel = Label(master=constraintsframe, text='Axis 6')
    a1constraintpositive = Entry(master=constraintsframe, bg='#ffaaaa', width=6, justify=RIGHT)
    a1constraintnegative = Entry(master=constraintsframe, bg='#ff6666', width=6, justify=RIGHT)
    a2constraintpositive = Entry(master=constraintsframe, bg='#aaffaa', width=6, justify=RIGHT)
    a2constraintnegative = Entry(master=constraintsframe, bg='#66ff66', width=6, justify=RIGHT)
    a3constraintpositive = Entry(master=constraintsframe, bg='#aaaaff', width=6, justify=RIGHT)
    a3constraintnegative = Entry(master=constraintsframe, bg='#6666ff', width=6, justify=RIGHT)
    a4constraintpositive = Entry(master=constraintsframe, bg='#ffaaff', width=6, justify=RIGHT)
    a4constraintnegative = Entry(master=constraintsframe, bg='#ff66ff', width=6, justify=RIGHT)
    a5constraintpositive = Entry(master=constraintsframe, bg='#ffffaa', width=6, justify=RIGHT)
    a5constraintnegative = Entry(master=constraintsframe, bg='#ffff66', width=6, justify=RIGHT)
    a6constraintpositive = Entry(master=constraintsframe, bg='#aaffff', width=6, justify=RIGHT)
    a6constraintnegative = Entry(master=constraintsframe, bg='#66ffff', width=6, justify=RIGHT)
    a1constraintsunit = Label(master=constraintsframe, text='deg')
    a2constraintsunit = Label(master=constraintsframe, text='deg')
    a3constraintsunit = Label(master=constraintsframe, text='deg')
    a4constraintsunit = Label(master=constraintsframe, text='deg')
    a5constraintsunit = Label(master=constraintsframe, text='deg')
    a6constraintsunit = Label(master=constraintsframe, text='deg')
    
    markersizelabel = Label(master=preferencesframe, text='Marker size')
    markersize = Scale(master=preferencesframe, from_=1, to=8, orient=HORIZONTAL)
    linewidthlabel = Label(master=preferencesframe, text='Line width')
    linewidth = Scale(master=preferencesframe, from_=1, to=8, orient=HORIZONTAL)
    showoriginlabel = Label(master=preferencesframe, text='Show origins')
    showorigin = Checkbutton(master=preferencesframe, text='', variable=showoriginstate)
    originsizelabel = Label(master=preferencesframe, text='Origin size')
    originsize = Scale(master=preferencesframe, from_=4, to=16, orient=HORIZONTAL)
    automatelabel = Label(master=preferencesframe, text='Travel on calculation')
    automate = Checkbutton(master=preferencesframe, text='', variable=automatestate)
    updatelabel = Label(master=preferencesframe, text='Live preview')
    update = Checkbutton(master=preferencesframe, text='', variable=updatestate)
    
    savebutton = Button(master=settingsactionsframe, text='Save', command=lambda: saveSettings(True))
    loadsavebutton = Button(master=settingsactionsframe, text='Load Saved', command=lambda: loadSettings(True))

    presetnames = ['Performance', 'Slim', 'Normal', 'Thick', 'Plain']
    presetname.set(presetnames[0])
    presetselect = OptionMenu(presetsframe, presetname, *presetnames)
    presetselect.configure(width=20)
    selectbutton = Button(master=presetsframe, text='Load Preset and Save', command=lambda: loadSettings(True, presetname, True))

    #configure settings grid system
    settingswindow.grid_columnconfigure(0, weight=2)
    settingswindow.grid_columnconfigure(1, weight=1)

    #configure lengths grid system
    lengthsframe.grid_columnconfigure(0, weight=1)
    lengthsframe.grid_columnconfigure(2, weight=1)
    lengthsframe.grid_columnconfigure(4, weight=1)
    lengthsframe.grid_columnconfigure(6, weight=1)
    lengthsframe.grid_columnconfigure(8, weight=1)
    lengthsframe.grid_columnconfigure(10, weight=1)

    #configure constraints grid system
    constraintsframe.grid_columnconfigure(0, weight=1)
    constraintsframe.grid_columnconfigure(2, weight=1)
    constraintsframe.grid_columnconfigure(4, weight=1)
    constraintsframe.grid_columnconfigure(6, weight=1)
    constraintsframe.grid_columnconfigure(8, weight=1)
    constraintsframe.grid_columnconfigure(10, weight=1)
    
    #configure preferences grid system
    preferencesframe.grid_columnconfigure(0, weight=1)
    preferencesframe.grid_columnconfigure(1, weight=1)
    preferencesframe.grid_columnconfigure(2, weight=1)
    preferencesframe.grid_columnconfigure(3, weight=1)
    preferencesframe.grid_columnconfigure(4, weight=1)
    preferencesframe.grid_columnconfigure(5, weight=1)
    
    #configure actions grid system
    settingsactionsframe.grid_columnconfigure(0, weight=1)
    settingsactionsframe.grid_columnconfigure(1, weight=1)

    #configure presets grid system
    presetsframe.grid_columnconfigure(0, weight=0)
    presetsframe.grid_columnconfigure(1, weight=1)
    
    #pack settings frames
    lengthsframe.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    constraintsframe.grid(row=1, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    preferencesframe.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    settingsactionsframe.grid(row=3, column=0, padx=8, pady=8, sticky=N+S+E+W)
    presetsframe.grid(row=3, column=1, padx=8, pady=8, sticky=N+S+E+W)

    #pack settings widgets
    a1lengthlabel.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    a2lengthlabel.grid(row=0, column=2, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    a3lengthlabel.grid(row=0, column=4, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    a4lengthlabel.grid(row=0, column=6, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    a5lengthlabel.grid(row=0, column=8, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    a6lengthlabel.grid(row=0, column=10, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    a1length.grid(row=1, column=0, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a2length.grid(row=1, column=2, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a3length.grid(row=1, column=4, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a4length.grid(row=1, column=6, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a5length.grid(row=1, column=8, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a6length.grid(row=1, column=10, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a1lengthsunit.grid(row=1, column=1, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a2lengthsunit.grid(row=1, column=3, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a3lengthsunit.grid(row=1, column=5, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a4lengthsunit.grid(row=1, column=7, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a5lengthsunit.grid(row=1, column=9, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a6lengthsunit.grid(row=1, column=11, padx=(0,8), pady=(0,8), sticky=N+S+E+W)

    a1constraintlabel.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)
    a2constraintlabel.grid(row=0, column=3, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)
    a3constraintlabel.grid(row=0, column=6, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)
    a4constraintlabel.grid(row=0, column=9, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)
    a5constraintlabel.grid(row=0, column=12, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)
    a6constraintlabel.grid(row=0, column=15, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)
    a1constraintpositive.grid(row=1, column=1, padx=0, pady=(0,8), sticky=N+S+E+W)
    a1constraintnegative.grid(row=1, column=0, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a2constraintpositive.grid(row=1, column=4, padx=0, pady=(0,8), sticky=N+S+E+W)
    a2constraintnegative.grid(row=1, column=3, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a3constraintpositive.grid(row=1, column=7, padx=0, pady=(0,8), sticky=N+S+E+W)
    a3constraintnegative.grid(row=1, column=6, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a4constraintpositive.grid(row=1, column=10, padx=0, pady=(0,8), sticky=N+S+E+W)
    a4constraintnegative.grid(row=1, column=9, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a5constraintpositive.grid(row=1, column=13, padx=0, pady=(0,8), sticky=N+S+E+W)
    a5constraintnegative.grid(row=1, column=12, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a6constraintpositive.grid(row=1, column=16, padx=0, pady=(0,8), sticky=N+S+E+W)
    a6constraintnegative.grid(row=1, column=15, padx=(8,0), pady=(0,8), sticky=N+S+E+W)
    a1constraintsunit.grid(row=1, column=2, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a2constraintsunit.grid(row=1, column=5, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a3constraintsunit.grid(row=1, column=8, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a4constraintsunit.grid(row=1, column=11, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a5constraintsunit.grid(row=1, column=14, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    a6constraintsunit.grid(row=1, column=17, padx=(0,8), pady=(0,8), sticky=N+S+E+W)
    
    markersizelabel.grid(row=0, column=0, padx=8, pady=(8,0), sticky=N+S+E+W)
    markersize.grid(row=1, column=0, padx=8, pady=(0,8), sticky=N+S+E+W)
    linewidthlabel.grid(row=0, column=1, padx=8, pady=(8,0), sticky=N+S+E+W)
    linewidth.grid(row=1, column=1, padx=8, pady=(0,8), sticky=N+S+E+W)
    showoriginlabel.grid(row=0, column=2, padx=8, pady=(8,0), sticky=N+S+E+W)
    showorigin.grid(row=1, column=2, padx=8, pady=(8,0), sticky=N+S+E+W)
    originsizelabel.grid(row=0, column=3, padx=8, pady=(8,0), sticky=N+S+E+W)
    originsize.grid(row=1, column=3, padx=8, pady=(0,8), sticky=N+S+E+W)
    automatelabel.grid(row=0, column=4, padx=8, pady=(8,0), sticky=N+S+E+W)
    automate.grid(row=1, column=4, padx=8, pady=(8,0), sticky=N+S+E+W)
    updatelabel.grid(row=0, column=5, padx=8, pady=(8,0), sticky=N+S+E+W)
    update.grid(row=1, column=5, padx=8, pady=(8,0), sticky=N+S+E+W)
    
    savebutton.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W)
    loadsavebutton.grid(row=0, column=1, padx=8, pady=8, sticky=N+S+E+W)

    presetselect.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W)
    selectbutton.grid(row=0, column=1, padx=8, pady=8, sticky=N+S+E+W)

    loadSettings()
    settingswindow.focus_set()

#update log box with new information
def updateLog(message=''):
    #unlock log readout
    logtext.config(state='normal')
    
    if 'log' not in globals():
        #create dummy data on startup
        global log
        log = []
        for i in range(12):
            log.append('(dd/mm/yy @ hh:mm:ss):\n')
    else:
        #queue new message
        if log[-1][23:] != message:
            currenttime = datetime.now().strftime('(%d/%m/%y @ %H:%M:%S): ')
            log.append(f'{currenttime}{message}')
            log.pop(0)

    #print queue to log readout
    logtext.delete('0.0', END)
    for i in range(12):
        logtext.insert(str(float(i+1)), f'{log[i]}\n')

    #lock log readout
    logtext.config(state='disabled')

#get inverse translation step size
def getInverseStepSizeT():
    try:
        step = float(inverseconfigstepsizet.get())
        return step
    except:
        messagebox.showerror('An error occurred', f'"Configure Tx, Ty and Tz" must be float')

#get inverse rotation step size
def getInverseStepSizeR():
    try:
        step = float(inverseconfigstepsizer.get())
        return step
    except:
        messagebox.showerror('An error occurred', f'"Configure Rx, Ry and Rz" must be float')

#get forward rotation step size
def getForwardStepSize():
    try:
        step = float(forwardconfigstepsize.get())
        return step
    except:
        messagebox.showerror('An error occurred', f'"Configure A1, A2, A3, A4, A5 and A6" must be float')

#change x translation by current step
def updateTx(scalar):
    try:
        current = float(tx.get())
    except:
        messagebox.showerror('An error occurred', f'"X Translation" must be float')
    step = getInverseStepSizeT()
    try:
        new = float(current + (step * scalar))
        tx.delete(0, END)
        tx.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change y translation by current step
def updateTy(scalar):
    try:
        current = float(ty.get())
    except:
        messagebox.showerror('An error occurred', f'"Y Translation" must be float')
    step = getInverseStepSizeT()
    try:
        new = float(current + (step * scalar))
        ty.delete(0, END)
        ty.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change z translation by current step
def updateTz(scalar):
    try:
        current = float(tz.get())
    except:
        messagebox.showerror('An error occurred', f'"Z Translation" must be float')
    step = getInverseStepSizeT()
    try:
        new = float(current + (step * scalar))
        tz.delete(0, END)
        tz.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change x rotation by current step
def updateRx(scalar):
    try:
        current = float(rx.get())
    except:
        messagebox.showerror('An error occurred', f'"X Rotation" must be float')
    step = getInverseStepSizeR()
    try:
        new = float(current + (step * scalar))
        rx.delete(0, END)
        rx.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change y rotation by current step
def updateRy(scalar):
    try:
        current = float(ry.get())
    except:
        messagebox.showerror('An error occurred', f'"Y Rotation" must be float')
    step = getInverseStepSizeR()
    try:
        new = float(current + (step * scalar))
        ry.delete(0, END)
        ry.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change z rotation by current step
def updateRz(scalar):
    try:
        current = float(rz.get())
    except:
        messagebox.showerror('An error occurred', f'"Z Rotation" must be float')
    step = getInverseStepSizeR()
    try:
        new = float(current + (step * scalar))
        rz.delete(0, END)
        rz.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change axis 1 angle by current step
def updateA1(scalar):
    try:
        current = float(a1.get())
    except:
        messagebox.showerror('An error occurred', f'"Axis 1" must be float')
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a1.delete(0, END)
        a1.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change axis 2 angle by current step
def updateA2(scalar):
    try:
        current = float(a2.get())
    except:
        messagebox.showerror('An error occurred', f'"Axis 2" must be float')
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a2.delete(0, END)
        a2.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change axis 3 angle by current step
def updateA3(scalar):
    try:
        current = float(a3.get())
    except:
        messagebox.showerror('An error occurred', f'"Axis 3" must be float')
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a3.delete(0, END)
        a3.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change axis 4 angle by current step
def updateA4(scalar):
    try:
        current = float(a4.get())
    except:
        messagebox.showerror('An error occurred', f'"Axis 4" must be float')
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a4.delete(0, END)
        a4.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change axis 5 angle by current step
def updateA5(scalar):
    try:
        current = float(a5.get())
    except:
        messagebox.showerror('An error occurred', f'"Axis 5" must be float')
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a5.delete(0, END)
        a5.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#change axis 5 angle by current step
def updateA6(scalar):
    try:
        current = float(a6.get())
    except:
        messagebox.showerror('An error occurred', f'"Axis 6" must be float')
    step = getForwardStepSize()
    try:
        new = float(current + (step * scalar))
        a6.delete(0, END)
        a6.insert(0, new)
    except Exception:
        pass
    if bool(globalsettings['Update']):
        plotData()

#configure window based on settings
def updateWindow():
    if bool(globalsettings['Automate']):
        passdatabutton.configure(state=DISABLED)
    else:
        passdatabutton.configure(state=NORMAL)
    if bool(globalsettings['Update']):
        updateplotbutton.configure(state=DISABLED)
        plotData()
    else:
        updateplotbutton.configure(state=NORMAL)

#open the root window
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
    global passdatabutton
    global updateplotbutton
    global logtext
    
    global goTo
    global eStop

    #only create placeholder function if user has not defined their own
    if 'goTo' not in locals():
        def goTo(data):
            print('"goTo" function call')
            print(data)

    #only create placeholder function if user has not defined their own
    if 'eStop' not in locals():
        def eStop(): #Emergency stop
            print('"eStop" function call')

    #load saved settings
    setSettings()
    
    #create and configure window object
    root = Tk()
    root.title('InvKinGUI')
    root.wm_iconphoto(False, ImageTk.PhotoImage(Image.open('icon root.ico')))
    root.geometry('+100+100')
    root.resizable(width=0, height=0)

    #define graph frame object
    graphframe = LabelFrame(master=root, text='Preview')

    #create inverse kinematics frames
    inverseframe = LabelFrame(master=root, text='Inverse Kinematics')
    txinverseframe = LabelFrame(master=inverseframe, text='X-Translation')
    tyinverseframe = LabelFrame(master=inverseframe, text='Y-Translation')
    tzinverseframe = LabelFrame(master=inverseframe, text='Z-Translation')
    rxinverseframe = LabelFrame(master=inverseframe, text='X-Rotation')
    ryinverseframe = LabelFrame(master=inverseframe, text='Y-Rotation')
    rzinverseframe = LabelFrame(master=inverseframe, text='Z-Rotation')
    inverseconfigtframe = LabelFrame(master=inverseframe, text='Configure Tx, Ty and Tz')
    inverseconfigrframe = LabelFrame(master=inverseframe, text='Configure Rx, Ry and Rz')

    #create inverse kinematics widgets
    txinverseup = Button(master=txinverseframe, text='Increment', bg='#ffaaaa', command=lambda: updateTx(1))
    tyinverseup = Button(master=tyinverseframe, text='Increment', bg='#aaffaa', command=lambda: updateTy(1))
    tzinverseup = Button(master=tzinverseframe, text='Increment', bg='#aaaaff', command=lambda: updateTz(1))
    rxinverseup = Button(master=rxinverseframe, text='Increment', bg='#ffaaaa', command=lambda: updateRx(1))
    ryinverseup = Button(master=ryinverseframe, text='Increment', bg='#aaffaa', command=lambda: updateRy(1))
    rzinverseup = Button(master=rzinverseframe, text='Increment', bg='#aaaaff', command=lambda: updateRz(1))
    tx = Entry(master=txinverseframe, width=10, justify=RIGHT)
    ty = Entry(master=tyinverseframe, width=10, justify=RIGHT)
    tz = Entry(master=tzinverseframe, width=10, justify=RIGHT)
    rx = Entry(master=rxinverseframe, width=10, justify=RIGHT)
    ry = Entry(master=ryinverseframe, width=10, justify=RIGHT)
    rz = Entry(master=rzinverseframe, width=10, justify=RIGHT)
    txunit = Label(master=txinverseframe, text='mm')
    tyunit = Label(master=tyinverseframe, text='mm')
    tzunit = Label(master=tzinverseframe, text='mm')
    rxunit = Label(master=rxinverseframe, text='deg')
    ryunit = Label(master=ryinverseframe, text='deg')
    rzunit = Label(master=rzinverseframe, text='deg')
    txinversedown = Button(master=txinverseframe, text='Decrement', bg='#ff6666', command=lambda: updateTx(-1))
    tyinversedown = Button(master=tyinverseframe, text='Decrement', bg='#66ff66', command=lambda: updateTy(-1))
    tzinversedown = Button(master=tzinverseframe, text='Decrement', bg='#6666ff', command=lambda: updateTz(-1))
    rxinversedown = Button(master=rxinverseframe, text='Decrement', bg='#ff6666', command=lambda: updateRx(-1))
    ryinversedown = Button(master=ryinverseframe, text='Decrement', bg='#66ff66', command=lambda: updateRy(-1))
    rzinversedown = Button(master=rzinverseframe, text='Decrement', bg='#6666ff', command=lambda: updateRz(-1))
    inverseconfigtlabel1 = Label(master=inverseconfigtframe, text='Increment / Decrement in')
    inverseconfigstepsizet = Entry(master=inverseconfigtframe, width=8, justify=RIGHT)
    inverseconfigstepsizet.insert(END, globalsettings['Inv_t_step'])
    inverseconfigtlabel2 = Label(master=inverseconfigtframe, text='mm steps')
    inverseconfigsavet = Button(master=inverseconfigtframe, text='Save', width=10, command=lambda: saveSettings(True, True))
    inverseconfigrlabel1 = Label(master=inverseconfigrframe, text='Increment / Decrement in')
    inverseconfigstepsizer = Entry(master=inverseconfigrframe, width=8, justify=RIGHT)
    inverseconfigstepsizer.insert(END, globalsettings['Inv_r_step'])
    inverseconfigrlabel2 = Label(master=inverseconfigrframe, text='degree steps')
    inverseconfigsaver = Button(master=inverseconfigrframe, text='Save', width=10, command=lambda: saveSettings(True, True))

    #assign inverse kinematics defaults
    tx.insert(0, 0.0)
    ty.insert(0, 0.0)
    tz.insert(0, 0.0)
    rx.insert(0, 0.0)
    ry.insert(0, 0.0)
    rz.insert(0, 0.0)

    #create forward kinematics frames
    forwardframe = LabelFrame(master=root, text='Forward Kinematics')
    forwardframe1 = LabelFrame(master=forwardframe, text='Axis 1')
    forwardframe2 = LabelFrame(master=forwardframe, text='Axis 2')
    forwardframe3 = LabelFrame(master=forwardframe, text='Axis 3')
    forwardframe4 = LabelFrame(master=forwardframe, text='Axis 4')
    forwardframe5 = LabelFrame(master=forwardframe, text='Axis 5')
    forwardframe6 = LabelFrame(master=forwardframe, text='Axis 6')
    forwardconfigframe = LabelFrame(master=forwardframe, text='Configure A1, A2, A3, A4, A5, and A6')

    #create forward kinematics widgets
    forwardup1 = Button(master=forwardframe1, text='Increment', bg='#ffaaaa', command=lambda: updateA1(1))
    forwardup2 = Button(master=forwardframe2, text='Increment', bg='#aaffaa', command=lambda: updateA2(1))
    forwardup3 = Button(master=forwardframe3, text='Increment', bg='#aaaaff', command=lambda: updateA3(1))
    forwardup4 = Button(master=forwardframe4, text='Increment', bg='#ffaaff', command=lambda: updateA4(1))
    forwardup5 = Button(master=forwardframe5, text='Increment', bg='#ffffaa', command=lambda: updateA5(1))
    forwardup6 = Button(master=forwardframe6, text='Increment', bg='#aaffff', command=lambda: updateA6(1))
    a1 = Entry(master=forwardframe1, width=10, justify=RIGHT)
    a2 = Entry(master=forwardframe2, width=10, justify=RIGHT)
    a3 = Entry(master=forwardframe3, width=10, justify=RIGHT)
    a4 = Entry(master=forwardframe4, width=10, justify=RIGHT)
    a5 = Entry(master=forwardframe5, width=10, justify=RIGHT)
    a6 = Entry(master=forwardframe6, width=10, justify=RIGHT)
    a1unit = Label(master=forwardframe1, text='deg')
    a2unit = Label(master=forwardframe2, text='deg')
    a3unit = Label(master=forwardframe3, text='deg')
    a4unit = Label(master=forwardframe4, text='deg')
    a5unit = Label(master=forwardframe5, text='deg')
    a6unit = Label(master=forwardframe6, text='deg')
    forwarddown1 = Button(master=forwardframe1, text='Decrement', bg='#ff6666', command=lambda: updateA1(-1))
    forwarddown2 = Button(master=forwardframe2, text='Decrement', bg='#66ff66', command=lambda: updateA2(-1))
    forwarddown3 = Button(master=forwardframe3, text='Decrement', bg='#6666ff', command=lambda: updateA3(-1))
    forwarddown4 = Button(master=forwardframe4, text='Decrement', bg='#ff66ff', command=lambda: updateA4(-1))
    forwarddown5 = Button(master=forwardframe5, text='Decrement', bg='#ffff66', command=lambda: updateA5(-1))
    forwarddown6 = Button(master=forwardframe6, text='Decrement', bg='#aaffff', command=lambda: updateA6(-1))
    forwardconfiglabel1 = Label(master=forwardconfigframe, text='Increment / Decrement in')
    forwardconfigstepsize = Entry(master=forwardconfigframe, width=8, justify=RIGHT)
    forwardconfigstepsize.insert(END, globalsettings['Fwd_step'])
    forwardconfiglabel2 = Label(master=forwardconfigframe, text='degree steps')
    forwardconfigsave = Button(master=forwardconfigframe, text='Save', width=10, command=lambda: saveSettings(True, True))

    #assign forwards kinematics defaults
    a1.insert(0, 0.0)
    a2.insert(0, 0.0)
    a3.insert(0, 0.0)
    a4.insert(0, 0.0)
    a5.insert(0, 0.0)
    a6.insert(0, 0.0)

    #create controls frame
    controlsframe = LabelFrame(master=root, text='Control')

    #create controls widgets
    endeffectorlabel = Label(master=controlsframe, text='End Effector')
    endeffectorslider = Scale(master=controlsframe, from_=0, to=100, orient=HORIZONTAL)
    calculateinversebutton = Button(master=controlsframe, text='Calculate Inverse Kinematics', command=calcInverseKinematics)
    calculateforwardbutton = Button(master=controlsframe, text='Calculate Forward Kinematics', command=calcForwardKinematics)
    estopbutton = Button(master=controlsframe, text='E - STOP', bg='#ff0000', fg='#ffffff', command=eStopFunc, font=('lithograph', 20, 'bold'))
    homebutton = Button(master=controlsframe, text='Home', command=goHome)
    passdatabutton = Button(master=controlsframe, text='Travel to Position', command=goToFunc)
    updateplotbutton = Button(master=controlsframe, text='Update Preview', command=plotData)
    opensettingsbutton = Button(master=controlsframe, text='Settings', command=openSettingsWindow)

    #configure log frame
    logframe = LabelFrame(master=root, text='Log')

    #configure log widget
    logtext = Text(master=logframe, height=13)

    #create graph object
    fig = Figure(figsize=(9, 8), dpi=100)
    fig.subplots_adjust(left=0.0, bottom=0.03, right=0.97, top=0.97)
    canvas = FigureCanvasTkAgg(fig, master=graphframe)

    #configure graph object
    ax = fig.add_subplot(111, projection="3d")
    ax.w_xaxis.line.set_color('#ff0000')
    ax.w_yaxis.line.set_color('#00ff00')
    ax.w_zaxis.line.set_color('#0000ff')
    ax.view_init(elev=30, azim=-135)

    #configure root grid system
    root.grid_rowconfigure(0, weight=0)
    root.grid_rowconfigure(1, weight=0)
    root.grid_rowconfigure(2, weight=0)
    root.grid_rowconfigure(3, weight=1)

    #configure inverse kinematics grid system
    inverseframe.grid_rowconfigure(0, weight=3)
    inverseframe.grid_rowconfigure(1, weight=2)
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

    #configure forward kinematics grid system
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

    #configure controls grid system
    controlsframe.grid_columnconfigure(0, weight=0)
    controlsframe.grid_columnconfigure(1, weight=1)
    controlsframe.grid_columnconfigure(2, weight=1)
    controlsframe.grid_columnconfigure(3, weight=1)
    controlsframe.grid_columnconfigure(4, weight=1)
    controlsframe.grid_columnconfigure(5, weight=0)

    #configure log grid system
    logframe.grid_columnconfigure(0, weight=1)

    #pack graph frame
    graphframe.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, rowspan=4)

    #pack graph widget
    toolbar = NavigationToolbar2Tk(canvas, graphframe)
    toolbar.update()
    canvas.get_tk_widget().pack(side=TOP)

    #pack inverse kinematics frames
    inverseframe.grid(row=0, column=1, padx=8, pady=8, sticky=N+S+E+W)
    txinverseframe.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W)
    tyinverseframe.grid(row=0, column=1, padx=8, pady=8, sticky=N+S+E+W)
    tzinverseframe.grid(row=0, column=2, padx=8, pady=8, sticky=N+S+E+W)
    rxinverseframe.grid(row=0, column=3, padx=8, pady=8, sticky=N+S+E+W)
    ryinverseframe.grid(row=0, column=4, padx=8, pady=8, sticky=N+S+E+W)
    rzinverseframe.grid(row=0, column=5, padx=8, pady=8, sticky=N+S+E+W)
    inverseconfigtframe.grid(row=1, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)
    inverseconfigrframe.grid(row=1, column=3, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)

    #pack inverse kinematics widgets
    txinverseup.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    tyinverseup.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    tzinverseup.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    rxinverseup.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    ryinverseup.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    rzinverseup.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    tx.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    ty.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    tz.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    rx.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    ry.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    rz.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    txunit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    tyunit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    tzunit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    rxunit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    ryunit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    rzunit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    txinversedown.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    tyinversedown.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    tzinversedown.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    rxinversedown.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    ryinversedown.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    rzinversedown.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    inverseconfigtlabel1.pack(side=LEFT, padx=(8,0))
    inverseconfigstepsizet.pack(side=LEFT)
    inverseconfigtlabel2.pack(side=LEFT)
    inverseconfigsavet.pack(side=RIGHT, padx=8, pady=(0,8))
    inverseconfigrlabel1.pack(side=LEFT, padx=(8,0))
    inverseconfigstepsizer.pack(side=LEFT)
    inverseconfigrlabel2.pack(side=LEFT)
    inverseconfigsaver.pack(side=RIGHT, padx=8, pady=(0,8))

    #pack forward kinematics frames
    forwardframe.grid(row=1, column=1, padx=8, pady=8, sticky=N+S+E+W)
    forwardframe1.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W)
    forwardframe2.grid(row=0, column=1, padx=8, pady=8, sticky=N+S+E+W)
    forwardframe3.grid(row=0, column=2, padx=8, pady=8, sticky=N+S+E+W)
    forwardframe4.grid(row=0, column=3, padx=8, pady=8, sticky=N+S+E+W)
    forwardframe5.grid(row=0, column=4, padx=8, pady=8, sticky=N+S+E+W)
    forwardframe6.grid(row=0, column=5, padx=8, pady=8, sticky=N+S+E+W)
    forwardconfigframe.grid(row=1, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=3)

    #pack forward kinematics widgets
    forwardup1.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwardup2.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwardup3.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwardup4.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwardup5.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwardup6.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    a1.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    a2.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    a3.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    a4.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    a5.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    a6.grid(row=1, column=0, padx=(8,0), pady=0, sticky=N+S+E+W)
    a1unit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    a2unit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    a3unit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    a4unit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    a5unit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    a6unit.grid(row=1, column=1, padx=(0,8), pady=0, sticky=N+S+E+W)
    forwarddown1.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwarddown2.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwarddown3.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwarddown4.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwarddown5.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwarddown6.grid(row=2, column=0, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    forwardconfiglabel1.pack(side=LEFT, padx=(8,0), pady=4)
    forwardconfigstepsize.pack(side=LEFT, padx=0, pady=4)
    forwardconfiglabel2.pack(side=LEFT, padx=(0,8), pady=4)
    forwardconfigsave.pack(side=RIGHT, padx=8, pady=(0,8))

    #pack controls frame
    controlsframe.grid(row=2, column=1, padx=8, pady=8, sticky=N+S+E+W)

    #pack controls widgets
    endeffectorlabel.grid(row=0, column=0, padx=8, pady=(8,0), sticky=N+S+E+W)
    endeffectorslider.grid(row=1, column=0, padx=8, pady=(0,8), sticky=N+S+E+W)
    calculateinversebutton.grid(row=0, column=1, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    calculateforwardbutton.grid(row=0, column=3, padx=8, pady=8, sticky=N+S+E+W, columnspan=2)
    estopbutton.grid(row=0, column=5, padx=8, pady=8, sticky=N+S+E+W, rowspan=2)
    homebutton.grid(row=1, column=1, padx=8, pady=8, sticky=N+S+E+W)
    passdatabutton.grid(row=1, column=2, padx=8, pady=8, sticky=N+S+E+W)
    updateplotbutton.grid(row=1, column=3, padx=8, pady=8, sticky=N+S+E+W)
    opensettingsbutton.grid(row=1, column=4, padx=8, pady=8, sticky=N+S+E+W)

    #pack log frame
    logframe.grid(row=3, column=1, padx=8, pady=8, sticky=N+S+E+W)

    #pack log widget
    logtext.grid(row=0, column=0, padx=8, pady=8, sticky=N+S+E+W)

    #home robot
    goHome()
    plotData()

    #load data into widgets
    root.after_idle(updateWindow)
    
    #start window
    root.mainloop()

if __name__ == '__main__':
    openMainWindow()
