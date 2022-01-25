# Inverse-Kinematics-6DoF-PUMA-GUI-Python

# Content

<ul>
<li>

[What is this?](#what-is-this)

</li>
<li>

[Technical definitions](#technical-definitions)

</li>
<li>

[GUI - Themes, colours and the colourblind](#gui---themes-colours-and-the-colourblind)

</li>
<li>

[Instruction manual](#root-window-manual)

</li>
</ul>

# What is this?
These scripts will be able to provide an intermediate GUI and calculate both forward and inverse kinematics for PUMA-Type 6DoF robots

InvKinGUI will provide a user interface so that you can control your robot.
The matplotlib plot shows a preview of the calculated joint positions and allows you to verify them before executing a move.
It has been designed to be easily imported into other programs for controlling your specific robot.
You are able to customise the GUI to make it either look nicer or provide a little performance boost if you are running on low-power hardware like a raspberry pi.

The functions have been laid out neatly and the code has been formatted so that it isn't too hard for beginners to use and understand.
Useful error messages are provided when errors are encountered for easy fixing.
Settings can be changed directly from the text file if need be.

# Technical definitions

The program handles the robot like the below diagram, the GUI uses degrees for the unit of angle but the radian is used in the code itself.

![kinematic_diagram](images/kinematic_diagram.png?raw=true)

# GUI - Themes, colours and the colourblind

The GUI has been redesigned to be easier to navigate, look more aesthetically pleasing and it can even change its colours. Not only does this give us that sweet sweet dark mode we all know and love but also to account for the colourblind. Seperate colour palettes for the three most common types of colour blindness (protanopia, deuteranopia and tritanopia) can be selected from the colours menu in settings.

## Root window

The window you will interact with most

[Instructions](#root-window-manual)

![root dark](images/root_dark.png?raw=true)

<details>
  <summary>Show light mode and colourblind modes</summary>
  
  ![root_light](images/root_light.png?raw=true)
  light mode

  ![root_protanope](images/root_dark_protanope.png?raw=true)
  dark mode protanope

  ![root_deuteranope](images/root_dark_deuteranope.png?raw=true)
  dark mode deuteranope

  ![root_tritanope](images/root_dark_tritanope.png?raw=true)
  dark mode tritanope

</details>

<hr>

## Settings window

Need to tailor the program to be more like how you want it? settings is the place to visit, select a preset or push all the buttons until something works.

[Instructions](#settings-window-manual)

![settings dark](images/settings_dark.png?raw=true)

<details>
  <summary>Show light mode and colourblind modes</summary>
  
  ![settings_light](images/settings_light.png?raw=true)
  light mode

  ![settings_protanope](images/settings_dark_protanope.png?raw=true)
  dark mode protanope

  ![settings_deuteranope](images/settings_dark_deuteranope.png?raw=true)
  dark mode deuteranope

  ![settings_tritanope](images/settings_dark_tritanope.png?raw=true)
  dark mode tritanope
  
</details>

<hr>

## Colours window

Don't like the preset colour options?
Change them to be exactly what you want with this handy window

[Instructions](#colours-window-manual)

![colours dark](images/colours_dark.png?raw=true)

<details>
  <summary>Show light mode and colourblind modes</summary>
  
  ![colours_light](images/colours_light.png?raw=true)
  light mode

  ![colours_protanope](images/colours_dark_protanope.png?raw=true)
  dark mode protanope

  ![colours_deuteranope](images/colours_dark_deuteranope.png?raw=true)
  dark mode deuteranope

  ![colours_tritanope](images/colours_dark_tritanope.png?raw=true)
  dark mode tritanope

</details>

# Root window manual

See also:

<ul>

<li>

[Root image](#root-window)

</li>
<li>

[Settings manual](#settings-window-manual)

</li>
<li>

[Colours manual](#colours-window-manual)

</li>
</ul>

## Preview

Displays an abstraction of your robot with the old position (ghost) in a more transparent view.

## Inverse kinematics

Takes a final destination and computes the angles required to reach that position. Use the increment and decrement buttons to change the target position or type it directly into the box.

## Forward kinematics

Takes joint angles and computes the final destination. Use the increment and decrement buttons to change the target angles or type it directly into the box.

## Controls - End effector

A slider that can be used for what you like really but I thought would be most useful to control the end effector

## Controls - Emergency stop

Sends the new position directly to the driving program.

When the "E-Stop" button is pressed ```eStopFunc()``` is called which will handle some internal magic and then call ```eStop()```.

**You need to define ```eStop()``` before calling ```openMainWindow()```**

If you do not define you own then the following function will be created:
```
def eStop():
    print("'eStop' function call")
```

## Controls - Travel - Direct

Sends the new position directly to the driving program.

When the "direct" button is pressed ```directTravelFunc()``` is called which will handle some internal magic and then call ```directTravel(data)``` where the single parameter data is a dictionary of the following:

```
{'tx': float,
'ty': float,
'tz': float,
'rx': float,
'ry': float,
'rz': float,
'a1': float,
'a2': float,
'a3': float,
'a4': float,
'a5': float,
'a6': float,
'end': float}
```

**You need to define ```directTravel(data)``` before calling ```openMainWindow()```**

If you do not define you own then the following function will be created:
```
def directTravel(data):
    print("'directTravel' function call")
    print(data)
```

## Controls - Travel - Lerp

Uses linear interpolation to trace a straight line from the old position to the new position and the result is saved to a csv file.

When the "lerp" button is pressed ```lerpTravelFunc()``` is called which will handle some internal magic and then call ```lerpTravel(data)``` where the single parameter path is a string containing the location of the csv file.

**You need to define ```lerpTravel(path)``` before calling ```openMainWindow()```**

If you do not define you own then the following function will be created:
```
def lerpTravel(path):
    print("'lerpTravel' function call")
    print(path)
```

![lerp_image](images/travel_lerp.png?raw=true)

## Controls - Calculate inverse kinematics

Take the parameters from the inverse kinematics frame and use them to compute the joint angles required to reach the destination and put them into the forward kinematics frame. Compute time is sent to the log.

## Controls - Calculate forward kinematics

Take the parameters from the forward kinematics frame and use them to compute the destination of the end effector and put them into the inverse kinematics frame. Compute time is sent to the log.

## Controls - Update preview

Redraw the preview with the most up-to-date information. The button is disabled if settings "Live preview" is selected.

## Controls - Home

Send the robot to its home position.

## Controls - Settings

Open the settings window.

## Log

Shows a list of what is being performed and when each event happened some calculation time statistics are also shown here.

# Settings window manual

See also:

<ul>

<li>

[Root manual](#root-window-manual)

</li>
<li>

[Settings image](#settings-window)

</li>
<li>

[Colours manual](#colours-window-manual)

</li>
</ul>

## Lengths

Define the lengths of each joint on the robot.

## Constraints

Define the maximum range of freedom for each joint, kinematics will return an error if any joints fall outside these constraints.

## Preferences - Marker size
Set the size of the dots representing each joint on the robot between 1 and 8 arbitrary units (steps of 1).

## Preferences - Line width

Set the thickness of the lines used to draw the robot between 1 and 8 arbitrary units (steps of 1).

## Preferences - Show origins

Displays the origins at the start and end of the robot.

## Preferences - Origin size

Length of the origins in % of the whole preview between 4 and 16% (steps of 1%).

## Preferences - Travel on calculation

Automatically perform a direct travel after kinematics are calculated.

This can be changed to perform lerp travel instead by changing all references of this

```
if bool(globalsettings["Automate"]):
    directTravelFunc()
```

to this

```
if bool(globalsettings["Automate"]):
    lerpTravelFunc()
```

## Preferences - Live preview

Automatically update the plot after a change.

## Preferences - Corner radius

Change the corner radius for all widgets between 0 and 10px (steps of 1px).

## Preferences - Show ghost

Shows the old robot position as a ghost in the preview.

## Preferences - Ghost opacity

The percentage opacity of the ghost between 0 and 100% (steps of 10%).

## Preferences - Lerp resolution

The number of steps to lerp between the old and new position between 100 and 1000 steps (steps of 100).

## Presets

### Performance

<ul>
  <li>Marker size: 2</li>
  <li>Line width: 2</li>
  <li>Show origins: false</li>
  <li>Origin size: 0</li>
  <li>Travel on calculation: false</li>
  <li>Live preview: false</li>
  <li>Corner radius: 0</li>
  <li>Show ghost: false</li>
  <li>Ghost opacity: 0</li>
  <li>Lerp resolution: 100</li>
</ul>

### Slim

<ul>
  <li>Marker size: 2</li>
  <li>Line width: 2</li>
  <li>Show origins: true</li>
  <li>Origin size: 16</li>
  <li>Travel on calculation: false</li>
  <li>Live preview: true</li>
  <li>Corner radius: 4</li>
  <li>Show ghost: true</li>
  <li>Ghost opacity: 50</li>
  <li>Lerp resolution: 500</li>
</ul>

### Normal

<ul>
  <li>Marker size: 4</li>
  <li>Line width: 4</li>
  <li>Show origins: true</li>
  <li>Origin size: 8</li>
  <li>Travel on calculation: false</li>
  <li>Live preview: true</li>
  <li>Corner radius: 7</li>
  <li>Show ghost: true</li>
  <li>Ghost opacity: 40</li>
  <li>Lerp resolution: 500</li>
</ul>

### Thick

<ul>
  <li>Marker size: 8</li>
  <li>Line width: 8</li>
  <li>Show origins: true</li>
  <li>Origin size: 8</li>
  <li>Travel on calculation: false</li>
  <li>Live preview: true</li>
  <li>Corner radius: 10</li>
  <li>Show ghost: true</li>
  <li>Ghost opacity: 30</li>
  <li>Lerp resolution: 500</li>
</ul>

## Save / Load

save the current settings or load the settings currently saved.

# Colours window manual

See also:

<ul>
<li>

[Root manual](#root-window-manual)

</li>
<li>

[Settings manual](#settings-window-manual)

</li>
<li>

[Colours image](#colours-window)

</li>
</ul>

## Preview palette

Click the change button next to the colour you would like to change and the colour picker for your native OS will prompt you to select a new colour which will be applied but only take effect after a restart of the program.

## Buttons palette

Click the change button next to the colour you would like to change and the colour picker for your native OS will prompt you to select a new colour which will be applied but only take effect after a restart of the program.

## Sample

Updated after you either choose a colour or when you click the refresh button and shows you all the colours you have selected which you can then save and the program will follow the selected colours after a restart.

## Colour blind modes

Drag the slider to the appropriate colour blind mode and you can preview the new colours. If they do not suit your strength of colour blindness (programs assumes 100% severity) you can change the colours using the selected mode as a starting point.

## Themes

Select whether to use the light or dark theme, can also follow system settings.

## Refresh

Refresh the colour sample on the right-hand side.

## Dimming

Changes how much to dim one button from its partner to distinguish between them.

## Save / Load

save the current settings or load the settings currently saved.
