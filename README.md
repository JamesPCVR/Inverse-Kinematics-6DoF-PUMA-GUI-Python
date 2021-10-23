# Inverse-Kinematics-6DoF-PUMA-GUI-Python

These scripts will be able to provide an intermediate GUI and calculate both forward and inverse kinematics for PUMA-Type 6DoF robots

InvKinGUI will provide a basic user interface so that you can control your robot.
The matplotlib plot shows a preview of the calculated joint positions and allows you to verify them before executing a move.
It has been designed to be easily imported into other programs for controlling your specific robot.
You are able to customise the GUI to make it either look nicer or provide a little performance boost if you are running on low-power hardware like an RPi.

As of now the GUI looks like this

![Kinematics GUI](images/InvKinGUIRootImg.png?raw=true "Kinematics GUI")

![Settinsg GUI](images/InvKinGUISettingsImg.png?raw=true "Settings GUI")

The functions have been laid out neatly so that it isnt too hard for beginners to use and understand.
Useful error messages are provided when errors are encountered for easy fixing.
Settings can be changed directly from the text file if need be.
