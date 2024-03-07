## Python Gesture Control

Python 3.11 is required to run OpenCV, please ensure that you are not trying to run with a different version 


This project uses poetry for python package management. To install poetry enter 
`pip install poetry`

After installing poetry enter the projects motionTracking directory and enter `poetry install`

As a final step go into the handcontrol directory and enter the command `poetry run main.py` , this will launch the program

To exit the gesture recognition tool press the q key

## Commands & Usage
This program uses both hands to control spotify.  When hands are in view of the camera to not keeping them in a fist prevents them from running a gesture, lifting a finger on the hand results in a gesture being run.

### Right Hand Gestures
- Right Thumb Up : Launch Spotify Application
- Right Index & Pinkey Up : Play/Pause
- Right Middle & Index Up : Turn Volume Down 
- Right Ring Up : Play Previous Song
- Right Middle Up : Play Next Song
- Right Pinkey & Thumb Up : Turn Volume Up
### Left Hand Gestures
- Left Thumb Up : Like Song
- Left Index Up : Go To Personal Library
