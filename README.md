# Social Navigation For A Quadruped Mobile Robot

## Contents
repository containing resources for Sebastian Aegidius' MSc thesis project!

In this repository are the python scripts for the implementations of the Social Force Model (SFM) and novel Augmented Social Force Model (ASFM) on the Boston Dynamics Spot robot for mobile navigation.

The mobile robot used in this project:
![spot shown](https://user-images.githubusercontent.com/66956640/188476203-c055e23c-2813-4460-a432-e5dfed2b4cf9.png)

Sebastian Aegidius' MSc thesis report is also available in this repository.

## Running the code

the script get_pose.py and spot_interface.py are python scripts to run on Boston Dynamics Spot.

The SFM.py is the SFM implementation, and ASFM.py is the ASFM implementation. Both scripts require a zed2i stereo camera feed to post on the local ROS master url to work as they use both the Spot kinematic_state topic and certain topics form the Zed2i's human detection.


## Video Demonstration
[Here](https://youtu.be/36d5Frar4pE) is a link to a video demonstration of the two models implemented.
