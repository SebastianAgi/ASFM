# Social Navigation For A Quadruped Mobile Robot

## Contents
repository containing resources for Sebastian Aegidius' MSc thesis project!

In this repository are the python scripts for the implementations of the Social Force Model (SFM) and novel Augmented Social Force Model (ASFM) on the Boston Dynamics Spot robot for mobile navigation.

The mobile robot used in this project:
![spot shown](https://user-images.githubusercontent.com/66956640/188476203-c055e23c-2813-4460-a432-e5dfed2b4cf9.png | width=100)
<img src="https://camo.githubusercontent.com/331400aee821efda2e36ee9b3bc8bce93b975109/68747470733a2f2f6779617a6f2e636f6d2f65623563353734316236613961313663363932313730613431613439633835382e706e67" alt="" data-canonical-src="https://gyazo.com/eb5c5741b6a9a16c692170a41a49c858.png" width="200" height="400" />

Sebastian Aegidius' MSc thesis report is also available in this repository.

## Running the code

the script get_pose.py and spot_interface.py are python scripts to run on Boston Dynamics Spot.

The SFM.py is the SFM implementation, and ASFM.py is the ASFM implementation. Both scripts require a zed2i stereo camera feed to post on the local ROS master url to work as they use both the Spot kinematic_state topic and certain topics form the Zed2i's human detection.


## Video Demonstration
[Here](https://youtu.be/36d5Frar4pE) is a link to a video demonstration of the two models implemented.
