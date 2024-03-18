# ASFM: Augmented Social Force Model for Legged Robot Social Navigation

**Authors**: Sebastian Aegidius, Rodrigo Chacón-Quesada, Yiannis Demiris, Demitrios Kanoulas

**Website**: https://rpl-cs-ucl.github.io/ASFM/

**Paper**: coming soon

## Contents
In this repository are the python scripts for the implementations of the novel Augmented Social Force Model (ASFM) on the Boston Dynamics Spot robot for mobile navigation along with the standard Social Force Model (SFM) that was used for comparison.

## Video Demonstration
[Here](https://youtu.be/36d5Frar4pE) is a link to a video demonstration of the two models implemented on the robot system.


## Mobile Robot
The mobile robot system used in this project:

<img src="https://user-images.githubusercontent.com/66956640/188476203-c055e23c-2813-4460-a432-e5dfed2b4cf9.png" alt="" data-canonical-src="[](https://user-images.githubusercontent.com/66956640/188476203-c055e23c-2813-4460-a432-e5dfed2b4cf9.png)" width="537" height="500" />


## Syste integration
<img src="https://user-images.githubusercontent.com/66956640/188483403-e3673c34-27d7-48ba-b504-c0e2d04e01e1.jpg" alt="" data-canonical-src="[](https://user-images.githubusercontent.com/66956640/188483403-e3673c34-27d7-48ba-b504-c0e2d04e01e1.jpg)" width="618" height="500" />


## Running the code

the script get_pose.py and spot_interface.py are python scripts to run on Boston Dynamics Spot.

The SFM.py is the SFM implementation, and ASFM.py is the ASFM implementation. Both scripts require a zed2i stereo camera feed to post on the local ROS master url to work as they use both the Spot kinematic_state topic and certain topics form the Zed2i's human detection. Make sure to have the Pose_load.txt file in the ROS workspace on the SpotCore and in the ROS workspace on the Orin.

VNC into Spot and launch the SpotCore:
```
make body_driver
```
In a second terminal run:
```
make pose
```
In a third terminal ssh into the connected Jetson Orin and launch ZED2i camera:
```
ssh Orin
roslaunch zed_wrapper zed2i.launch
```
In a fourth terminal ssh into the Jetosn Orin and run the model script:
```
(SFM)
rosrun zed_obj_det_sub_tutorial SFM.py

(ASFM)
rosrun zed_obj_det_sub_tutorial ASFM.py
```
the beginning and end point coordinates can be changed in the Pose_load.txt file
