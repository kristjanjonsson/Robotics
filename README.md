# Robotics
## Setup
This sets up ROS for both the motion node and the keyboard node skeletons. They implement a little service.
* `cd catkin_ws`
* Clone repo into `src/A53070542_assignment_4`
*  `source devel/setup.bash`
* `catkin_make`
* `roslaunch src/A53070542_assignment_4/launch/start_assignment_4.launch`
* In a new terminal `rosrun A53070542_assignment_4 keyboard.py`

## OpenCV
`bbox.py` uses the two motion detection algorithms to draw bounding boxes over movement. To test it out run
```python bbox.py mode fname```
where mode is either f (farneback optical flow) or m (mog2) and fname (optional) is a path to a video file. There are a bunch of parameters that can be tuned. Those are the

* Kernel sizes for dilate and erode.
* Combinations of dilate, erode, open, close.
* Various parameters in calcOpticalFlowFarneback
* Pretty much all the magic numbers you see.

