# CV Calibration
---
<a id="md-description" name="description"></a>
## Description
---
This ROS package is made to be used in combination with [ros_kortex_vision](https://github.com/Kinovarobotics/ros_kortex_vision) package to provide a way to easily calibrate the color and depth camera streams. The calibration will however only be visible using [RViz](http://wiki.ros.org/rviz) or any visualizer that can listen to the static transforms topic to offset the point cloud data from the depth stream.
This was meant as a solution for [Issue #1](https://github.com/Kinovarobotics/ros_kortex_vision/issues/1)

<a id="md-description" name="contents"></a>
## Table of contents
---
- [Description](#description)
- [Table of contents](#contents)
- [Requirements](#Requirements)
- [Installation](#Installation)
    - [Build](#Build)
- [Usage](#Usage)
    - [Running the program](#run)
    - [Using the program](#use)


## Requierements
---
This project was designed to be achievable by anyone using things you can find or make easily.
You will need:
- A flat rectangular object. It has to be of a uniform color. For better results, it has to be brighter than your background. Optionnally you can use a flashlight to make it brighter to the camera. This is needed for the color shape detection.
- A way to hold this object at a distance from the background. If whatever you are using to hold the object in place is thin enough, you will be able to filter it out using the program.
- A dark (or darker than the object used for calibration) bakcground.

As mentionned, this setup does not need to be fancy. The setup used for testing this program was made of a heavy base plate, two clamps, a cardboard square (any rectangular shape will do) covered with a white paper sheet, and some tape to hold everything together.

## Installation
---
This guide assumes you have already used and built the [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) library. This means you need to install ROS and make a catkin workspace.

### Build
---
These are the steps to build the project:
```sh
cd ~/<catkin_workspace>/src
git clone https://github.com/IlianaKinova/cv_calibration.git
python3 -m pip install -r cv_calibration/requirements.txt
cd ../
```

Now, for the the first time only, you will need to make the project:
```sh
catkin_make
```

And now you can source your workspace:
```sh
source devel/setup.bash
```
This step is needed in every new terminal you open and want to run the code.

## Usage
---
<a id="md-run" name="run"></a>
### Running the program
---
To run the program, make sure you are in your catkin workspace directory and make sure you have sourced your project ([see Build](#Build)). Start the kinova_vision_rgbd driver from [ros_kortex_vision](https://github.com/Kinovarobotics/ros_kortex_vision#usage). Open RViz using:
```sh
rviz
```
And now you can launch the program:
```sh
roslaunch cv_calibration rviz_calibration.launch
```

<a id="md-use" name="use"></a>
### Using the program
---
The program will guide you through the steps to make it work.
A video will be available soon explaining how calibrate the streams.
