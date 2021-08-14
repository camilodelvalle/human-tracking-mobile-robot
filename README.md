# Mobile robot for human detection and tracking

ROS implementation of a diﬀerential drive robot (Kobuki) for human detection and tracking using depth images and a SVM supervised learning model. Project developed for the vision-based control and mobile robotics course at the National University of Colombia. This repo also contains the solutions for the Path Planning Laboratory.


## Requirements

- Ubuntu 18.04 Bionic.
- ROS Melodic.
- Gazebo.
- ROS Packages for Kobuki robot.

  - [kobuki](https://github.com/yujinrobot/kobuki)

  - [kobuki_desktop](https://github.com/yujinrobot/kobuki_desktop)

  - [kobuki_core](https://github.com/yujinrobot/kobuki_core)

  - [kobuki_msg](https://github.com/yujinrobot/kobuki_msgs)

- [Python-OpenCV](https://opencv.org/)

- [Python Robotics](https://github.com/AtsushiSakai/PythonRobotics). Used for path planning laboratory.

## Installation

Clone this repo and copy the *rycsv_pkg* package into the *src* folder of your catkin workspace.

```bash
git clone https://github.com/camilodelvalle/human-tracking-mobile-robot
cd human-tracking-mobile-robot
cp -R rycsv_pkg/ ~/catkin_ws/src/
```

Build the package.

```bash
cd ~/catkin_ws
catkin_make
```

## Usage - Path Planning Laboratoty

Path planning using the Probabilistic Roadmap (PRM) method. 

1. Run `./path_planning.py` to generate csv files with a new path or use existing files in the *paths* folder.

2. In a new terminal window, launch gazebo world:

    `roslaunch rycsv_pkg labs.launch`

3. In a new terminal window, run the controller node:

    `rosrun rycsv_pkg controller_node.py -lab4 px_path_example.csv py_path_example.csv`

Demo: [Video - Lab](https://youtu.be/P-NghbBB4m8).

<img src="./demo/lab4.gif"/>


## Usage - Final Project

Mobile robot for human detection and tracking using depth images, SVM and HOG descriptor.

1. In a new terminal window, launch gazebo world:

    `roslaunch rycsv_pkg project.launch`

2. In a new terminal window, run the node for human detection:

    `rosrun rycsv_pkg detector_node.py`

3. In a new terminal window, run the controller node:

    `rosrun rycsv_pkg controller_node.py -project`

Demo: [Video - Project](https://youtu.be/ZPHnVIA5eyM).

<img src="./demo/project.gif"/>


## License

This project is licensed under the MIT License. See the LICENSE file.
