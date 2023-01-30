# Project Sleepy

## Installing on your computer
### Clone the repository to your computer.
Move to an empty catkin workspace directory and run

```git clone <project adress> src```.

Replace \<project adress\> with the project adress you find by clicking the green button on the main project page with '\<\> Code' written on it and choosing either https or ssh depending on which you prefer.
### Install dependencies
After you have cloned the repo to your computer, run the following command to install all the dependencies:

```rosdep install --from-paths src --ignore-src -r -y```

If you don't do this step you'll get some error when trying to build the imu-package.

### Build
Run ``catkin build`` and all that yazz.
