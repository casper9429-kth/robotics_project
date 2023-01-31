# Project Sleepy

## Installing on Your Computer
### Prepare your catkin workspace.
Move to your catkin workspace directory and clean it out:

```
cd dd2419_ws
catkin clean
```

Delete the src folder.

### Clone the repository to your computer.

Clone the project repository (= all the code) into your workspace's ``src`` folder (the new ``src`` folder will be created for you):

```
git clone <project adress> src
```

Replace \<project adress\> with the project adress you find by clicking the green button on the main project page with '\<\> Code' written on it and choose https (or ssh if you know what you are doing).

### Install dependencies
After you have cloned the repo to your computer, install all the missing dependencies:

```
rosdep install --from-paths src --ignore-src -r -y
```

If you don't do this step you'll probably get an error when trying to build the imu package.

### Build
Done! Everything should be installed and ready for you to build the packages with catkin.

## Useful Command Line Tools

```
alias s="source ~/dd2419_ws/devel/setup.bash"
alias cb="cd ~/dd2419_ws;catkin build;cd -;source ~/dd2419_ws/devel/setup.bash"
```
