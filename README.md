# Project Sleepy

- [Project Sleepy](#project-sleepy)
  - [Installing on Your Computer](#installing-on-your-computer)
    - [Prepare your catkin workspace.](#prepare-your-catkin-workspace)
    - [Clone the repository to your computer.](#clone-the-repository-to-your-computer)
    - [Install dependencies](#install-dependencies)
    - [Build](#build)
  - [Remote Access to the Robot](#remote-access-to-the-robot)
    - [Connect to the robot](#connect-to-the-robot)
    - [Moonlight (Remote Acess to the robot)](#moonlight-remote-acess-to-the-robot)
    - [VPN (Optional)](#vpn-optional)
  - [Useful Command Line Tools](#useful-command-line-tools)
    - [Catkin](#catkin)
    - [ZeroTier (Optional)](#zerotier-optional)
  - [Useful Links](#useful-links)



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

When trying to clone with https, you will need to login. Github does not accept anymore the classic authentication, you need to use a token instead of your password. To generate a new token : profile > settings > developper settings > token(classic) > generate new token (classic). Then name it, select an expiration date and the authorizations you want and click on generate. Save that token somewhere, you won't be able to see it again.
An anternative is to use ssh instead of https.

### Install dependencies
After you have cloned the repo to your computer, install all the missing dependencies:

```
sudo apt-get install v4l-utils
rosdep install --from-paths src --ignore-src -r -y
sudo apt --fix-broken install
```

It will ask you to enter password for secure boot twice, use password ``robotrobot`` both times.

You have to use tab to highlight the ``ok`` and enter to click it, because the mouse won't work.

```
rosdep install --from-paths src --ignore-src -r -y
```

If you don't do this step you'll probably get an error when trying to build the packages.

### Build
Done! Everything should be installed and ready for you to build the packages with catkin.


## Remote Access to the Robot
### Connect to the robot

Go into wifi settings and connect to the wifi network ``sleepy``

The password is ``sleppy25``

You should know have access to the robot. Its IP address is 10.42.0.1

You should also have connection to internet. But it will be slow because the robot has a bad network card.





### Moonlight (Remote Acess to the robot)


Make sure you have snap installed, should be installed by default.

open terminal and run:

```
sudo snap install moonlight
```

From now on, every time you want to connect to the robot, run in terminal:

```
moonlight
```

If you are connected to the wifi network ``sleepy``:

A screen named ``robot`` should appear, click it.
Otherwise push the ``+`` button and write in ``10.42.0.1`` and click ``Add``

First time you connect you will have to enter a pin, the pin is ``1234``

If it is the first time you do this it will give you a pin, write it down. Ask somebody how already have connection to the robot 
or connect to the robot with a cable to a monitor.

Go to web browser and go to ``https://localhost:47990/pin`` .
Put in the pin you got on your computer and click ``send``.

From know on you can connect to the robot by running ``moonlight`` in terminal. And clicking the screen with the name robot.

For our purpose, it is recommended to go to the settings tab in moonlight on your computer, it is the gear icon in the top right corner.
Add pin the two upper boxes ``capture keyboard shortcust`` and ``and optimize mouse for remote desktop``.

If it is to slow and laggy, go into the settings tab in moonlight on your computer, it is the gear icon in the top right corner. And lower the frame rate and/or the resolution. This shouldn't be needed if you are connected to the robot through the sleepy wifi network.


### VPN (Optional)
If you want to access the robot from a remote location or you and the robot is in a really locked down wan, you can use zerotier to create a VPN tunnel to it and access it. This is optional and not needed for the project.

run in terminal:

```
curl -s https://install.zerotier.com | sudo bash
```

save the ZeroTier address at end of output

go to https://www.zerotier.com/ and click login

login using google (click the button)

email: robot.is.sleepy@gmail.com

password: WeAreGoingToMakeIt

click sleepy network

go to
Settings -> Advanced -> Manually Add Member
enter your ZeroTier address and click submit

scroll down to Members (under Settings) and add a name to your computer (the one with your ZeroTier address).

Everything is set up!

To connect to the network run (last part is the Network ID)

```
sudo zerotier-cli join 9bee8941b5443912
```

## Useful Command Line Tools

Put this in your ~/.bashrc:

### Catkin

```
alias s="source ~/dd2419_ws/devel/setup.bash"
alias cb="cd ~/dd2419_ws;catkin build;cd -;source ~/dd2419_ws/devel/setup.bash"
```

This gives you two useful bash shortcuts. Check out the ``alias`` command if you don't know how it works. The ``cb`` command can be run anywhere on your computer and will run catkin build and source the setup.bash file without changing your directory. If your catkin workspace is not called ``dd2419_ws`` just switch it out to your workspace's name.

### ZeroTier (Optional)

```
alias vc="sudo zerotier-cli join 9bee8941b5443912"
alias vd="sudo zerotier-cli leave 9bee8941b5443912"
```

These let you connect and disconnect quickly to the ZeroTier VPN.

## Useful commands

1. Filter rosbag: ``rosbag filter <rosbag_to_filter>.bag <rosbag_without_tf>.bag "topic != '/tf'"``
2. Launch rosbag : ``rosbag play --clock --pause <rosbag_filename>.bag`` 

## Useful Links


1. [ARM Documentation](https://drive.google.com/drive/folders/11wl0ss4zelJUnhpM2iadch4rDxFnV4dg?usp=sharing)
2. [Orientation Sensor Fusion](http://wiki.ros.org/imu_filter_madgwick)
3. [Localization](https://roverrobotics.com/blogs/guides/fusing-imu-encoders-with-ros-robot-localization)
