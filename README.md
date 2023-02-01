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

## Useful Command Line Tools

```
alias s="source ~/dd2419_ws/devel/setup.bash"
alias cb="cd ~/dd2419_ws;catkin build;cd -;source ~/dd2419_ws/devel/setup.bash"
```

## VPN
why do we need this?

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

