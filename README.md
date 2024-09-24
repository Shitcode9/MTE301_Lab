# Labs for MTE301: Programming for Mechatronics
## Table of Contents

[Workspace setup](https://github.com/TakShimoda/MTE301_Lab?tab=readme-ov-file#workspace-setup)<br />
[Lab 1: Elementary Robotic Navigation](https://github.com/TakShimoda/MTE301_Lab?tab=readme-ov-file#lab-1-elementary-robotic-navigation)<br /> 
[Lab 2: Obstacle Avoidance](https://github.com/TakShimoda/MTE301_Lab?tab=readme-ov-file#lab-2-obstacle-avoidance) 

## Workspace setup and Information
### Basic setup
Follow the same steps discussed in the lab manuals. What you do may be different based on whether it's the first time cloning, or you did parts of lab 1 and want to re-clone the repository for lab 2 while keeping the changes to lab1.cpp. In that case, follow the lab 2 manual closely.

In general, you clone the repository:
```
git clone https://github.com/TakShimoda/MTE301_Lab
```
Then, you go to your workspace and open Visual Studio Code with the following:
```
cd ~/MTE301_Lab/Lab1 #assuming you cloned the repository in the root directory, and you're working on lab 1
code . # open VS code
```

### Extra functionalities
**Pause and rewind/fast forward**: at any time the SFML window is open when you execute your code, you can pause it by pressing ```p``` on your keyboard. Press ```p``` again to unpause it. While it's paused, you can also rewind or fast-forward by pressing the left and right arrow keys to carefully observe how your robot is moving each iteration.

### Debugger
Optionally, if you want to use the debugger to observe the stack trace and keep track of variables to observe if your code is working as intended, go to the [VSCode_Files](https://github.com/TakShimoda/MTE301_Lab/tree/master/VSCode_Files) folder and follow the steps there.
## Lab 1: Elementary Robotic Navigation
<img src="https://github.com/user-attachments/assets/d9214c43-6af7-4de7-a8cb-cb3bf2725aef" width="500">

After you finish your code (assuming you cloned this repository to the root directory):

```cd ~/MTE301_Lab/Lab1```

```make lab1```

```./lab1```

## Lab 2: Obstacle Avoidance

After you finish your code (assuming you cloned this repository to the root directory):

```cd ~/MTE301_Lab/Lab2```

```make lab2```

```./lab2```
