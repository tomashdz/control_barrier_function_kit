# CBF ROS package

## Prelininary

You need install  
ROS: http://wiki.ros.org/melodic/Installation/Ubuntu  
TMC HSR simlator: https://docs.hsr.io/hsrb_user_manual_en/howto/pc_install.html  
turtlebot3: If you need agent.

```bash
 sudo apt install ros-melodic-turtlebot3
```

## Setup

Make your ROS catkin workspace in somewhere.

```bash
mkdir -p CBF_ws/src
cd CBF_ws/src
catkin_init_workspace
```

clone this repo in /src directory.

```bash
git clone https://github.com/syaghoub/Risk-based-Stochastic-Control-Barrier-Functions-.git
```

CBF_ROS directory in the repo. is actual ROS pkg.
You can build the ROS package.

```bash
cd CBF_ws/
catkin_make
```

## Usage

You can run follwoing command when you start to use this workspace every time.

```bash
cd CBF_ws/
source devel/setup.sh  
```

### launch
