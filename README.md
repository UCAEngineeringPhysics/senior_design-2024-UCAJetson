# BearCart

BearCart is an autonomous driving project. The goal is to upgrade an off-the-shelf
RC car into an AI powered self-driving platform. The software is running on a
Raspberry Pi 5 SBC in its native operating system: Raspberry Pi OS.
Visit [documentations](https://ucaengineeringphysics.github.io/BearCart/) for more details.
![portrait](/_DOCS/assemble/mechanical/images/bc_portrait.jpg)

This project is strongly inspired by the 
[DonkeyCar](https://github.com/autorope/donkeycar) project.


## Quick Start
Fire up the terminal on your Raspberry Pi, and run following commands in it.

### Install Dependencies 
```bash
sudo apt install python3-pip
pip install pip --upgrade --break-system-packages
```

### Clone The Repository
```bash
cd ~
git clone https://github.com/UCAEngineeringPhysics/BearCart.git
```

### Install Python Packages
```bash
cd ~/BearCart
pip install -r requirements.txt --break-system-packages
```

## Demo Videos
- [Initial BearCart](https://youtube.com/shorts/Kcm6qQqev3s)
- [Another Autopilot](https://youtu.be/8GX6HnfgrJQ)

## Useful resources

[Nvidia developer isaac ros](https://developer.nvidia.com/isaac/ros)

[Nvidia ISAAC ROS GitHub Repos](https://github.com/NVIDIA-ISAAC-ROS)
[Nvidia ISAAC ROS Documentation](https://nvidia-isaac-ros.github.io/)

[Examples (JetsonHacksNano)](https://github.com/JetsonHacksNano)