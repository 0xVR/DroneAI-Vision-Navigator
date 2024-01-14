# DroneAI-Vision-Navigator
![Logo](https://i.imgur.com/l3ajfEU.png)

Autonomous drone vision system using ROS and PyTorch for real-time object classification. Enables drones to make informed decisions based on their surroundings.

## Installation

1. Clone the repo into your ROS workspace

```sh
git clone https://github.com/0xVR/DroneAI-Vision-Navigator.git
```

2. Build the packages

```sh
colcon build
```

3. Source the environment to include the newly built packages.

```sh
source install/setup.bash
```

If you are using a different shell (like `zsh`), adjust the setup script accordingly.

## Usage

A barebones decision making plugin has been included in the `plugins/` directory. This file may be edited to add decision-making functionality to the drone, based on the image classification.

The model is a pretrained ResNet 18, so it's based on the ImageNet categories, which can be found [here](https://files.fast.ai/models/imagenet_class_index.json)

Once ready, run the ROS nodes and update dependencies in your existing `package.xml` and `CMakeLists.txt`.
