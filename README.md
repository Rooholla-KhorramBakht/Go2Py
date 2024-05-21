# Go2Py

Go2Py is a Pythonic interface and driver for low-level and high-level control of Unitree Go2 quadruped robots. The motivation of this project is to remove the burden of developing interface, safety systems, and basic components required for starting locomotion research using the Go2 quadruped robot. It provides a modular pipeline for real-time communication with the robot in both simulated and real-world environments with a unified interface. 

<p align="center">
  <img src="docs/assets/openfig.png" alt="image" width="60%" height="auto"/>
</p>

This project is comprised of the following components:
- **C++ Bridge:** A dockerized ROS2 bridge built upon the [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) that implements a remote-controlled emergency stop and publishes the robot states as standard ROS2 topics usable by upstream systems such as NAV2. 
- **Robot Interface:** A simple Python class that represents the robot and communicates with the C++ bridge through either DDS (ROS independent) or ROS2 interfaces. 
- **Robot Management FSM:** A finite state machine for controlling the behavior of the robot up to the point of handover to the user low-level controller (sitting down, standing up) with safety monitors (motor temperatures, emergency stops).
- **Robot Model:** A simple to use [Pinocchio](https://github.com/stack-of-tasks/pinocchio) wrapper for computing the kinematics and dynamics parameters of the robot. 
- **Simulation Interface:** Simulation environments based on Mujoco and Nvidia Orbit (To be added) with a Python interface identical to the real robot. 

## How Does Using it Look Like?
Communication with the robot will be as simple as importing a Python class:
```python
from Go2Py.robot.interface.dds import GO2Real
from Go2Py.robot.model import Go2Model
robot = GO2Real(mode='lowlevel')
model = Go2Model()
while running:
    joint_state = robot.getJointStates()
    imu = robot.getIMU()
    remote = robot.getRemoteState()
    model.update(state['q'], state['dq'],T,vel) # T and vel from the EKF
    info = model.getInfo()
    
    #User control computations ...

    robot.setCommands(q_des, dq_des, kp, kd, tau_ff)
```
An identical workflow can be followed for simulation:
```python
from Go2Py.sim.mujoco import Go2Sim
from Go2Py.robot.model import Go2Model
robot = Go2Sim()
model = Go2Model()
robot.standDownReset()
while running:
    joint_state = robot.getJointStates()
    imu = robot.getIMU()
    remote = robot.getRemoteState()
    model.update(state['q'], state['dq'],T,vel) # T and vel from the EKF
    info = model.getInfo()

    #User control computations ...

    robot.setCommands(q_des, dq_des, kp, kd, tau_ff)
    robot.step()
```
## Installation
Follow through the steps here to [setup](docs/setup.md) the robot and Go2Py. 

## Examples 
A set of examples and tutorials are provided to get you up and running quickly:

- [Communicating with the robot](examples/00-robot-interface.ipynb) 
- [Interacting with ROS2](examples/01-ros2-tools.ipynb) 
- [Simulating with MuJoCo](examples/02-MuJoCo-sim.ipynb)
- [Robot dynamics and kinematics](examples/03-robot-dynamic-model.ipynb)
- [Finite state machine in low-level control](examples/04-FSM.ipynb) 
- [Walk These Ways RL Controller](examples/05-walk-these-ways-RL-controller.ipynb)
