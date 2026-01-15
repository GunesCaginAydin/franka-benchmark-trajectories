## franka-benchmark-trajectories
Real benchmarks for dynamical identification on Franka Emika Panda, available tasks are: vertical-spiral (VS), fixed-spiral (FS), fixed-circular (FC), object-push (PUSH) and object-pull (PULL). 

.cpp file is compiled for a Proportional-Integral + Joint Impedance control logic for unique random actuations of Franka Emika Panda.

.sh file is run for looping for looping over a randomized regime of tasks and task parameters.

To collect real training data, run the .sh Bash script and supervise the robot throughout the generation process. It is safe to assume that data collection for a single trajectory will take twice the time prescribed for the controller.

### System Requirements

<h4> Operating Systems </h4>
Ubuntu 20.04 LTS (Focal Fossa), <br>
Ubuntu 22.04 (Jammy Jellyfish), <br>
Ubuntu 24.04 (Noble Numbat)

<h4> Libraries </h4>
libfranka (follow build directives from https://github.com/frankarobotics/libfranka) <br>
Eigen3 (install binary directly from https://libeigen.gitlab.io/) 
<br>
<br>
!Eigen3 and libfranka header files has to be under the directory /usr/include/. For any other locations, update library references under run.sh. 
<br>
<br>
To verify that the libraries are correctly installed, copy and paste the below scripts into a terminal.
<br>
<br>

---

```
ls -l /usr/lib/libfranka.so
```
Expected output:
```
/usr/lib/libfranka.so -> libfranka.so.0.19.0
```

<br>

---
```
ls /usr/include/franka/
```
Expected output:
```
active_control_base.h      active_torque_control.h  control_tools.h
gripper_state.h            lowpass_filter.h         robot.h
...
```

<br>

---
```
dpkg -l | grep libfranka
```
Expected output:
```
ii  libfranka  0.19.0  amd64  libfranka - Franka Robotics C++ library
```

<br>

---
```
ls /usr/include/Eigen3
```
Expected output:
```
Dense.h                    Core.h
LU.h                       Jacobi.h
...
```
---
<h4> Hardware </h4>
Franka Robotics robot with FCI feature installed, Network connection to robot (1000BASE-T Ethernet recommended) <br> <br>
To compile the programs, the directory <b>must have</b> dedicated folders called <i>data</i> and <i>include</i>. <i>include</i> folder should have <i>examples_common.cpp</i> and <i>examples_common.h</i> which can be downloaded from https://github.com/frankarobotics/libfranka. 

### Connecting to Franka
Connection to Franka Emika Panda is established via an ethernet cable. Plugging the cable should suffice in controlling the robot.

### Compiling the cpp Program
The controller program is not to be directly compiled but rather indirectly compiled through the Bash script. <br>

Running the <i>run.sh</i> script starts the data collection protocol for various input regimes and times. For longer acquisitions, it is preferable to change the content of <i>run.sh</i> directly to customize the controller and task types.

### Controller

The currently available controller is a mix between a PI and a joint-velocity controller that randomizes over:

1) initial joint positions (very conservatively) 

2) task parameters (radius, period, direction)

3) controller parameters (translational and rotational proportional gain, integral gain)

### Data Collection

Data is logged into a .csv files, collectively found under the data folder in the working directory. Data collection follows joint input torques (1:7), cartesian position (8:10), cartesian orientation (11:14) and joint positions (15:21) consecutively for each row of data acquisition. 

### Troubleshooting

The trajectory of motion may flatter on some instances of parameter space, it is best always to be attentive to the robot.