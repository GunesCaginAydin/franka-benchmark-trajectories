## franka-benchmark-trajectories
Real benchmarks for dynamical identification on Franka Emika Panda, available tasks are: vertical-spiral (VS), fixed-spiral (FS), fixed-circular (FC), object-push (PUSH) and object-pull (PULL). 

.cpp files provide proportional-derivative-integral (PID) and cartesian-impedance (CIC) control logic for uniquely random actuations of Franka Emika Panda.

.sh files provide looping for data collection over a randomized regime of tasks and task parameters.

To collect real training data, run PID.sh or CIC.sh and supervise the robot throughout the generation process. It is safe to assume that data collection for a single trajectory will take twice the time prescribed for the controller.

### Connecting to Franka

### Compiling the cpp Controllers

### Sequential Actuation and Data Collection

### PID Controller

PID controller randomizes over:

1) x

2) x

### CIC Controller

CIC controller randomizes over:

1) x

2) x

### Data Collection

Data is logged into a .csv files, colelctively found under the data folder. Data collection follows joint input torques (1:7), cartesian position (8:10), cartesian orientation (11:14) and joint positions (15:21) consecutively for each row of data. 

### Troubleshooting

The trajectory of motion may flatter on some instances of parameter space, it is best always to be attentive to the robot.