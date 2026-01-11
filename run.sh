#!/bin/bash

ls -lah
echo "Starting Data Acquisition!"
echo "========================"
echo "Please be attentive to the robot :)"
echo "========================"

echo "Compiling the controller..."
g++ -std=c++17 CIC_controller.cpp include/examples_common.cpp -o controller.exe  -I /usr/include/eigen3 -I /usr/include -l franka
echo "Finished compiling."

echo "Executing the controller for VS, FS and FC tasks for 100 trajectories each."
./controller.exe VS 1
echo "Finished VS tasks."
echo "========================"
./controller.exe FS 1
echo "Finished FS tasks."
echo "========================"
./controller.exe FC 1
echo "Finished FC tasks."
echo "========================"
echo "Finished execution. Data is found under data/ folder."