# Algorithm Testing Repository
This repository is accompanying the paper "On Inverse Kinematics Solving: Space-Time Decoupling". It contains various algorithms and a script to test them.

The code can be used for benchmarking or educational purposes. More case studies to be added later on. 

## Implemented Methods in the Repository

- `Inverse_Kinematics_Planar_robot.m`: Contains the conventional algorithms from state-of-the-art.
- `Inverse_Kinematics_Planar_robot_T_Free.m`: Contains the proposed algorithm in the paper.
- `run_test_script.m`: Script to choose and run the test for either of the above files. 

## Prerequisites

### Octave
Ensure you have GNU Octave installed on your system. You can download it from the official [GNU Octave website](https://www.gnu.org/software/octave/download.html). This project has been tested with Octave version 9.2.

### MATLAB
Alternatively, if you have MATLAB, you can use it to run the code in the folder "MATLAB_Version". Please note the following:
- The "Optimization Toolbox" is required.
- The code has been tested with MATLAB versions 2023b and 2024a.

## Running the Test Script

To test the algorithms, type in the command line in Octave or MATLAB:
run("run_test_script.m")
