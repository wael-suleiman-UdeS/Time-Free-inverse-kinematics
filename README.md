# Algorithm Testing Repository
This repository is accompanying the paper "On Inverse Kinematics Solving: Space-Time Decoupling". It contains various algorithms and a script to test them.

The code can be used for benchmarking or educational purposes. More case studies to be added later on. 

## Implemented Methods in the Repository

- `Inverse_Kinematics_Planar_robot.m`: Contains the conventional algorithms from state-of-the-art.
- `Inverse_Kinematics_Planar_robot_T_Free.m`: Contains the proposed algorithm in the paper.
- `run_test_script.m`: Script to choose and run the test for either of the above files. 

## Prerequisites

Ensure you have Octave installed on your system. You can download it from [GNU Octave](https://www.gnu.org/software/octave/download.html). It has been tested with Octave Version 9.2.

## Running the Test Script

To test the algorithms, type in the command line in Octave:
run("run_test_script.m")
