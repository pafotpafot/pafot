# PAFOT: A Position-Based Approach for Finding Optimal Tests of Autonomous Vehicles

This project contains the source code for PAFOT. It uses genetic algorithms and a 9-positions approach to generate test scenarios likely to reveal collisions on simulated tests of Automated Driving Systems (ADS).

## Requisites:

Apart from a computer with enough processing and GPU power to run the CARLA Simulator, you will need:

- Python3
- CARLA Simulator 0.9.13
- CARLA Map Town06
- loguru
- numpy

Please note this project was developped on Ubuntu 18.04LTS. While it might work on Windows, the implementation has not been tested in that operative system.


## How to run?

1. Run CARLA either on a container, by following instructions on the official site, or by building it from source.
2. Execute `run.sh`. It will run the experiments for 10 times.
3. Results will be saved to `/Output` in CSV format. You'd might need to create this directory if it isn't already created.
