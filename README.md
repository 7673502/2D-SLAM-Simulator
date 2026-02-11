# 2D SLAM Simulator

![example GIF of simulator in action](./slam_sim_example.gif)

This is a Rust implementation of a simulator for [EKF-SLAM](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf) and [FastSLAM](https://ai.stanford.edu/~koller/Papers/Montemerlo+al:AAAI02.pdf) with planned support for GraphSLAM.

## What is SLAM?

[SLAM](https://www.mathworks.com/discovery/slam.html) stands for "simulataneous localization and mapping." This is a problem in which an agent (in our case, a robot) must map its surroundings and determine its position in that map at the same time. This project visualizes a simulated robot along with the estimated poses given by two famous SLAM algorithms: EKF-SLAM and FastSLAM.


## Controls

- <kbd>&uarr;</kbd> <kbd>&darr;</kbd> <kbd>&larr;</kbd> <kbd>&larr;</kbd> movement
- <kbd>o</kbd> place obstruction
- <kbd>l</kbd> place landmark
- <kbd>esc</kbd> enter/exit visibility settings

Hit the setting button in the top left to choose which algorithms' pose and landmark estimates are visible.

## Project Structure

TODO

## Sources
[1] [Simultaneous localization and mapping with the extended Kalman filter](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf)
[2] [FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem](https://ai.stanford.edu/~koller/Papers/Montemerlo+al:AAAI02.pdf)

## License

[MIT License](LICENSE).

