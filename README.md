# Cooperative Control of ASVs and UAVs

For my final year project at the Institute for Systems and Robotics in Lisbon, I am designing a control system for an array of autonomous marine and aerial vehicles moving in cooperation with one another for data collection missions.

## Getting Started

The files are broken down into ASV (Autonomous Surface Vehicle), UAV (Unmanned Aerial Vehicle), and Coordinated Control.
Simulations for each vehicle can be run individually, some specific information can be found in the comments. As of [30/10/2017] the Cooperative motion component is still a work-in-progress.

### Prerequisites

All that is needed are full versions of both:

* [Matlab R2017a]
* [Simulink]

### Installing

To run simulations the files must be added to the path, this can be done in the command line

```
addpath('ASV Control','Coordinated Control','UAV Control' -end);
```

## Acknowledgments

* [Prof. Antonio Pascoal]
* [Rita Cunha]
* [Staff & Students at ISR]
