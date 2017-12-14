# Cooperative Control of ASVs and UAVs

Software developed for my Masters Project at the Institute for Systems and Robotics (ISR), Lisbon, Portugal.

Contained is a control system (in development) for a network of autonomous marine and aerial vehicles, which act in cooperation with one another. Specifically these vehicles are the [MEDUSA](http://dsor.isr.ist.utl.pt/vehicles/medusa/) and a (more general) Quadcopter drone. 

The applications for this system are in data collection in a marine environment; geospatial surveys, tracking of wildlife, cinematography. Further work is currently being carried out at ISR and other European institutions in the field of autonomous marine and aerial vehicles working in cooperation.

## Getting Started

The files are broken down into "ASV Control" (Autonomous Surface Vehicle), "UAV Control" (Unmanned Aerial Vehicle), "Coordinated Control", and "General". The first three are of particular note; they contain the structures for simulations, as well as the control algorithms, and system dynamics involved. The latter contains some data processing and plotting function which are of general use. 

All software is available as Matlab code. The ASV also has a stable Simulink model, although no such model has been produces for the UAV.

Simulations for any combination of vehicles can be run, currently this is only stable up to around 4 vehicles [14/12/2017]. The general structure for these simulations is best seen in ```/Coordinatied Control/Matlab/test1UAV1ASV.m```. I have tried to keep the comments useful and up to date but this is still a work in progress.

### Prerequisites
* [Matlab R2017a]
* [Simulink] (Optional)

### Installing

To run simulations all files should be added to the path, this can be done in the Matlab command line:
```
addpath('ASV Control','Coordinated Control','UAV Control','General' -end);
```

## Acknowledgments

* [Prof. Antonio Pascoal]
* [Rita Cunha]
* [Staff & Students at ISR]
