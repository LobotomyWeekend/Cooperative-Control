# Cooperative Control of ASVs and UAVs

Software developed in the Autumn semester of 2017 for my Masters Project at the [Institute for Systems and Robotics](http://welcome.isr.tecnico.ulisboa.pt/) (ISR), Lisbon, Portugal. My thesis is entitled 'Cooperative Motion Control of Quadcopters and Autonomous Surface Vehicles'.

The final report for this project is avaliable [here](https://github.com/LobotomyWeekend/Cooperative-Control/blob/master/Documentation/final_report.pdf).

Contained within this repository are scripts for the simulation and control of a network of autonomous marine and aerial vehicles, which act in cooperation with one another. Specifically these vehicles are the semi-submersibe [MEDUSA](http://dsor.isr.ist.utl.pt/vehicles/medusa/) and a (more generalised) Quadcopter drone. 

The applications for this system are in data collection in a marine environment; geospatial surveys, tracking of wildlife, cinematography. Further work is currently being carried out at ISR and other European institutions in the field of autonomous marine and aerial vehicles working in cooperation.

![Mission scenario](https://github.com/LobotomyWeekend/Cooperative-Control/blob/master/Documentation/proposed_scenario.png)

## Getting Started

The files are broken down into "ASV Control" (Autonomous Surface Vehicle), "Quadcopter Control", "Coordinated Control", and "General".
"ASV Control" contains the code required to simulate and control the Medusa, including functions for vehicle dynamics and kinematics, and prepared simulations for various mission scenarios. "Quadcopter Control" is similar to ASV control, the structure of these two vehicle simulations has been kept similar where possible.

"General" contains data processing and plotting functions which are common to all vehicle types. "Coordinated Control" contains the cooperative controller, and prepared simulations which include more than one vehicle.

All software is available as Matlab code. The ASV also has a stable Simulink model, although no such model has been produced for the UAV.

Simulations for any combination of vehicles can be run, currently this is only stable up to around 4 vehicles [14/12/2017]. The general structure for these simulations is best seen in ```/Coordinatied Control/Matlab/test1UAV1ASV.m```. I have tried to keep the comments up to date for readability.

Any questions regarding the code, feel free to contact me on ```mattcole @ protonmail.com```.

### Prerequisites
* [Matlab R2017a](https://www.mathworks.com/downloads/web_downloads/get_release)
* [Simulink] (Optional)

### Installation

To run simulations all files should be added to the path, this can be done in the Matlab command line:
```
addpath('ASV Control','Coordinated Control','Quadcopter Control','General' -end);
```
To test all features, run a cooperative simulation involving the ASV and Quadcopter. You should recieve plots of a lawnmower trajectory, cross track error in time, and correction velocity in time.
```
test1UAV1ASV
```

## Acknowledgments

* [Prof. Antonio Pascoal](http://welcome.isr.tecnico.ulisboa.pt/author/antoniomanueldossantos/) - ISR Lisbon
* [Dr. Euan McGookin](https://www.gla.ac.uk/schools/engineering/staff/euanmcgookin/) - University of Glasgow
* [Rita Cunha](http://welcome.isr.tecnico.ulisboa.pt/author/ritamariamendesdealmeidacorreiada/) - ISR Lisbon
* [Staff & Students at ISR]
