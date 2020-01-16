# Getting Started Tutorial

Welcome to the pythonapi for LG simulator! This tutorial provides the basic steps
for getting started using the pythonapi for LG simulator.

Download the latest release from our shared drive to a folder of your choice.

## Installing prerequisites
The current version is designed to be used with Ubuntu 16.04, windows, Python 3.5 or higher.

Install python if it is not installed in your computer,(suggest install python in conda, but it is not mandatory) and then execute:
```
  sudo apt-get update #just for linux system
  pip3 install setuptools websockets
  pip3 install py_trees==0.8.3 
  pip3 install numpy scipy matplotlib
  pip3 install --user -e .
  pip install xmlschema
```
Note: 
* If pip3 is not supported, use pip instead
* py-trees newer than v0.8 is *NOT* supported.

Include pythonapi to the python path(i.e. to *PYTHONPATH* environment variable), if this variable is not exist create it first.
* In windows system:

![Add pythonapi directory into PYTHONPATH](./images/pythonpath_environment_variable.PNG)

* In linux system:
```Bash
export LG_PYTHONAPI_ROOT=/path/to/your/lg/simulator/pythonapi/installation
cat "export PYTHONPATH=$PYTHONPATH:${LG_PYTHONAPI_ROOT}" >> ~/.bashrc
source ~/.bashrc
```
NOTE: ${LG_PYTHONAPI_ROOT} needs to be replaced with your pythonapi installation directory.


## Running the following example
First of all, you need to start the LG simulator binary, wait a few second when server is ready, you could see the following window.

![lg sim start page](./images/lgSimStartPage.PNG)

You need to set the position of ego vehicle and/or npc vehicle. A mapgui tool is used to select positions on map. Follow the instructions [here](mapgui.md) to select the positions.
After setting the positions you can continue to run the following example.

## Run a test scenario

There are already been some test scenarios in this pythonapi repo. The following picture shows the test scenarios and trigger methods supported.
![supported scenarios](./images/supportedTestScenarios.png)

Take **npc_cut_in** as an example. You need to set the positions of ego vehicle and npcs to proper positions(npc is in front of ego vehicle in right neighouring lane).
Open a terminal and navigate to pythonapi directory, and then execute:
```bash
python scenario_runner.py --scenario NpcCutIn
```
You can change **NpcCutIn** with any one of the following scenarios to run that scenario:
* FollowLeadingVehicle
* NpcCutOff
* NpcCutIn
* ObstacleInFront
* NpcAbnormalSpeed
* NpcSpeedProfile
* PedestrainStillInFront
* NpcTrafficJam

NOTE: If you require help or want to explore other command line parameters, start the scenario
runner as follows:
```
python scenario_runner.py --help
```

Run simulink model that contains the ADS stack in our example the **UDPSender.slx**. 

