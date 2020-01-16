# Scenario Runner for LGSVL Simulator

## backgroud 

this component is mainly derived from [carla/scenario_runner](https://github.com/carla-simulator/scenario_runner), please take a review first.

## How to run scenario

* start the simulator (server) binary, wait a few second when server is ready;
* go to the path of your pythonAPI and run a scenario with the following command:

```
python3 ./scenarios/scenario_runner.py -scenario NPCCutOff 

```

scenario_runner has provided a few scenarios. e.g. `NPCCutOff`, `NPCCutIn`, `FollowLeadingVehicle`, `ObstacleInFront`, but it's easily to customized more.
 

## FAQ:

1. In windows system, the default python package installation tool is pip, if you can not use `pip3` use `pip` instead.


