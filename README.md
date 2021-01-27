![autoware](https://user-images.githubusercontent.com/8327598/69472442-cca50b00-0ded-11ea-9da0-9e2302aa1061.png)
TestWare
======
> **WARNING**: This repository is provided solely for simulation research purposes. It should not be used to drive cars. 

TestWare is a minimally invasive fork of the [Autoware Architecture Proposal](https://github.com/tier4/AutowareArchitectureProposal.proj) (AWAP), from [commit](https://github.com/tier4/AutowareArchitectureProposal.proj/tree/6804847385ec3b683e124ade50d8e03174c08f55). AWAP is a prototype system targeted at SAE Level 4 autonomous driving capabilities. We provide and OpenAI Gym like interface for interacting with the AWAP implementation using the CARLA simulator. 

### Utilities for CARLA Integration

**Sensors:** We include utilities which read the sensor configuation and instantiate the specified sensors and transforms on the CARLA ego vehicle. In addition we provide methods for converting the measurements of these sensors to ROS messages and publishing the serialized data to Autoware in a synchronous manner (along with a clock which accurately reflects the state of CARLA's physics engine). See ``/testware/utils/CarlaUtils.py`` for the implementation of sensor instantiation features and ``/testware/utils/AutowareUtils.py`` for measurement publishing. 

**Control:** 
We use the vehicle model to predict the yaw rate and slip angle of the ego vehicle such that Autoware control inputs can be applied as velocity and angular velocity commands. See ``/testware/utils/vehicle_model.py`` for an implementation of the dynamic bicycle model from "The Science of Vehicle Dynamics (2014), M. Guiggiani". Additionally, ``/testware/testware.py`` and ``/testware/utils/CarlaUtils.py`` contain methods for stepping the simulator and computing the control inputs. 

**Resets:** We enable resetting the AWAP stack and initilizing the pose and goal pose after the conclusion of each rollout. See ``/trustware/trustware.py`` for details and ``/demo/demo.py`` for an example. In addition we provide ``/rosutils/simple_planning_simulator_core.cpp`` to handle the rest on the AWAP side.

**Synchronous Execution:** We provide a clock signal to ROS which matches the passage of time in CARLA, and ensure lockstep execution of sensor observations, then planning and control functions. See ``/demo/demo.py`` for an example. 

**Launch Files and Maps:** We include a sample map and scenario as well as launch files to start AWAP ros nodes. See ``/rosutils/carla_simultor.launch`` for details and related files. 

### Modifications to the Autoware Architecture Proposal Source
We have made minimal changes to the AWAP source code, all changes are contained in ``/testware/rosutils``. Note that the files contained in this folder are copied into the ``/testware/AutowareArchitectureProposal`` directory structure when the Docker image is built. 

# Dependencies
We have tested the code on Ubuntu 16.04, 18.04, and 20.04 machines. Some features of the Autoware Architecture Proposal (and thus TestWare) require CUDA; therefore, we are skeptical that it will be possible to use OSX or Windows machines, even with Docker. We do not provide any support for non-Ubuntu operating systems.

## Docker
Install [Docker for Ubuntu](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/). Make sure to `sudo usermod -aG docker your-user` and then you do not need to run below docker scripts as `sudo`. You will need to make sure that you have Docker version 19 or greater. 

## CARLA
Get CARLA from https://github.com/carla-simulator/carla/releases. We have validated TestWare with CARLA releases 0.9.10.1. Note that for 0.9.9.x and above, visuals and weather look best using Vulkan instead of OpenGL when running CARLA. 

## NVIDIA and CUDA
You need an NVIDIA graphics card with driver version 440 or greater. You will need a card with at least 4 GB of memory to support running CARLA and Autoware perception components simultaneously. 

# Setup
0. Run `./build_docker.sh`

# Running the demo
0. Start CARLA on port 2010.
	
	An example of this command would be `DISPLAY= ./CarlaUE4.sh -opengl -quality-level=Epic -carla-world-port=2010` from a directory that contains the binary `CarlaUE4.sh`
1. Run `./run_docker.sh`
2. Run `source /AutowareArchitectureProposal/devel/setup.bash` inside the docker
3. Run `python demo.py`

Licensing
------

TestWare is released under the Apache V2 license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Trustworthy AI, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

