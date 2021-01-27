TestWare
======
> **WARNING**: This repository is provided solely for simulation research purposes. It should not be used to drive cars. 

TestWare is a minimally invasive fork of the Autoware Architecture Proposal, a prototype system targeted at SAE Level 4 autonomous driving capabilities. 

### Utilities for CARLA
**Sensors:** We include utilities which read the sensor configuation and instantiate the specified sensors and transforms on the CARLA ego vehicle. See ``CarlaUtils.py`` for details. In addition we provide methods for converting the measurements of these sensors to ROS messages and publishing the serialized data to Autoware in a synchronous manner (along with a clock which accurately reflects the state of CARLA's physics engine). 

**Control:** 

### Resetting the environment


### Synchronous execution

### Other notes
Docker version, NVIDIA drivers, Carla versions, egg file, other maps

## Citing

If you find this code useful in your work, please consider citing our [paper](https://arxiv.org/abs/1912.03618):

```
@article{norden2019efficient,
  title={Efficient black-box assessment of autonomous vehicle safety},
  author={Norden, Justin and O'Kelly, Matthew and Sinha, Aman},
  journal={arXiv preprint arXiv:1912.03618},
  year={2019}
}
```

# Dependencies
We have tested the code on Ubuntu 16.04, 18.04, and 20.04 machines. Some features of the Autoware Architecture Proposal (and thus TestWare) require CUDA; therefore, we are skeptical that it will be possible to use OSX or Windows machines, even with Docker. We do not provide any support for non-Ubuntu operating systems.

## Python 3.7
Depending on your Linux distro this should already be preinstalled. This dependency is due to the requirements of CARLA 0.9.9 rather than any aspect of TestPilot.

## Docker
Install [Docker for Ubuntu](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/). Make sure to `sudo usermod -aG docker your-user` and then you do not need to run below docker scripts as `sudo`

## Python Libraries
Dependencies are detailed in requirements.txt
* `colorama 0.4.3`
* `cython 0.29.21`
* `numba 0.51.2`
* `numpy 1.19.2`
* `opencv-python 4.4.0.42`
* `pygame 1.9.6`
* `pyzmq 19.0.2`

Newer versions of these libraries likely work but have not been tested.

## CARLA (to run the demo)
Get CARLA from https://github.com/carla-simulator/carla/releases. We have validated TestWare with CARLA releases 0.9.10.1. Note that for 0.9.9.x and above, visuals and weather look best using Vulkan instead of OpenGL when running CARLA. 


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

TestWare is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Trustworthy AI, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

