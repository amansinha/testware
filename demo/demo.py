# -*- coding: utf-8 -*-
'''
Copyright (c) 2021, Trustworthy AI, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1.  Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2.  Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3.  Neither the name of the copyright holder(s) nor the names of any contributors
may be used to endorse or promote products derived from this software without
specific prior written permission. No license is granted to the trademarks of
the copyright holders even if such marks are included in this software.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''
###############################################################################
# demo.py
# Author: Aman Sinha
# Purpose: example of running Testware agent
# Notes:
#   to make sure my editor saves in utf-8 here is a nice character: é
###############################################################################

from __future__ import print_function
import argparse
import glob
import numpy as np
import os
import sys
import time


try:
    sys.path.append(glob.glob('../testware/utils/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

sys.path.append('../')
import testware
import testware.utils.Utils as Utils
import testware.utils.CarlaUtils as CarlaUtils

TOWN = 'Town04'
FPS = 20
STARTPOSE_TOWN04 = carla.Transform(carla.Location(x=-6.221527, y=-168.1685, z=1.200000),
                                   carla.Rotation(pitch=0.000000, yaw=89.775124, roll=0.000000))
WEATHER_PRESETS = [carla.WeatherParameters.ClearNoon,
                   carla.WeatherParameters.CloudyNoon,
                   carla.WeatherParameters.WetNoon,
                   carla.WeatherParameters.WetCloudyNoon,
                   carla.WeatherParameters.SoftRainNoon,
                   carla.WeatherParameters.MidRainyNoon,
                   carla.WeatherParameters.HardRainNoon,
                   carla.WeatherParameters.ClearSunset,
                   carla.WeatherParameters.CloudySunset,
                   carla.WeatherParameters.WetSunset,
                   carla.WeatherParameters.WetCloudySunset,
                   carla.WeatherParameters.SoftRainSunset,
                   carla.WeatherParameters.MidRainSunset,
                   carla.WeatherParameters.HardRainSunset]
RUNNING_STEPS = 200


def initialize_environment(allcams, carla_ip, carla_port):
    agent = testware.Testware(allcams)
    done = False
    while not done:
        try:
            client = CarlaUtils.carla_connect(carla_ip, carla_port, TOWN)
            waypoints = CarlaUtils.get_waypoints(client, STARTPOSE_TOWN04)
            done = True
        except RuntimeError:
            time.sleep(1)
    return client, agent, waypoints


def simulate(client, agent, waypoints, seed, draw=False):
    def cleanup():
        for sensor in sensor_list:
            sensor.destroy()
        for actor in actor_list:
            actor.destroy()

    np.random.seed(seed)
    world = client.get_world()

    # set weather
    world.set_weather(WEATHER_PRESETS[np.random.randint(len(WEATHER_PRESETS))])

    # start agent, initialize pose, and add sensors
    agent.start()
    agent.setPose(waypoints, client, world)
    actor_list = [agent.carla_actor]
    sensor_list = agent.setSensors(draw, client, world)

    # run the simulation
    with CarlaUtils.CarlaSyncMode(world, *sensor_list, fps=FPS) as sync_mode:
        for cnt in range(RUNNING_STEPS):
            sensor_outs = sync_mode.tick(timeout=60.0)
            # snapshot = sensor_outs[0]
            num_cams = 6 if agent.allcams else 1
            camera_outs = sensor_outs[1:1 + num_cams]
            lidar_outs = sensor_outs[1 + num_cams:1 + num_cams + 3]
            if draw:
                image_hover = sensor_outs[-1]
            if cnt == 0:
                CarlaUtils.initialize_velocities_batch(actor_list, [agent.v0], client)
            control_actions = agent.step(cnt, FPS, camera_outs, lidar_outs)
            CarlaUtils.set_controls(actor_list, [control_actions], client)
            if draw:
                Utils.draw_image(DISPLAY, image_hover)
                pygame.display.flip()

        # stop agent
        agent.stop(FPS)
        # miscellaneous other cleanup for stopping the simulation
        cleanup()
    if any((sensor.is_alive for sensor in sensor_list)) or any((actor.is_alive for actor in actor_list)):
        client.reload_world()
    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--draw', type=int, default=1, help='draw images to screen')
    parser.add_argument('--allcams', type=int, default=0, help='Use all cameras (if 1) or only front camera (0, default)')
    parser.add_argument('--carla_ip', default='localhost', help='IP address for CARLA')
    parser.add_argument('--carla_port', type=int, default=2010, help='Port for CARLA')
    parser.add_argument('--runs', type=int, default=2, help='Number of simulations to run')
    args = parser.parse_args()
    args.draw = bool(args.draw)
    args.allcams = bool(args.allcams)
    if args.draw:
        global DISPLAY
        import pygame
        pygame.init()
        DISPLAY = pygame.display.set_mode((1200, 1200), pygame.HWSURFACE | pygame.DOUBLEBUF)
    client, agent, waypoints = initialize_environment(allcams=args.allcams,
                                                      carla_ip=args.carla_ip,
                                                      carla_port=args.carla_port)
    for i in range(args.runs):
        print('iteration', i)
        simulate(client=client, agent=agent, waypoints=waypoints, seed=i, draw=args.draw)
    agent.kill()
