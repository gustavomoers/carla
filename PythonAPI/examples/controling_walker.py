import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import random
import time
import numpy as np
import cv2



actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
  
    walker_bp = world.get_blueprint_library().filter("walker.pedestrian.*")
    controller_bp = world.get_blueprint_library().find('controller.ai.walker')


    actors = []

    for i in range(20000):

        trans = carla.Transform()
        trans.location = world.get_random_location_from_navigation()
        trans.location.z +=1


        walker = random.choice(walker_bp)
        actor = world.spawn_actor(walker, trans)
        world.wait_for_tick()


        # spectator = world.get_spectator()
        # transform = actor.get_transform()
        # spectator.set_transform(carla.Transform(transform.location + carla.Location(y=-10,z=2.5),
        # carla.Rotation(yaw=90)))



        controller = world.spawn_actor(controller_bp, carla.Transform(), actor)
        world.wait_for_tick()


        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())




        actors.append(actor)
        actors.append(controller)


    while (1):
        time.sleep(0.1)



finally:
    client.apply_batch([carla.command.DestroyActor(x) for x in actors])