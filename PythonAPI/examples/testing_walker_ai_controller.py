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

IM_WIDTH = 640
IM_HEIGHT = 480


def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    i3 = i2[:, :, :3]
    cv2.imshow("", i3)
    cv2.waitKey(1)
    return i3/255.0


actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    #world.unload_map_layer(carla.MapLayer.Buildings)

    # --------------
    # Spawn ego vehicle
    # --------------

    # Get the blueprint library and filter for the vehicle blueprints
    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')

    # Get the map's spawn points
    #spawn_points = world.get_map().get_spawn_points()

    ego_location = carla.Transform(carla.Location(x=-131.6, y=-70.5, z=0.500000), 
                                   carla.Rotation(pitch=0.000000, yaw=90, roll=0.000000))

    ego_vehicle = world.spawn_actor(vehicle_blueprints.filter('model3')[0], ego_location)
    

    #vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    #vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.

    actor_list.append(ego_vehicle)
    


    # --------------
    # Spawn attached RGB camera
    # --------------
    cam_bp = None
    cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_bp.set_attribute("image_size_x",str(640))
    cam_bp.set_attribute("image_size_y",str(480))
    cam_bp.set_attribute("fov",str(110))
    cam_location = carla.Location(2,0,1)
    cam_rotation = carla.Rotation(0,0,0)
    cam_transform = carla.Transform(cam_location,cam_rotation)
    ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle)

    # add sensor to list of actors
    actor_list.append(ego_cam)

    # do something with this sensor
    cc = carla.ColorConverter.LogarithmicDepth
    ego_cam.listen(lambda image: image.save_to_disk('_out/%06d.png' % image.frame))
    

    ## creating parking vehicles

    position = carla.Transform(ego_vehicle.get_transform().location + carla.Location(-3.5,40,0.5), 
                           carla.Rotation(0,90,0))
    parked_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), position)
    actor_list.append(parked_vehicle)
    

    # --------------
    # Spectator on vehicle view
    # --------------

    spectator = world.get_spectator()
    transform = ego_vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(y=-10,z=2.5),
    carla.Rotation(yaw=90)))
    


    # --------------
    # Spawn pedestrian
    # --------------

    walkers_blueprints = world.get_blueprint_library().filter('*walker*')
    walker_loc = carla.Transform(parked_vehicle.get_transform().location + carla.Location(-2.5,3,0.5), 
                               carla.Rotation(0,90,0))
    walker = world.spawn_actor(random.choice(walkers_blueprints), walker_loc)
    world.wait_for_tick()

    ## Walker controller
    controller_bp = world.get_blueprint_library().find('controller.ai.walker')

    controller = world.spawn_actor(controller_bp, carla.Transform(), walker)
    #world.wait_for_tick()


    #controller.start()
    #controller.go_to_location(world.get_random_location_from_navigation())


    actor_list.append(walker)
    actor_list.append(controller)

    ego_vehicle.enable_constant_velocity(carla.Vector3D(18.5,0,0))
    time.sleep(1.5)
    walker.apply_control(carla.WalkerControl(carla.Vector3D(5.0, 3.0, 0.0), speed=1.5))



    time.sleep(30)

finally:
    print('destroying actors')
    controller.stop()
    for actor in actor_list:
        actor.destroy()
    print('done.')