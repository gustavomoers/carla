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
from controllers import PIDLongitudinalController

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

    # Set synchronous mode settings
    # new_settings = world.get_settings()
    # new_settings.synchronous_mode = True
    # new_settings.fixed_delta_seconds = 0.08
    # world.apply_settings(new_settings) 

    # client.reload_world(False)



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



    #--------------
    #Spawn attached RGB camera
    #--------------
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



    ## SEGMENTATION CAMERA

    seg_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
    seg_bp.set_attribute("image_size_x",str(640))
    seg_bp.set_attribute("image_size_y",str(480))
    seg_bp.set_attribute("fov",str(110))
    seg_location = carla.Location(2,0,1)
    seg_rotation = carla.Rotation(0,0,0)
    seg_transform = carla.Transform(seg_location,seg_rotation)
    seg_cam = world.spawn_actor(seg_bp,seg_transform,attach_to=ego_vehicle)

    # add sensor to list of actors
    actor_list.append(seg_cam)


    # --------------
    # Spawn COLLISION sensor
    # --------------

    collision_hist = []
    def collision_data(event):
        print(event.other_actor)
        collision_hist.append(event)


    transform = carla.Transform(carla.Location(x=2.5, z=0.7))
    colsensor_bp = world.get_blueprint_library().find("sensor.other.collision")
    colsensor = world.spawn_actor(colsensor_bp, transform, attach_to=ego_vehicle)
    colsensor.listen(lambda event: collision_data(event))
    actor_list.append(colsensor)



    
    # --------------
    # Spawn lane invasion sensor
    # --------------

    lane_hist = []
    def lane_data(event):
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        print(text[0])
        lane_hist.append(event)


    lane_bp = world.get_blueprint_library().find("sensor.other.lane_invasion")
    lane = world.spawn_actor(lane_bp, transform, attach_to=ego_vehicle)
    lane.listen(lambda event: lane_data(event))
    actor_list.append(lane)


    #ego_vehicle.set_simulate_physics()
    #ego_vehicle.show_debug_telemetry()


   

    ## creating parking vehicles

    position = carla.Transform(ego_vehicle.get_transform().location + carla.Location(-3.5,15,0.5), 
                           carla.Rotation(0,90,0))
    parked_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), position)
    actor_list.append(parked_vehicle)


    # --------------
    # Spectator on vehicle view
    # --------------

    spectator = world.get_spectator()
    transform = parked_vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(y=-10,z=25.5),
    carla.Rotation(pitch=-90)))
 
    # --------------
    # Spawn pedestrian
    # --------------

    walkers_blueprints = world.get_blueprint_library().filter('*walker*')
    walker_loc = carla.Transform(parked_vehicle.get_transform().location + carla.Location(0,3,0.5), 
                               carla.Rotation(0,90,0))
    walker = world.try_spawn_actor(random.choice(walkers_blueprints), walker_loc)
    actor_list.append(walker)


 
    speed_controller = PIDLongitudinalController(ego_vehicle)


    # #ego_vehicle.set_autopilot(True)
    # ego_vehicle.enable_constant_velocity(carla.Vector3D(18.0,0,0))

    # try:

    #     walker.apply_control(carla.WalkerControl(carla.Vector3D(1, 0, 0.0), speed=1.2))

    #     # walker.apply_control(carla.WalkerControl(carla.Vector3D(random.uniform(1, 3), 
    #     #     random.uniform(0, 3), 0.0), speed=random.uniform(0.7, 1.2)))

    # except:
    #     pass   



    control = speed_controller.run_step(80)
    ego_vehicle.apply_control(control)





   # do something with camera sensor
    cc = carla.ColorConverter.LogarithmicDepth
    ego_cam.listen(lambda image: image.save_to_disk('_out/cam/%06d.png' % image.frame))
    #ego_cam.listen(lambda data: process_img(data))




    # do something with segmentation sensor
    cc = carla.ColorConverter.CityScapesPalette 
    seg_cam.listen(lambda image: image.save_to_disk('_out/seg/%06d.png' % image.frame, cc))
    #ego_cam.listen(lambda data: process_img(data))


    time.sleep(5)


except Exception as err:
    print(Exception, err)
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')

finally:
    
    for actor in actor_list:
        actor.destroy()
    print('done.')