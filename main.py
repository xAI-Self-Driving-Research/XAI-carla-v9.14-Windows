import carla
import random
import time
import json
import carla
import glob
import os
import sys
import datetime
import re
import socket
import textwrap
import argparse
import logging
from numpy import random
from carla import VehicleLightState as vls
import math
import datetime

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



# Helper functions for traffic generation and settings configuration
def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def get_actor_blueprints(world, filter, generation, no_bikes=False):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            if no_bikes:
                bps = [x for x in bps if x.get_attribute('number_of_wheels').as_int() != 2]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []
    
def get_ip(host):
    if host in ['localhost', '127.0.0.1']:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(('10.255.255.255', 1))
            host = sock.getsockname()[0]
        except RuntimeError:
            pass
        finally:
            sock.close()
    return host

def find_weather_presets():
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), x) for x in presets]

def list_options(client):
    maps = [m.replace('/Game/Carla/Maps/', '') for m in client.get_available_maps()]
    indent = 4 * ' '
    def wrap(text):
        return '\n'.join(textwrap.wrap(text, initial_indent=indent, subsequent_indent=indent))
    print('weather presets:\n')
    print(wrap(', '.join(x for _, x in find_weather_presets())) + '.\n')
    print('available maps:\n')
    print(wrap(', '.join(sorted(maps))) + '.\n')

def list_blueprints(world, bp_filter):
    blueprint_library = world.get_blueprint_library()
    blueprints = [bp.id for bp in blueprint_library.filter(bp_filter)]
    print('available blueprints (filter %r):\n' % bp_filter)
    for bp in sorted(blueprints):
        print('    ' + bp)
    print('')

def inspect(args, client):
    address = '%s:%d' % (get_ip(args.host), args.port)

    world = client.get_world()
    elapsed_time = world.get_snapshot().timestamp.elapsed_seconds
    elapsed_time = datetime.timedelta(seconds=int(elapsed_time))

    actors = world.get_actors()
    s = world.get_settings()

    weather = 'Custom'
    current_weather = world.get_weather()
    for preset, name in find_weather_presets():
        if current_weather == preset:
            weather = name

    if s.fixed_delta_seconds is None:
        frame_rate = 'variable'
    else:
        frame_rate = '%.2f ms (%d FPS)' % (
            1000.0 * s.fixed_delta_seconds,
            1.0 / s.fixed_delta_seconds)

    print('-' * 34)
    print('address:% 26s' % address)
    print('version:% 26s\n' % client.get_server_version())
    print('map:        % 22s' % world.get_map().name)
    print('weather:    % 22s\n' % weather)
    print('time:       % 22s\n' % elapsed_time)
    print('frame rate: % 22s' % frame_rate)
    print('rendering:  % 22s' % ('disabled' if s.no_rendering_mode else 'enabled'))
    print('sync mode:  % 22s\n' % ('disabled' if not s.synchronous_mode else 'enabled'))
    print('actors:     % 22d' % len(actors))
    print('  * spectator:% 20d' % len(actors.filter('spectator')))
    print('  * static:   % 20d' % len(actors.filter('static.*')))
    print('  * traffic:  % 20d' % len(actors.filter('traffic.*')))
    print('  * vehicles: % 20d' % len(actors.filter('vehicle.*')))
    print('  * walkers:  % 20d' % len(actors.filter('walker.*')))
    print('-' * 34)
# End of helper functions

class Client:  
    def __init__(self):
        self.idx = 0
        self.config = json.load((open('config.json')))
        self.delta_seconds = self.config["delta_seconds"]
        self.host = self.config["host"]
        self.port = self.config["port"]
        self.client = carla.Client(self.host, self.port)
        self.config_params = self.config['config_params']
        self.stopping_time = self.config['scenario_stopping_time_in_sec']
        self.speed = self.config["scenario_speed"]
        self.world = self.client.get_world()
        self.settings = self.world.get_settings()
        self.settings.synchronous_mode = True
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []
        self.all_actors = []
        self.sensors = []
        self.seed = self.config["seed"]
        
    # Getter and Setter functions
    def get_seed(self):
        return self.seed
    def get_idx(self):
        return self.idx
    def get_world(self):
        return self.world    
    def get_client(self):
        return self.client
    def get_map(self):
        return self.map
    def get_weather(self):
        return self.weather
    def get_spectator(self):
        return self.spectator
    def get_ego_vehicle(self):
        return self.ego_vehicle
    def get_traffic_config(self):
        return self.traffic_config
    def get_sensors(self):
        return self.sensors
    def get_settings(self):
        return self.settings
    
    def set_seed(self, seed):
        self.seed = seed
        random.seed(self.seed)
    def set_idx(self, idx):
        self.idx = idx
    def set_world(self, world):
        self.world = world
    def set_client(self, client):
        self.client = client
    def set_map(self, map):
        self.map = map
    def set_weather(self, weather):
        self.weather = weather
    def set_spectator(self, spectator):
        self.spectator = spectator
    def set_ego_vehicle(self, ego_vehicle):
        self.ego_vehicle = ego_vehicle
    def set_traffic_config(self, traffic_config):
        self.traffic_config = traffic_config
    def set_sensors(self, sensors):
        self.sensors = sensors
    def set_settings(self, settings):
        self.settings = settings
    # End of getters and setters
        
    def setup_settings(self):
        self.configure(['--host', self.host, '--port', str(self.port), '--map', self.map, '--weather', self.weather, '--delta-seconds', str(self.delta_seconds), *self.config_params],)
        
    def setup_traffic(self):
        if self.seed:
            self.generate_traffic([*self.traffic_config, '--seed', str(self.seed)])
        else:
            self.generate_traffic([*self.traffic_config])
        
    def setup_sensors(self):
        # Let's add now a "depth" camera attached to the vehicle. Note that the
        # transform we give here is now relative to the vehicle.
        for sensor_config in self.sensor_configs:
            sensor_bp = self.world.get_blueprint_library().find(sensor_config["model"])
            
            # @TODO Fix this hardcoded configuration of sensors
            # This only works for the sensor.camera.rgb model
            # Instead, we should be able to either pass any sensor model and
            # the attributes or hardcode the models that we want to use
            sensor_bp.set_attribute('image_size_x', str(sensor_config["image_size_x"]))
            sensor_bp.set_attribute('image_size_y', str(sensor_config["image_size_y"]))
            sensor_bp.set_attribute('shutter_speed', str(sensor_config["shutter_speed"]))
            sensor_bp.set_attribute('fov', str(sensor_config["fov"]))
            # --------------------------------------------------------------------
            
            sensor_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            self.sensor = self.world.spawn_actor(sensor_bp, sensor_transform, attach_to=self.ego_vehicle)
            self.sensors.append(self.sensor)
            print('created %s' % self.sensor.type_id)

            # Now we register the function that will be called each time the sensor
            # receives an image. In this example we are saving the image to disk
            # converting the pixels to gray-scale.
            #cc = carla.ColorConverter.LogarithmicDepth
            curr_time = datetime.datetime.now().strftime("%Y-%m-%d, %H-%M-%S")
            self.sensor.listen(lambda image: image.save_to_disk('out/%s/%06d.png' % (curr_time, image.frame)))
            
            #self.sensors.append(sensor)
    
    def setup_ego_vehicle(self):
        # In this tutorial script, we are going to add a vehicle to the simulation
        # and let it drive in autopilot. We will also create a camera attached to
        # that vehicle, and save all the images generated by the camera to disk.
        
        # Once we have a client we can retrieve the world that is currently
        # running.
        self.world = self.client.get_world()

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library_filtered = self.world.get_blueprint_library().filter(self.ego_vehicle_model)
        blueprint_library = [x for x in blueprint_library_filtered if (x.get_attribute('number_of_wheels').as_int() == 4 and not x.id.__contains__('vehicle.carlamotors.'))]
        if blueprint_library == []:
            blueprint_library = blueprint_library_filtered
        

        # Now let's filter all the blueprints of type 'vehicle' and choose one
        # at random.
        bp = random.choice(blueprint_library)

        # A blueprint contains the list of attributes that define a vehicle's
        # instance, we can read them and modify some of them. For instance,
        # let's randomize its color.
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        # Now we need to give an initial transform to the vehicle. We choose a
        # random transform from the list of recommended spawn points of the map.
        transform = random.choice(self.world.get_map().get_spawn_points())

        # So let's tell the world to spawn the vehicle.
        self.ego_vehicle = self.world.try_spawn_actor(bp, transform)

        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.
        print('created %s' % self.ego_vehicle.type_id)

        # Let's put the vehicle to drive around.
        self.ego_vehicle.set_autopilot(True)

        # Oh wait, I don't like the location we gave to the vehicle, I'm going
        # to move it a bit forward.
        location = self.ego_vehicle.get_location()
        location.x += 40
        self.ego_vehicle.set_location(location)
        print('moved vehicle to %s' % location)

    
    def setup_spectator(self):
        self.spectator = self.world.get_spectator()
        spectator_vector = self.ego_vehicle.get_transform().get_forward_vector()
        spectator_location = carla.Location(self.ego_vehicle.get_transform().location.x - self.spectate_ego_settings["spectator_distance"]*spectator_vector.x, 
                                            self.ego_vehicle.get_transform().location.y - self.spectate_ego_settings["spectator_distance"]*spectator_vector.y, 
                                            self.ego_vehicle.get_transform().location.z + self.spectate_ego_settings["spectator_height"])
        self.spectator.set_transform(carla.Transform(spectator_location, self.ego_vehicle.get_transform().rotation))
        
    def setup_scenario(self, scenario):
        print("Setting up scenario: " + scenario["name"] + " ...")
        self.name = scenario["name"]
        self.map = scenario["map"]
        self.weather = scenario["weather"]
        try:
            self.spectate_ego_settings = self.config["scenarios"][self.idx]["ego_vehicle_config"]["spectate_ego_settings"]
        except KeyError:
            self.spectate_ego_settings = None
        try:
            self.ego_vehicle_model = self.config["scenarios"][self.idx]["ego_vehicle_config"]["model"]
        except KeyError:
            self.ego_vehicle = None
        try:
            self.sensor_configs = self.config["scenarios"][self.idx]["ego_vehicle_config"]["sensor_configs"]
        except KeyError:
            self.sensor_configs = None
        try:
            self.traffic_config = self.config["scenarios"][self.idx]["traffic_config"]
        except KeyError:
            self.traffic_config = None

        
    def update_scenario(self):
        print("Loading scenario: " + self.name + " ...")

        if self.settings:
            self.setup_settings()
        #self.world.tick()
        #time.sleep(3)
        if self.ego_vehicle_model:
            self.setup_ego_vehicle()
        #self.world.tick()
        #time.sleep(3)
        if self.spectate_ego_settings:
            self.setup_spectator()
        if self.sensor_configs:
            self.setup_sensors()
        if self.traffic_config:
            self.setup_traffic()
    
    def clean(self):
        print('\ndestroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(self.all_id), 2):
            self.all_actors[i].stop()

        print('\ndestroying %d walkers' % len(self.walkers_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])

        time.sleep(0.5)
        
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []
        self.all_actors = []
        
        self.traffic_config = None
        self.ego_vehicle_model = None
        self.sensor_configs = None
        self.spectate_ego_settings = None
        for x in self.sensors:
            self.world.get_actor(x.id).destroy()
        try:
            self.ego_vehicle.destroy()
        except AttributeError:
            pass
        
        self.ego_vehicle = None
        self.spectator = None
        self.sensors = []
        
    # -------------------------------------------------------------------------------------------------------------
    def configure(self, args_list):
        argparser = argparse.ArgumentParser(
            description=__doc__)
        argparser.add_argument(
            '--host',
            metavar='H',
            default='localhost',
            help='IP of the host CARLA Simulator (default: localhost)')
        argparser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port of CARLA Simulator (default: 2000)')
        argparser.add_argument(
            '-d', '--default',
            action='store_true',
            help='set default settings')
        argparser.add_argument(
            '-m', '--map',
            help='load a new map, use --list to see available maps')
        argparser.add_argument(
            '-r', '--reload-map',
            action='store_true',
            help='reload current map')
        argparser.add_argument(
            '--delta-seconds',
            metavar='S',
            type=float,
            help='set fixed delta seconds, zero for variable frame rate')
        argparser.add_argument(
            '--fps',
            metavar='N',
            type=float,
            help='set fixed FPS, zero for variable FPS (similar to --delta-seconds)')
        argparser.add_argument(
            '--rendering',
            action='store_true',
            help='enable rendering')
        argparser.add_argument(
            '--no-rendering',
            action='store_true',
            help='disable rendering')
        argparser.add_argument(
            '--no-sync',
            action='store_true',
            help='disable synchronous mode')
        argparser.add_argument(
            '--weather',
            help='set weather preset, use --list to see available presets')
        argparser.add_argument(
            '-i', '--inspect',
            action='store_true',
            help='inspect simulation')
        argparser.add_argument(
            '-l', '--list',
            action='store_true',
            help='list available options')
        argparser.add_argument(
            '-b', '--list-blueprints',
            metavar='FILTER',
            help='list available blueprints matching FILTER (use \'*\' to list them all)')
        argparser.add_argument(
            '-x', '--xodr-path',
            metavar='XODR_FILE_PATH',
            help='load a new map with a minimum physical road representation of the provided OpenDRIVE')
        argparser.add_argument(
            '--osm-path',
            metavar='OSM_FILE_PATH',
            help='load a new map with a minimum physical road representation of the provided OpenStreetMaps')
        argparser.add_argument(
            '--tile-stream-distance',
            metavar='N',
            type=float,
            help='Set tile streaming distance (large maps only)')
        argparser.add_argument(
            '--actor-active-distance',
            metavar='N',
            type=float,
            help='Set actor active distance (large maps only)')
        
        args = argparser.parse_args(args_list)

        #self.client = carla.Client(args.host, args.port, worker_threads=1)
        #self.client.set_timeout(100.0)

        if args.default:
            args.rendering = True
            args.delta_seconds = 0.0
            args.weather = 'Default'
            args.no_sync = True

        if args.map is not None:
            print('load map %r.' % args.map)
            self.world = self.client.load_world(args.map)
        elif args.reload_map:
            print('reload map.')
            self.world = self.client.reload_world()
        elif args.xodr_path is not None:
            if os.path.exists(args.xodr_path):
                with open(args.xodr_path, encoding='utf-8') as od_file:
                    try:
                        data = od_file.read()
                    except OSError:
                        print('file could not be readed.')
                        sys.exit()
                print('load opendrive map %r.' % os.path.basename(args.xodr_path))
                vertex_distance = 2.0  # in meters
                max_road_length = 500.0 # in meters
                wall_height = 1.0      # in meters
                extra_width = 0.6      # in meters
                self.world = self.client.generate_opendrive_world(
                    data, carla.OpendriveGenerationParameters(
                        vertex_distance=vertex_distance,
                        max_road_length=max_road_length,
                        wall_height=wall_height,
                        additional_width=extra_width,
                        smooth_junctions=True,
                        enable_mesh_visibility=True))
            else:
                print('file not found.')
        elif args.osm_path is not None:
            if os.path.exists(args.osm_path):
                with open(args.osm_path, encoding='utf-8') as od_file:
                    try:
                        data = od_file.read()
                    except OSError:
                        print('file could not be readed.')
                        sys.exit()
                print('Converting OSM data to opendrive')
                xodr_data = carla.Osm2Odr.convert(data)
                print('load opendrive map.')
                vertex_distance = 2.0  # in meters
                max_road_length = 500.0 # in meters
                wall_height = 0.0      # in meters
                extra_width = 0.6      # in meters
                self.world = self.client.generate_opendrive_world(
                    xodr_data, carla.OpendriveGenerationParameters(
                        vertex_distance=vertex_distance,
                        max_road_length=max_road_length,
                        wall_height=wall_height,
                        additional_width=extra_width,
                        smooth_junctions=True,
                        enable_mesh_visibility=True))
            else:
                print('file not found.')

        else:
            self.world = self.client.get_world()
        self.settings = self.world.get_settings()
        if args.no_rendering:
            print('disable rendering.')
            self.settings.no_rendering_mode = True
        elif args.rendering:
            print('enable rendering.')
            self.settings.no_rendering_mode = False

        if args.no_sync:
            print('disable synchronous mode.')
            self.settings.synchronous_mode = False

        if args.delta_seconds is not None:
            self.settings.fixed_delta_seconds = args.delta_seconds
        elif args.fps is not None:
            self.settings.fixed_delta_seconds = (1.0 / args.fps) if args.fps > 0.0 else 0.0

        if args.delta_seconds is not None or args.fps is not None:
            if self.settings.fixed_delta_seconds > 0.0:
                print('set fixed frame rate %.2f milliseconds (%d FPS)' % (
                    1000.0 * self.settings.fixed_delta_seconds,
                    1.0 / self.settings.fixed_delta_seconds))
            else:
                print('set variable frame rate.')
                self.settings.fixed_delta_seconds = None

        #if args.tile_stream_distance is not None:
        #    self.settings.tile_stream_distance = args.tile_stream_distance
        #if args.actor_active_distance is not None:
        #    self.settings.actor_active_distance = args.actor_active_distance
        self.settings.max_substep_delta_time=0.005
        self.world.apply_settings(self.settings)

        if args.weather is not None:
            if not hasattr(carla.WeatherParameters, args.weather):
                print('ERROR: weather preset %r not found.' % args.weather)
            else:
                print('set weather preset %r.' % args.weather)
                self.world.set_weather(getattr(carla.WeatherParameters, args.weather))

        if args.inspect:
            inspect(args, self.client)
        if args.list:
            list_options(self.client)
        if args.list_blueprints:
            list_blueprints(self.world, args.list_blueprints)
        
    def generate_traffic(self, args_list):
        argparser = argparse.ArgumentParser(
            description=__doc__)
        argparser.add_argument(
            '--host',
            metavar='H',
            default='127.0.0.1',
            help='IP of the host server (default: 127.0.0.1)')
        argparser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        argparser.add_argument(
            '-n', '--number-of-vehicles',
            metavar='N',
            default=30,
            type=int,
            help='Number of vehicles (default: 30)')
        argparser.add_argument(
            '-w', '--number-of-walkers',
            metavar='W',
            default=10,
            type=int,
            help='Number of walkers (default: 10)')
        argparser.add_argument(
            '--safe',
            action='store_true',
            help='Avoid spawning vehicles prone to accidents')
        argparser.add_argument(
            '--filterv',
            metavar='PATTERN',
            default='vehicle.*',
            help='Filter vehicle model (default: "vehicle.*")')
        argparser.add_argument(
            '--generationv',
            metavar='G',
            default='All',
            help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
        argparser.add_argument(
            '--filterw',
            metavar='PATTERN',
            default='walker.pedestrian.*',
            help='Filter pedestrian type (default: "walker.pedestrian.*")')
        argparser.add_argument(
            '--generationw',
            metavar='G',
            default='2',
            help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
        argparser.add_argument(
            '--no-bikes',
            action='store_true',
            default=False)
        argparser.add_argument(
            '--tm-port',
            metavar='P',
            default=8000,
            type=int,
            help='Port to communicate with TM (default: 8000)')
        argparser.add_argument(
            '--asynch',
            action='store_true',
            help='Activate asynchronous mode execution')
        argparser.add_argument(
            '--hybrid',
            action='store_true',
            help='Activate hybrid mode for Traffic Manager')
        argparser.add_argument(
            '-s', '--seed',
            metavar='S',
            type=int,
            help='Set random device seed and deterministic mode for Traffic Manager')
        argparser.add_argument(
            '--seedw',
            metavar='S',
            default=0,
            type=int,
            help='Set the seed for pedestrians module')
        argparser.add_argument(
            '--car-lights-on',
            action='store_true',
            default=False,
            help='Enable automatic car light management')
        argparser.add_argument(
            '--hero',
            action='store_true',
            default=False,
            help='Set one of the vehicles as hero')
        argparser.add_argument(
            '--respawn',
            action='store_true',
            default=False,
            help='Automatically respawn dormant vehicles (only in large maps)')
        argparser.add_argument(
            '--no-rendering',
            action='store_true',
            default=False,
            help='Activate no rendering mode')

        args = argparser.parse_args(args_list)

        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)


        #self.client = carla.Client(args.host, args.port)
        #self.client.set_timeout(10.0)
        synchronous_master = False
        random.seed(args.seed if args.seed is not None else int(time.time()))

        
        self.world = self.client.get_world()

        traffic_manager = self.client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        self.settings = self.world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not self.settings.synchronous_mode:
                synchronous_master = True
                self.settings.synchronous_mode = True
                self.settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if args.no_rendering:
            self.settings.no_rendering_mode = True
        self.world.apply_settings(self.settings)

        blueprints = get_actor_blueprints(self.world, args.filterv, args.generationv, args.no_bikes)
        blueprintsWalkers = get_actor_blueprints(self.world, args.filterw, args.generationw)

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points
        
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = args.hero
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in self.client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                self.vehicles_list.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if args.car_lights_on:
            all_vehicle_actors = self.world.get_actors(self.vehicles_list)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        if args.seedw:
            self.world.set_pedestrians_seed(args.seedw)
            random.seed(args.seedw)
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(self.walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(self.walkers_list)):
            self.all_id.append(self.walkers_list[i]["con"])
            self.all_id.append(self.walkers_list[i]["id"])
        self.all_actors = self.world.get_actors(self.all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        #if args.asynch or not synchronous_master:
        #    world.wait_for_tick()
        #else:
        #self.world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(self.all_id), 2):
            # start walker
            self.all_actors[i].start()
            # set walk to random point
            self.all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # max speed
            self.all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(self.vehicles_list), len(self.walkers_list)))

        # Example of how to use Traffic Manager parameters
        traffic_manager.global_percentage_speed_difference(30.0)

        #while True:
        #    if not args.asynch and synchronous_master:
        #        world.tick()
        #    else:
        #        world.wait_for_tick()    
        
    # -------------------------------------------------------------------------------------------------------------

def main():
    # Connect to the client and retrieve the world object
    client = Client()
 
    try:
        if client.config == None:
            # Load a default scenario
            client.setup_scenario(scenario={"name": "Default", "map": "Town07", "weather": "DustStorm"})
            client.update_scenario()
            while True:
                client.get_world().tick()
                time.sleep(client.delta_seconds/client.speed)
        else:
            for run in range(0,client.config["num_runs"]):
                client.idx = 0
                
                if client.config["seed"] != []:
                    if len(client.config["seed"]) == client.config["num_runs"]:
                        client.set_seed(client.config["seed"][run])
                    else:
                        client.set_seed(client.config["seed"][0])
                print(client.seed)
                for scenario in client.config["scenarios"]:
                    # Setup and update the scenario
                    client.setup_scenario(scenario)
                    client.update_scenario()
                    #time.sleep(2)
                    # Run the scenario for the specified amount of time from the config
                    for i in range(0, int(client.stopping_time*1000), int(client.delta_seconds*1000)):
                        client.get_world().tick()
                        # Follow the ego vehicle if specified
                        if client.spectate_ego_settings:
                            client.setup_spectator()
                        time.sleep(client.delta_seconds/client.speed)
                        
                    # Cleanup any remaining actors before moving on to the next scenario
                    client.clean()
                    # Index the next scenario
                    client.idx += 1
    finally:
        # Clean the scenario if a failure occurs, user exits the program, or the program ends
        client.clean()
    
if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')