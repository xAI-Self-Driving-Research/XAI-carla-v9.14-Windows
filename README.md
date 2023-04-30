# XAI-carla-v9.14-Windows

## Git Clone with SSH
We advice that you git clone with SSH since we ran into some issues just using HTTPS. 

1. Before cloning the repo, make sure that your environment is setup to use SSH. Refer to this article for additional details: https://www.toolsqa.com/git/clone-repository-using-ssh/
2. Copy the SSH url for the repo
3. In your terminal, enter: ```git clone <url>```

## How to run the code
1. Open the project in Visual Studio
2. Source the environment by running the following command in the terminal from the root directory: ```. ./env/Scripts/activate```. If you don't have a virtual environment, you can create one by running the following command: ```python3 -3.8 -m venv env```. If you don't have python 3.8, you can download it here: https://www.python.org/downloads/release/python-3810/
3. Run the carla server by running the following command wherever Carla is installed: ```./CarlaUE4.exe```
4. Modify the ```config.json``` file to reflect the desired configurations and scenarios. You can view the ```options.txt``` file for a list of a few options.
5. Run the code by running the following command in the terminal from the root directory: ```python3 main.py```

## Config File
The config file is used to specify the desired configurations and scenarios. 
- It's important to note that not all fields are necessary to include for each scenario. It's designed to use default options if a field is not specified. For example, any list of strings can be left empty if you don't want to specify any options. These will be filled with default options. ex: "traffic_config": ["--number-of-vehicles", "15", "--number-of-walkers", "20","--safe","--no-bikes"] omitted, which will spawn 30 vehicles, 10 pedestrians, and include bikes.
- The only necessary scenario parameters are name, map, and weather.
- Any parameters outside the ```scenarios``` block are necessary for the program to run.
- If you want the main carla server to follow your ego vehicle, you must specify the ```spectator_ego_settings``` block with the ```spectator_height``` over the vehicle and the ```spectator_distance``` from behind the vehicle. Otherwise, the spectator will be initated at the origin with manual control enabled.
- If you want a sensor to be placed on the vehicle for collecting data, you must specify the ```sensor_configs``` block with the desired sensor and its parameters. Otherwise, no sensors will be placed on the vehicle. If you want multiple sensors, you can specify multiple sensor config blocks. ex: 
```
[
        {
            "model": "sensor.camera.rgb",
            "image_size_x": 640,
            "image_size_y": 480,
            "shutter_speed": 200,
            "fov": 105
        },
        {
            "model": "sensor.camera.depth",
            "image_size_x": 640,
            "image_size_y": 480,
            "shutter_speed": 200,
            "fov": 105
        }
]
```
- Any list of strings surrounded by brackets is a list of options for the scenario. If you want the parameters for these options, run either ```python3 PythonAPI/util/config.py --help``` or ```python3 PythonAPI/examples/generate_traffic.py --help``` to see the options for the functions these options are passed to.
- For the more global settins,
  - delta_seconds is the time between each frame in seconds. This is used to calculate the time between each frame in milliseconds.
  - scenario_speed is the speed multiplier of all agents in the scenario.
  - scenario_stopping_time_in_sec is the time in seconds that the scenario will run for.
  - seed is the seed to be used for the random number generator. If you don't want this, set the value to 0
- All sensor readings will be saved to an ```out``` directory from root. From there, each scenario will have its own directory with the sensor frames saved in a subdirectory named after the time the scenario started. The sensor readings will be saved in a png file with the frame number as the name. ex: ```out/2023-04-30, 03-00-35/1282696.png```
- If you want to run your scenarios multiple times, set ```num_runs``` to the number of times you want to run the scenarios. Then, if you want a different seed for each run, set ```seed``` to an array containing each seed you want to use for each scenario. Otherwise, if you want 1 seed for all scenarios, set ```seed``` to a single value. If you don't want to use a seed, set ```seed``` to 0.
## Features to implement in CARLA
We want to be able to modify certain features in CARLA to reflect a variety of scenarios. These features are split up between static and dynamic. 

### Static Features
1. Roadway
2. Weather/Environment
3. Vehicle

### Dynamic Features
1. Aggressive Behavior
2. Decision
3. Speed
4. Traffic
5. Vision
6. Movement Prior to Crash
7. Pre-crash event
8. Attempted Avoidance Maneuvers
9. Stability of Vehicle
10. Location on Trafficway
