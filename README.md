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
