# Setup of the project
1. Extract Uppaal Stratego in directory `~/Desktop/uppaalStratego/`
1. Create a build folder inside Simulation folder
1. Enter the build folder in terminal and run `cmake ..`
1. Run command `make`
1. Run experiments:

	1. Enter the folder `external/bin` folder in the build folder.

	1. Execute the shell script with the name of the experiment to run as an argument. E.g. `./run_a_scene.sh scene3`

## Requirements
### Minimum Requirements to run the simulations
1. Ubuntu 20.04
1. ROS Noetic
1. ROS Map Server
1. GCC ^9
1. LUA 5.2
1. CMake ^3.10

### Installation Guide
#### ROS Noetic 
1. Follow the installation guide: ([Installation Guide](https://wiki.ros.org/noetic/Installation/Ubuntu))
#### ROS Map Server
1. `sudo apt-get install ros-noetic-map-server`
#### GCC ^9
1. ```sudo apt install build-essential```
1. Check for successful install using `gcc -v`
#### LUA 5.2
1. Install lua 5.2 and 5.2-dev
```
sudo apt install lua5.2-dev
sudo apt install lua5.2
```

2. Check for successful install using ```lua -v```
#### CMake ^3.10
1. For installation of Cmake, we use the [Kitware APT Repository](https://apt.kitware.com/) for updating Cmake to 3.20.
```
sudo apt-get update
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ xenial main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
sudo apt-get update
sudo rm /usr/share/keyrings/kitware-archive-keyring.gpg
sudo apt-get install kitware-archive-keyring
sudo apt-get install cmake
```
2. Check for successful install using ```cmake --version```