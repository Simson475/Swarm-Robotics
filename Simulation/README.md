# Setup of the project
1. Extract Uppaal Stratego in directory `~/Desktop/uppaalStratego/`
1. Create a build folder inside Simulation folder
1. Enter the build folder in terminal and run `cmake ..`
1. Run command `make`
1. Configure and run server:
	
	1. Enter the folder `server` in the build folder. Now one is capable of running the server with command `./Server 20009`.
    
    1. If needed, the hard-coded PORT value in [connector.cpp](https://github.com/DEIS-Tools/Swarm-Robotics/blob/master/Simulation/connection/connector.cpp) can be changed. The new value will be the argument when execution the server.
		
1. Run experiments:

	1. Enter the folder `external/bin` folder in the build folder. 
	
	1. Execute the shell script with the name of the experiment to run as an argument. E.g. `./run_a_scene.sh scene2`
