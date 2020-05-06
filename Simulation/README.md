# Setup of the project
1. Follow the intructions [argos3](https://github.com/ilpincy/argos3) in order to install argos3 tool on your machine
2. Extract Uppaal in directory `~/Desktop/uppaalStratego/`
3. Install [nlohmann-json](https://github.com/nlohmann/json) package on your machine
4. Create build folder inside Simulation folder
5. Open build folder in terminal and `cmake ..`
6. While still in the build folder use command `make`
7. Configure server
	7.1. By default simulator will connect to localhost, if one wants to change that go to
    [connector.cpp](https://github.com/DEIS-Tools/Swarm-Robotics/blob/master/Simulation/controllers/connector.cpp) and change `SERVER_ADDRESS` to desired IP adress
	7.2. By default port used to connect to the server is `20009`, if one wants to change 
		that go to same file from the previous step and change `PORT` to desired port
	7.2. Go inside [uppaalStratego](https://github.com/DEIS-Tools/Swarm-Robotics/tree/master/Simulation/uppaalStratego) folder and run in terminal `g++ -o Server Server.cpp`stratego.cpp
	7.3. Now one is capable of running the server with command `./Server 20009` if port was
		changed in previous steps, replace 20009 with new port
8. While having server running go to [Simulation](https://github.com/DEIS-Tools/Swarm-Robotics/tree/master/Simulation) and run command `argos3 -c experiment/scene2/trajectory.argos`


# Running tests
1. Enter folder [testing](https://github.com/DEIS-Tools/Swarm-Robotics/tree/master/Simulation/testing)
2. Create build folder 
3. `cmake ..`
4. `make && ./testMain`