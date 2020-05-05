Setup of the project:
1. Follow the intructions https://github.com/ilpincy/argos3 in order to install argos3 tool on your machine
2. Extract Uppaal in directory ~/Desktop/uppaalStratego/
3. Install https://github.com/nlohmann/json package on your machine
5. Open build folder in terminal and cmake ..
6. While still in the build folder use command make
7. Configure server
	7.1. By default simulator will connect to localhost, if one wants to change that go to 
		controllers/connector.cpp and change SERVER_ADDRESS to desired IP adress
	7.2. By default port used to connect to the server is 20009, if one wants to change that go to same file
		from the previous step and change PORT to desired port
	7.2. Go inside uppaalStratego folder and type in terminal g++ -o Server Server.cpp
	7.3. 