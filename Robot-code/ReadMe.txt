Setup of the project:
1. Download Uppaal Stratego 64bit https://people.cs.aau.dk/~marius/stratego/download.html
2. Extract Uppaal in directory ~/Desktop/uppaalStratego/
3. Install https://github.com/nlohmann/json package on your machine
4. Create build folder inside mapConversion
5. Open build folder in terminal and cmake ..
6. While still in the build folder make && ./Main

Content of the project:
Config folder:
    Queries of Uppaal
    Models of Uppaal
    Static and Dynamic config files
Data folder:
    Stations coordinates in JSON format
    data.txt file which is the data received from the robot
Library folder:
    Consist of library which helps Uppaal to retrieve data from config files
Plotting folder:
    Consist of plotting data in order to visualize the final map

map_elements consist of fundamental classes in the project
map_Structure holds the most important data and methods in order to create static and dynamic configs
stratego is responsible of communication between this project and Uppaal
main is responsible of project workflow

