#include "gnuplot.h"
#include "map_structure.h"
#include "stratego.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#define THRESHOLD_OF_HARD_POINT 68
#define DEFAULT_MAP_PATH "../Data/"

//Retrieves data such as height, width and other paramaters of the map;
//In the future there might be more data needed to be extracted from the map file
std::string readFromFile(std::string fileName, int &width, int &height);

//Given width and height extracts all points of the map
void harvestData(std::vector<std::vector<pair<int, bool>>> &result,
                 std::string data, int width, int height);

//Function responsible for extraction of each Figure in the map
std::vector<Figure> extractFigures(std::vector<std::vector<pair<int, bool>>> &data);

//Using GNUplot and Plotting folder data draws the final map
void drawWorld(int width, int height);

//Clean ups the Plotting folder data
void cleanResultFiles();

//Main method for parsing and creating the map
void initializeMap();

//Used for path configuration, for text user interface
void setupPaths();

Map_Structure &sMap = Map_Structure::get_instance();
std::string custom_UPPAAL_PATH = "", custom_MAP_PATH = "";

int main() {
  bool exit = false, initializedMap = false;
  int choice;

  while (!exit) {
    std::cout << "Choose option 1-5" << std::endl;
    std::cout << "1. Setup paths" << std::endl;
    std::cout
        << "2. Initilize the map (Generates the static analysis of the map)"
        << std::endl;
    std::cout << "3. Request for station plan" << std::endl;
    std::cout << "4. Request for waypoint plan" << std::endl;
    std::cout << "5. Exit" << std::endl;

    std::cin >> choice;

    switch (choice) {
      case 1:
        setupPaths();
        break;
      case 2:
        try {
          initializeMap();
        }
        catch (char const *msg) {
          std::cout << "error: " << msg << std::endl;
        }
        initializedMap = true;
        break;
      case 3:
        if (initializedMap) {
          try {
            std::cout << stratego::getSingleTrace(queryType::stations,
                                                  custom_UPPAAL_PATH.empty() ? DEFAULT_UPPAAL_PATH : custom_MAP_PATH)
                      << std::endl;
          }
          catch (char const *msg) {
            std::cout << "error: " << msg << std::endl;
          }
        }
        else
          std::cout << "Please initialize map first" << std::endl;
        break;
      case 4:
        if (initializedMap) {
          try {
            std::cout << stratego::getSingleTrace(queryType::waypoints,
                                                  custom_UPPAAL_PATH.empty() ? DEFAULT_UPPAAL_PATH : custom_MAP_PATH)
                      << std::endl;
          }
          catch (char const *msg) {
            std::cout << "error: " << msg << std::endl;
          }
        }
        else
          std::cout << "Please initialize map first" << std::endl;
        break;
      case 5:
        exit = true;
        break;
      default:
        exit = true;
        break;
    }
  }
  return 0;
}

void setupPaths(){
  int choice;
  regex regexp("^(~/|/)([[a-zA-Z_0-9]+/]*)+$");
  smatch m;

  while (true){
    std::cout << "Choose option 1-4" << std::endl;
    std::cout << "1. Set Uppaal Stratego 64-bit path" << std::endl;
    std::cout << "2. Set map path"<< std::endl;
    std::cout << "3. Use default paths" << std::endl;
    std::cout << "4. Back" << std::endl;

    std::cin >> choice;
    std::cout << "choice: "<<choice<<std::endl;
    switch (choice) {
      case 1:
        std::cout<<"Enter Uppaal path (Default:~/Desktop/uppaalStratego/)" <<std::endl;
        cin >> custom_UPPAAL_PATH;
        while(!regex_match(custom_UPPAAL_PATH, regexp)){
          std::cout<< "Incorrect path, please try again" <<std::endl;
          cin >> custom_UPPAAL_PATH;

        }
        std::cout << "New Uppaal path: " << custom_UPPAAL_PATH <<std::endl;
        break;
      case 2:
        std::cout<<"Enter map and station folder path (Default:.../mapConversion/Data/)" <<std::endl;
        std::cout<<"Map file name: data.txt" <<std::endl;
        std::cout<<"Station file name: points.json" <<std::endl;
        while(!regex_match(custom_MAP_PATH, regexp)){
          std::cout<< "Incorrect path, please try again" <<std::endl;
          cin >> custom_MAP_PATH;
        }
        std::cout << "New Map folder path: " << custom_MAP_PATH <<std::endl;
        break;
      case 3:
        custom_MAP_PATH = "";
        custom_UPPAAL_PATH = "";
        std::cout<< "The paths have been chosen to be default ones" <<std::endl;
        std::cout <<"Uppaal path:"<< DEFAULT_UPPAAL_PATH <<std::endl;
        std::cout <<"Map folder path:"<< DEFAULT_MAP_PATH <<std::endl;
        break;
      case 4:
        return;
      default:
        return;
    }
  }
}

void initializeMap() {
  try{
    cleanResultFiles();

    int width = -1, height = -1;
    // gathers data from file and cleans it up
    std::string data = readFromFile(custom_MAP_PATH.empty()? DEFAULT_MAP_PATH :custom_MAP_PATH , width, height);
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;

    std::vector<std::vector<pair<int, bool>>> matrix(height);
    // harvest the data from string into 2D vector
    harvestData(matrix, data, width, height);
    // std::cout << "x matrix: " << matrix.back().size()<<std::endl;
    // std::cout << "y matrix: " << matrix.size()<<std::endl;

    sMap.initializeStations(DEFAULT_MAP_PATH);
    std::cout << "initializeStations complete" << std::endl;

    extractFigures(matrix);
    std::cout << "extractFigures complete" << std::endl;

    sMap.collectAllWayPoints();
    std::cout << "collectAllWayPoints complete" << std::endl;
    sMap.setAllPossibleLines();
    std::cout << "setAllPossibleLines complete" << std::endl;

    //perform sorting of all the lines for the creation of static config
    sMap.sortLines();
    //After the setup of the map was completed, invoked the drawing of the map
    drawWorld(width, height);

    //Invokes the creation of static_config file
    sMap.createStaticJSON("../config");
  }
  catch(char const* msg){
    throw msg;
  }
  return;
}

void cleanResultFilesHelper(std::string path){
  std::ofstream ofs;
  ofs.open(path, std::ofstream::out | std::ofstream::trunc);
  if(ofs.fail()) throw "Failed opening file: " + path;
  ofs.close();
  return;
}

void cleanResultFiles() {
  try{
    mkdir("../Plotting", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cleanResultFilesHelper("../Plotting/offset.dat");
    cleanResultFilesHelper("../Plotting/figures.dat");
    cleanResultFilesHelper("../Plotting/stations.dat");
    cleanResultFilesHelper("../Plotting/lines.dat");
  }
  catch(char const* msg){
    throw msg;
  }
  return;
}

void harvestData(std::vector<std::vector<pair<int, bool>>> &result,
                 std::string data, int width,
                 int height) { // stores data in result
  std::stringstream ss(data);
  int counter = 1, counter2 = 0;
  while (ss.good()) {
    std::string substr;
    getline(ss, substr, ',');
    result[counter2].push_back(make_pair(stoi(substr.c_str()), false));
    if (counter++ == width) {
      counter2++;
      counter = 1;
    }
  }
  //plots the corners of the map
  ofstream myfile;
  myfile.open("../Plotting/figures.dat", std::ios_base::app);
  if(myfile.fail())
    throw "Failed opening file: /Plotting/figures.dat";
  myfile << result[0].size() << " " << 0 << "\n";
  myfile << 0 << " " << result.size() << "\n";
  myfile << 0 << " " << 0 << "\n";
  myfile.close();
  
  return;
}

std::string readFromFile(std::string fileName, int &width, int &height) {
  std::string line, full;
  std::size_t pos;
  std::string fileLoc = fileName+"data.txt";
  std::ifstream myfile(fileLoc);
  if(myfile.fail()) throw "Failed opening file: " + fileLoc;
  // read from file
  while (getline(myfile, line)) {
    // if line consist of width of height harvest the values
    if (line.find("width: ") != std::string::npos) {
      pos = line.find("width: ") + 7;
      width = stoi(line.substr(pos, line.size()).c_str());
    }
    if (line.find("height: ") != std::string::npos) {
      pos = line.find("height: ") + 8;
      height = stoi(line.substr(pos, line.size()).c_str());
    }
    full += line;
  }
  myfile.close();
  if(width == -1 || width == -1) throw "Map file does not consist of width or height";
  // clean up data
  pos = full.find("data: [") + 7;
  full = full.substr(pos, full.size());
  pos = full.find("]");
  full.erase(pos, full.size());
  full.erase(std::remove_if(full.begin(), full.end(), ::isspace), full.end());
  full.erase(std::remove(full.begin(), full.end(), '\n'), full.end());
  return full;
}

void identifyAdjPoints(int row, int column, std::shared_ptr<Figure> &fig,
                       std::vector<std::vector<pair<int, bool>>> &data) {
  if (data[row][column].first > THRESHOLD_OF_HARD_POINT && !data[row][column].second) {
    Point point = Point(row, column, 0, Type::realCorner, "", fig);
    fig->addPoint(point);

    data[row][column].second = true;
    for (int i = -3; i <= 3; i++) {
      for (int j = -3; j <= 3; j++) {
        if (column + i >= 0 && row + j >= 0) {
          //recursively call the method with different values 
          //in order to see if there exist more points belonging to this figure
          identifyAdjPoints(row + j, column + i, fig, data);
        }
      }
    }
  }
  return;
}

std::vector<Figure>
extractFigures(std::vector<std::vector<pair<int, bool>>> &data) {
  std::vector<Figure> figures;
  for (int i = 0; i < data.size(); i++) {
    for (int j = 0; j < data[i].size(); j++) {
      if (data[i][j].first > THRESHOLD_OF_HARD_POINT && !data[i][j].second) {
        std::shared_ptr<Figure> fig(new Figure());
        //Whenever we find first hard point, we are creating the new Figure Object for it
        //and scout the nearest area in order to harvest the rest of the points which belong to the figure
        identifyAdjPoints(i, j, fig, data);
        sMap.figures.push_back(std::move(fig));
        //After harvesting the figure we find it's convex hull
        sMap.figures.back()->convexHull();
      }
    }
  }
  return figures;
}

void drawWorld(int width, int height) {
  GnuplotPipe gp;
  std::string command =
      "set style line 2 lc rgb 'black' pt 5\n set style line 3 lc rgb 'red' pt "
      "5\n set style line 4 lc rgb 'blue' pt 5\n set xrange [0:";
  command += width;
  command += "] \n set yrange [0:";
  command += height;
  command += "] \n plot '../Plotting/figures.dat' with points ls 2 ,  '../Plotting/offset.dat' with "
             "points ls 3,  '../Plotting/stations.dat' with points ls 4, '../Plotting/lines.dat' "
             "using 1:2 with linespoints";
  gp.sendLine(command);
  gp.sendEndOfData();
}
