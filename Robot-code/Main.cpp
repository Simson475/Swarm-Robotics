#include "gnuplot.h"
#include "map_structure.h"
#include "stratego.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sys/resource.h>

#define THRESHOLD_OF_HARD_POINT 65
#define DEFAULT_MAP_PATH "../Data/"
//If there are less then MAX_NOISINESS points in area it will be considered
//as noisiness, thus will be ignored
#define MAX_NOISINESS 3
//Used to find all the adj points. The bigger the value the wider
//range is going to be used
#define POINT_BELONGS 8

//Retrieves data such as height, width and other parameters of the map;
//In the future there might be more data needed to be extracted from the map file
std::string readFromFile(std::string fileName, int &width, int &height);

//Given width and height extracts all points of the map
void harvestData(std::vector<std::vector<pair<int, bool>>> &result,
                 const std::string& data);

//Function responsible for extraction of each Figure in the map
std::vector<Figure> extractFigures(std::vector<std::vector<pair<int, bool>>> &data);

//Using GNUplot and Plotting folder data draws the final map
void drawWorld();

//Clean ups the Plotting folder data
void cleanResultFiles();

//Main method for parsing and creating the map
void initializeMap();

//Used for path configuration, for text user interface
void setupPaths();

Map_Structure &sMap = Map_Structure::get_instance();
std::string custom_UPPAAL_PATH, custom_MAP_PATH;
int width = -1, height = -1;

int main() {
    const rlim_t kStackSize = 100 * 1024 * 1024;   // min stack size = 100 MB
    struct rlimit rl{};
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0)
            {
                fprintf(stderr, "setrlimit returned result = %d\n", result);
            }
        }
    }
  bool initializedMap = false;
  int choice;

  while (true) {
    std::cout << "Choose option 1-6" << std::endl;
    std::cout << "1. Setup paths" << std::endl;
    std::cout
        << "2. Initialize the map (Generates the static analysis of the map)"
        << std::endl;
    std::cout << "3. Request for station plan" << std::endl;
    std::cout << "4. Request for waypoint plan" << std::endl;
    std::cout << "5. Display map" << std::endl;
    std::cout << "6. Exit" << std::endl;

    std::cin >> choice;

    switch (choice) {
      case 1:
        setupPaths();
        break;
      case 2:
        try {
          initializeMap();
        }
        catch (std::exception& e) {
          std::cout << "error: " << e.what() << std::endl;
        }
        initializedMap = true;
        break;
      case 3:
        if (initializedMap) {
          try {
            std::cout << stratego::getSingleTrace(stratego::queryType::stations,
                                                  custom_UPPAAL_PATH.empty() ? DEFAULT_UPPAAL_PATH : custom_MAP_PATH)
                      << std::endl;
          }
          catch (std::runtime_error &e) {
            std::cout << "error: " << e.what() << std::endl;
          }
        }
        else
          std::cout << "Please initialize map first" << std::endl;
        break;
      case 4:
        if (initializedMap) {
          try {
            std::cout << stratego::getSingleTrace(stratego::queryType::waypoints,
                                                  custom_UPPAAL_PATH.empty() ? DEFAULT_UPPAAL_PATH : custom_MAP_PATH)
                      << std::endl;
          }
          catch (std::exception& e) {
            std::cout << "error: " << e.what() << std::endl;
          }
        }
        else
          std::cout << "Please initialize map first" << std::endl;
        break;
      case 5:
        if (initializedMap) {
          //After the initialization of the map was completed, one may invoke the drawing of the map
          drawWorld();
        }
        else std::cout << "Please initialize map first" << std::endl;
        break;
      case 6:
        return 0;
      default:
        std::cout << "Please select a proper choice 1-6" <<std::endl;
        break;
    }
  }
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
          std::cout << "Please select a proper choice 1-4" <<std::endl;
          break;
    }
  }
}

void initializeMap() {
  try{
    cleanResultFiles();

    // gathers data from file and cleans it up
    std::string data = readFromFile(custom_MAP_PATH.empty()? DEFAULT_MAP_PATH :custom_MAP_PATH , width, height);
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;

    std::vector<std::vector<pair<int, bool>>> matrix(height);
    // harvest the data from string into 2D vector
    harvestData(matrix, data);
    // std::cout << "x matrix: " << matrix.back().size()<<std::endl;
    // std::cout << "y matrix: " << matrix.size()<<std::endl;

    sMap.initializeStations(DEFAULT_MAP_PATH);
    std::cout << "initializeStations complete" << std::endl;

    extractFigures(matrix);
    std::cout << "extractFigures complete" << std::endl;

    sMap.collectAllWayPoints();
    std::cout << "collectAllWayPoints complete" << std::endl;
    sMap.setAllPossibleLines(width*height);
    std::cout << "setAllPossibleLines complete" << std::endl;

    //perform sorting of all the lines for the creation of static config
    sMap.sortLines();

    //Invokes the creation of static_config file
    sMap.createStaticJSON("../config");
  }
  catch(std::exception& e){
    throw e;
  }
}

void cleanResultFilesHelper(const std::string& path){
  std::ofstream ofs;
  ofs.open(path, std::ofstream::out | std::ofstream::trunc);
  if(ofs.fail()) throw std::runtime_error("Failed opening file: " + path);
  ofs.close();
}

void cleanResultFiles() {
  try{
    
    if (mkdir("../Plotting", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
    {
      if( errno != EEXIST ) {
        throw std::runtime_error("cannot create Plotting folder");
      }
    }
    cleanResultFilesHelper("../Plotting/offset.dat");
    cleanResultFilesHelper("../Plotting/figures.dat");
    cleanResultFilesHelper("../Plotting/stations.dat");
    cleanResultFilesHelper("../Plotting/lines.dat");
  }
  catch(std::exception& e){
    throw e;
  }
  }

void harvestData(std::vector<std::vector<pair<int, bool>>> &result,
                 const std::string& data) { // stores data in result
  std::stringstream ss(data);
  int counter = 1, counter2 = 0;
  while (ss.good()) {
    std::string substr;
    getline(ss, substr, ',');
    result[counter2].push_back(make_pair(stoi(substr), false));
    if (counter++ == width) {
      counter2++;
      counter = 1;
    }
  }
  //plots the corners of the map
  ofstream file;
  file.open("../Plotting/figures.dat", std::ios_base::app);
  if(file.fail())
    throw std::runtime_error("Failed opening file: /Plotting/figures.dat");
  file << result[0].size() << " " << 0 << "\n";
  file << 0 << " " << result.size() << "\n";
  file << 0 << " " << 0 << "\n";
  file.close();

}

std::string readFromFile(std::string fileName, int &width, int &height) {
  std::string line, full;
  std::size_t pos;
  std::string fileLoc = fileName+"data.txt";
  std::ifstream file(fileLoc);
  if(file.fail()) throw std::runtime_error("Failed opening file: " + fileLoc);
  // read from file
  while (getline(file, line)) {
    // if line consist of width of height harvest the values
    if (line.find("width: ") != std::string::npos) {
      pos = line.find("width: ") + 7;
      width = stoi(line.substr(pos, line.size()));
    }
    if (line.find("height: ") != std::string::npos) {
      pos = line.find("height: ") + 8;
      height = stoi(line.substr(pos, line.size()));
    }
    full += line;
  }
  file.close();
  if(width == -1 || height == -1) throw std::runtime_error("Map file does not consist of width or height");
  // clean up data
  pos = full.find("data: [") + 7;
  full = full.substr(pos, full.size());
  pos = full.find(']');
  full.erase(pos, full.size());
  full.erase(std::remove_if(full.begin(), full.end(), ::isspace), full.end());
  full.erase(std::remove(full.begin(), full.end(), '\n'), full.end());
  return full;
}

void identifyAdjPoints(int row, int column, std::shared_ptr<Figure> &fig,
                       std::vector<std::vector<pair<int, bool>>> &data) {
  if ((data[row][column].first > THRESHOLD_OF_HARD_POINT || data[row][column].first ==-1) && !data[row][column].second) {
  //if (data[row][column].first > THRESHOLD_OF_HARD_POINT && !data[row][column].second) {
    //Point point = Point(row, column, 0, Type::realCorner, "", fig);
   // std::cout << fig->getPoints().max_size() <<std::endl;
    fig->addPoint(Point(row, column, 0, Type::realCorner, " ", fig));
    data[row][column].second = true;
    for (int i = -POINT_BELONGS; i <= POINT_BELONGS; i++) {
      for (int j = -POINT_BELONGS; j <= POINT_BELONGS; j++) {
        if (column + i >= 0 && row + j >= 0 && column + i < width && row + j < height) {
          //recursively call the method with different values 
          //in order to see if there exist more points belonging to this figure
          identifyAdjPoints(row + j, column + i, fig, data);
        }
      }
    }
  }
}

std::vector<Figure>
extractFigures(std::vector<std::vector<pair<int, bool>>> &data) {
    bool once = true;
  std::vector<Figure> figures;
  for (int i = 0; i < data.size(); i++) {
    for (int j = 0; j < data[i].size(); j++) {
      if ((data[i][j].first > THRESHOLD_OF_HARD_POINT || data[i][j].first ==-1) && !data[i][j].second) {
     //if (data[i][j].first > THRESHOLD_OF_HARD_POINT && !data[i][j].second) {
              std::shared_ptr<Figure> fig(new Figure());
              //Whenever we find first hard point, we are creating the new Figure Object for it
              //and scout the nearest area in order to harvest the rest of the points which belong to the figure
              identifyAdjPoints(i, j, fig, data);
              //If figure contains of less then MAX_NOISINESS then we ignore this figure
              if(fig->getPoints().size() >MAX_NOISINESS){
                  if(!once) {
                      //After harvesting the figure we find it's convex hull
                      fig->convexHull();
                      //Finally save the new figure
                      sMap.figures.push_back(fig);
                  }
                  once = false;
              }
          }
    }
  }
  return figures;
}

void drawWorld() {
  GnuplotPipe gp;
  std::string command =
      "set style line 2 lc rgb 'black' pt 5\n set style line 3 lc rgb 'red' pt "
      "5\n set style line 4 lc rgb 'blue' pt 5\n set xrange [0:";
  command += std::to_string(width);
  command += "] \n set yrange [0:";
  command += std::to_string(height);
  command += "] \n plot '../Plotting/figures.dat' with points ls 2 ,  '../Plotting/offset.dat' with "
             "points ls 3,  '../Plotting/stations.dat' with points ls 4, '../Plotting/lines.dat' "
             "using 1:2 with linespoints";
  gp.sendLine(command);
  gp.sendEndOfData();
}
