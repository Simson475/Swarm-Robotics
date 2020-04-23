#include "gnuplot.h"
#include "map_structure.h"
#include "stratego.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
//Retreives data such as height, width and other paramaters of the map;
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

Map_Structure &sMap = Map_Structure::get_instance();

int main() {
  bool exit = false, initializedMap = false;
  int choise;
  while (!exit) {
    std::cout << "Choose option 1-4" << std::endl;
    std::cout
        << "1. Initilize the map (Generates the static analyzese of the map)"
        << std::endl;
    std::cout << "2. Request for station plan" << std::endl;
    std::cout << "3. Request for waypoint plan" << std::endl;
    std::cout << "4. Exit" << std::endl;

    std::cin >> choise;

    switch (choise) {
    case 1:
      initializeMap();
      initializedMap = true; 
      break;
    case 2:
      if (initializedMap)
        std::cout << stratego::getSingleTrace(true)
                  << std::endl; 
      else
        std::cout << "Please initialize map first" << std::endl;
      break;
    case 3:
      if (initializedMap)
        std::cout << stratego::getSingleTrace(false)
                  << std::endl; 
      else
        std::cout << "Please initialize map first" << std::endl;
      break;
    case 4:
      exit = true;
      break; 
    }
  }

  return 0;
}
void initializeMap() {
  cleanResultFiles();

  int width, height;
  // gathers data from file and cleans it up
  std::string data = readFromFile("../Data/data.txt", width, height);
  std::cout << "width: " << width << std::endl;
  std::cout << "height: " << height << std::endl;

  std::vector<std::vector<pair<int, bool>>> matrix(height);
  // harvest the data from string into 2D vector
  harvestData(matrix, data, width, height);
  // std::cout << "x matrix: " << matrix.back().size()<<std::endl;
  // std::cout << "y matrix: " << matrix.size()<<std::endl;

  sMap.initializeStations("../Data/");
  std::cout << "initializeStations complete" << std::endl;

  extractFigures(matrix);
  std::cout << "extractFigures complete" << std::endl;

  sMap.collectAllWayPoints();
  std::cout << "collectAllWayPoints complete" << std::endl;
  sMap.setAllPossibleLines();
  std::cout << "setAllPossibleLines complete" << std::endl;
  //plot the off set data to the plotting files
  ofstream myfile;
  myfile.open("../Plotting/plot2.dat", std::ios_base::app);
  for (auto &fig : sMap.figures) {
    for (auto &pts : fig->getOffSet()) {
      myfile << pts->getY() << " " << pts->getX() << "\n";
    }
  }
  myfile.close();
  //After the setup of the map was completed, invoked the drawing of the map
  drawWorld(width, height);

  //perform sorting of all the lines for the creation of static config
  std::sort(sMap.lines.begin(), sMap.lines.end(), less<Line>());
  std::map<int, int> mapOfWords;
  for (auto &line : sMap.lines) {
    int id = line.Geta().lock()->getId();
    if (mapOfWords.find(id) != mapOfWords.end()) {
      int count = mapOfWords.find(id)->second;
      count++;
      mapOfWords[id] = count;
    } else {
      mapOfWords.insert(std::make_pair(id, 1));
    }
  }
  //Invokes the creation of static_config file
  sMap.createStaticJSON("../config");
}
void cleanResultFiles() {
  try{
  std::ofstream ofs;
  ofs.open("../Plotting/plot2.dat", std::ofstream::out | std::ofstream::trunc);
  ofs.close();
  ofs.open("../Plotting/plot.dat", std::ofstream::out | std::ofstream::trunc);
  ofs.close();
  ofs.open("../Plotting/plot3.dat", std::ofstream::out | std::ofstream::trunc);
  ofs.close();
  ofs.open("../Plotting/plot4.dat", std::ofstream::out | std::ofstream::trunc);
  ofs.close();
  }
  catch(exception e){}
}
void harvestData(std::vector<std::vector<pair<int, bool>>> &result,
                 std::string data, int width,
                 int height) { // stores data in result
  std::stringstream ss(data);
  int counter = 1, counter2 = 0;
  while (ss.good()) {
    std::string substr;
    getline(ss, substr, ',');
    result[counter2].push_back(make_pair(atoi(substr.c_str()), false));
    if (counter++ == width) {
      counter2++;
      counter = 1;
    }
  }
  //plots the corner of the map
  ofstream myfile;
  myfile.open("../Plotting/plot.dat", std::ios_base::app);
  myfile << result[0].size() << " " << 0 << "\n";
  myfile << 0 << " " << result.size() << "\n";
  myfile << 0 << " " << 0 << "\n";
  myfile.close();

  return;
}
std::string readFromFile(std::string fileName, int &width, int &height) {
  std::string line, full;
  std::size_t pos;
  std::ifstream myfile(fileName);
  // read from file
  if (myfile.is_open()) {
    while (getline(myfile, line)) {
      // if line consist of width of height harvest the values
      if (line.find("width: ") != std::string::npos) {
        pos = line.find("width: ") + 7;
        width = atoi(line.substr(pos, line.size()).c_str());
      }
      if (line.find("height: ") != std::string::npos) {
        pos = line.find("height: ") + 8;
        height = atoi(line.substr(pos, line.size()).c_str());
      }

      full += line;
    }
    myfile.close();
  } else
    throw std::invalid_argument("File can't be opened");
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
  if (data[row][column].first > 68 && !data[row][column].second) {
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
}

std::vector<Figure>
extractFigures(std::vector<std::vector<pair<int, bool>>> &data) {
  std::vector<Figure> figures;
  for (int i = 0; i < data.size(); i++) {
    for (int j = 0; j < data[i].size(); j++) {
      if (data[i][j].first > 68 && !data[i][j].second) {
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
  command += "] \n plot '../Plotting/plot.dat' with points ls 2 ,  '../Plotting/plot2.dat' with "
             "points ls 3,  '../Plotting/plot3.dat' with points ls 4, '../Plotting/plot4.dat' "
             "using 1:2 with linespoints";
  gp.sendLine(command);
  gp.sendEndOfData();
}
