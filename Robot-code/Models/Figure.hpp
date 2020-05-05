#ifndef FIGURE
#define FIGURE

#include <sys/stat.h>
#include <sys/types.h>
#include <math.h> 
#include <vector>
#include <string>
#include <memory>
#include <bits/stdc++.h> 
#include <iostream>
#include <fstream>
#include <Line.hpp>


#define VELOCITY 100.0f
#define INF 999999
#define robotDiameter 1.5f
#define offset robotDiameter * 3

class Point;

class Figure:public std::enable_shared_from_this<Figure>{
    private:
    //Storage of all rough points of the figure
    std::vector<Point> points;
    //Storate of convex hull a.k.a. in this off set points of figure
    std::vector<std::shared_ptr<Point>> offsetPoints;
    //The center cordinates of this figure
    std::shared_ptr<Point> avrgCenter;
    //final off set lines of this figure
    std::vector<std::shared_ptr<Line>> offSetLines;
    
    public:
    //Empty constructor
    Figure(){};
    //adds hard point to this Figure
    void addPoint(Point& p){points.push_back(p);}
    //simple getters
    std::vector<Point>& getPoints(){return points;}
    std::vector<std::shared_ptr<Line>>& getOffSetLines(){return offSetLines;}
    std::vector<std::shared_ptr<Point>> getOffSet(){return offsetPoints;}
    std::shared_ptr<Point>& getCenter() { return avrgCenter; }
    //Create of this figure convex hull
    void convexHull();
    //Given convex hull finds this Figure's avarage center coordinates
    void findAvrgCenter(std::vector<Point>& hull);
    //Connects all offsets points to get viable lines from them.
    void connectLines();
    //Arranges properly the convex hull
    void cleanConvex(std::vector<Point>& hull);
    //Moves 2 of convex hull points in order to get their off set by distance d ( which should be robot diameter )
    std::pair<Point, Point> moveLine(Point p1, Point p2, float d);
    //Clean ups convex hull, from unnesacary points in order to reduce amount of vias
    void cleanUp(std::vector<Point>& hull);
    //Finds where two lines would intersect if one would extend both of them
    std::shared_ptr<Point> findIntersection(std::pair<Point, Point>& pair1, std::pair<Point, Point>& pair2);
    //function to check if given point is inside of this Figure
    bool isInside(Point p);
    //removes point from offsetPoints for optimization purposes
    void removeOffPoint(std::shared_ptr<Point>& p);

};
#endif