#ifndef PATHING_H
#define PATHING_H


#define ROW 400
#define COL 400


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <set>
#include <stack>
#include "waypointnavigation.h"

// Creating a pair of ints
typedef std::pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef std::pair<double, std::pair<int, int>> pPair;

// A struct to hold the parameters for coordinates
struct cell
{
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    //Creating cost parameters
    double f, g, h;
};


class pathing
{
public:
    pathing();
    ~pathing();

    static int grid[ROW][COL];
    static cv::Mat image;

    static void VerboseAStarSearch(Pair src, Pair dest);
    static void AStarSearch(Pair src, Pair dest);
    static void AStarMultiSearch(std::vector<int> src, std::vector<std::vector<int>> dest);
    static void CreatePathToCurrentDestination();

private:
    static bool IsValid(int row, int col);
    static bool IsUnBlocked(int row, int col);
    static bool IsDestination(int row, int col, Pair dest);

    static double CalculateHValue(int row, int col, Pair dest);
    static bool GetCollinearity(Waypoint a, Waypoint b, Waypoint c);

    static void TracePath(cell cellDetails[][COL], Pair dest);
    static Pair ConvertCoordsToPathingCoord(double x, double y);
};



#endif // PATHING_H
