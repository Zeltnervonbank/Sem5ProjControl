#ifndef PATHING_H
#define PATHING_H


#define ROW 80
#define COL 120


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <set>
#include <stack>

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

    static bool isValid(int row, int col);
    static bool isUnBlocked(int row, int col);
    static bool isDestination(int row, int col, Pair dest);
    static double calculateHValue(int row, int col, Pair dest);
    static void tracePath(cell cellDetails[][COL], Pair dest);
    static void aStarSearch(Pair src, Pair dest);
    static void aStarmulti(std::vector<int> src, std::vector<std::vector<int>> dest);
};



#endif // PATHING_H
