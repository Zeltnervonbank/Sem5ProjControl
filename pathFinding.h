#ifndef PATH_FINDING_H
#define PATH_FINDING_H

#endif // PATH_FINDING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <set>
#include <stack>

using namespace std;
using namespace cv;


// A C++ Program to implement A* Search Algorithm

using namespace std;

#define ROW 80
#define COL 120

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;
vector<int> vec1;
vector<int> vec2;


// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;

// A structure to hold the neccesary parameters
struct cell
{
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
};


class pathing
{
public:
bool isValid(int, int);
bool isUnBlocked(int, int, int);
bool isDestination(int, int, Pair);
double calculateHValue(int, int, Pair);
void tracePath(cell, Pair);
void aStarSearch(int, Pair, Pair);
void aStarmulti(int, vector<int>, vector<vector<int>>);
}




