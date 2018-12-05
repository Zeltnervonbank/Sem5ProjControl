#ifndef PATH_FINDING_H
#define PATH_FINDING_H
#define ROW 80
#define COL 120
#endif // PATH_FINDING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <set>
#include <stack>

using namespace std;
using namespace cv;

using namespace std;

// Creating a pair of ints
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;

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
bool isValid(int, int);
bool isUnBlocked(int, int);
bool isDestination(int, int, Pair);
double calculateHValue(int, int, Pair);
void tracePath(cell, Pair);
void aStarSearch(Pair, Pair);
void aStarmulti(vector<int>, vector<vector<int>>);
~pathing();
};




