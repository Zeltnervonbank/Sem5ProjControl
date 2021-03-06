#include "pathing.h"

pathing::pathing()
{}

pathing::~pathing()
{}

// A Function to check if a cell is valid or not
bool pathing::IsValid(int row, int col)
{
    // Returns true if row number and column number is inside the grid

    if(row < ROW && col < COL)
    {
        if(row >= 0 && col >= 0)
        return true;
    }
    return false;
}

// A Function to check if a cell is blocked or not
bool pathing::IsUnBlocked(int row, int col)
{
    // Returns true if the cell is not blocked else false
    return grid[row][col] == 1;
}

// A function to check if the current posistion is the destination
bool pathing::IsDestination(int row, int col, Pair dest)
{
    return row == dest.first && col == dest.second;
}

// A Function to calculate the heuristic.
double pathing::CalculateHValue(int row, int col, Pair dest)
{
    // Return the estimated distance from row and col to dest
    return ((double)sqrt ((row-dest.first)*(row-dest.first)
                        + (col-dest.second)*(col-dest.second)));
}

// A function to track the path from the source to destination
void pathing::TracePath(cell cellDetails[][COL], Pair dest, int destType)
{
    cv::Mat image = cv::imread("../Sem5ProjControl/floor_plan.png", CV_LOAD_IMAGE_COLOR); //laptop location
    printf ("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;

    std::stack<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row
            && cellDetails[row][col].parent_j == col ))
    {
        Path.push (std::make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push (std::make_pair (row, col));

    std::vector<Waypoint> waypoints;

    while (!Path.empty())
    {
        std::pair<int,int> p = Path.top();
        Path.pop();

        //std::printf("-> (%d,%d) ", p.first, p.second);
        Waypoint wp =
        {
            .x = (p.first - (ROW / 2.0)) * 0.25,
            .y = -(p.second - (COL / 2.0)) * 0.25
        };
        wp.isMarble = destType == 2;
        waypoints.push_back(wp);

        image.at<cv::Vec3b>(p.second, p.first) = 255;
    }

    // Try to limit the amount of waypoints by eliminating those that are in a straight line
    for(size_t i = 0; i < waypoints.size(); i++)
    {
        // Always include first and last points
        if(i == 0)
        {
            Globals::waypoints.push(waypoints[i]);
            continue;
        }        
        else if(i == waypoints.size() - 1)
        {
            if(destType == 1)
            {
                waypoints[i].isDestination = true;
            }

            Globals::waypoints.push(waypoints[i]);
            continue;
        }

        // Get the values of each vector
        double vec1X = waypoints[i].x - waypoints[i - 1].x;
        double vec1Y = waypoints[i - 1].y - waypoints[i].y;

        double vec2X = waypoints[i + 1].x - waypoints[i].x;
        double vec2Y = waypoints[i].y - waypoints[i + 1].y;

        // Get dot product of vectors
        double dot = vec1X * vec2X + vec1Y * vec2Y;

        // Get cross product of vectors
        double cross = vec1X * vec2X - vec1Y * vec2Y;

        // Get the magnitude of each vector
        double v1Dist = sqrt(pow(vec1X, 2) + pow(vec1Y, 2));
        double v2Dist = sqrt(pow(vec2X, 2) + pow(vec2Y, 2));

        // Calculate angle between vectors
        double difference = cross < 0 ? -acos(dot / (v1Dist * v2Dist)) : acos(dot / (v1Dist * v2Dist));

        if(abs(difference) > 0)
        {
            Globals::waypoints.push(waypoints[i]);
        }

        // Add point if the next point is not collinear with this and the last point
        /*if(!GetCollinearity(waypoints[i - 1], waypoints[i], waypoints[i + 1]))
        {
            WaypointNavigation::waypoints.push(waypoints[i]);
            std::cout << "-> (" << waypoints[i].x << ", " << waypoints[i].y << ") ";
        }*/
    }

    //std::cout << std::endl;
    cv::namedWindow("scaled", CV_WINDOW_AUTOSIZE);
    cv::imshow("scaled", image);
}

bool pathing::GetCollinearity(Waypoint a, Waypoint b, Waypoint c)
{
    double area = abs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0);
    return area < 0.01;
}

// A Function to find the shortest path between a given source cell to a destination cell
// A* Search Algorithm
void pathing::VerboseAStarSearch(Pair src, Pair dest)
{
    // If the source is out of range
    if (!IsValid (src.first, src.second))
    {
        printf ("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (!IsValid (dest.first, dest.second))
    {
        printf ("Destination is invalid\n");
        return;
    }

    // Either the source or the destination is blocked
    if (!IsUnBlocked(src.first, src.second) || !IsUnBlocked(dest.first, dest.second))
    {
        printf ("Source or the destination is blocked\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (IsDestination(src.first, src.second, dest) == true)
    {
        printf ("We are already at the destination\n");
        return;
    }

    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));

    // Declare a 2D array of structure to hold the details
    //of that cell
    cell cellDetails[ROW][COL];

    int i, j;

    for (i = 0; i < ROW; i++)
    {
        for (j = 0; j < COL; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    /*
    Create an open list having information as-
    <f, <i, j>>
    where f = g + h,
    and i, j are the row and column index of that cell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    This open list is implenented as a set of pair of pair.*/
    std::set<pPair> openList;

    // Put the starting cell on the open list and set its
    // 'f' as 0
    openList.insert(std::make_pair (0.0, std::make_pair (i, j)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty())
    {
        pPair p = *openList.begin();

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        /*
        Generating all the 8 successor of this cell
         N.W  N  N.E
           \  |  /
            \ | /
        W----Cell----E
            / | \
           /  |  \
         S.W  S  S.E
        Cell-->Popped Cell  (i, j)
        N --> North         (i - 1, j)
        S --> South         (i + 1, j)
        E --> East          (i, j + 1)
        W --> West          (i, j - 1)
        N.E--> North-East   (i - 1, j + 1)
        N.W--> North-West   (i - 1, j - 1)
        S.E--> South-East   (i + 1, j + 1)
        S.W--> South-West   (i + 1, j - 1) */

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------

        // Only process this cell if this is a valid one
        if (IsValid(i - 1, j))
        {
            // If the destination cell is the same as the
            // current successor
            if (IsDestination(i - 1, j, dest))
            {
                // Set the Parent of the destination cell
                cellDetails[i - 1][j].parent_i = i;
                cellDetails[i - 1][j].parent_j = j;
                printf ("The destination cell is found\n");
                TracePath (cellDetails, dest, 1);
                foundDest = true;
                return;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (!closedList[i - 1][j] && IsUnBlocked(i - 1, j))
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = CalculateHValue (i - 1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //			 OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i - 1][j].f == FLT_MAX || cellDetails[i - 1][j].f > fNew)
                {
                    openList.insert( std::make_pair(fNew, std::make_pair(i - 1, j)));

                    // Update the details of this cell
                    cellDetails[i - 1][j].f = fNew;
                    cellDetails[i - 1][j].g = gNew;
                    cellDetails[i - 1][j].h = hNew;
                    cellDetails[i - 1][j].parent_i = i;
                    cellDetails[i - 1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------
        if (IsValid(i + 1, j))
        {
            if (IsDestination(i + 1, j, dest))
            {
                cellDetails[i + 1][j].parent_i = i;
                cellDetails[i + 1][j].parent_j = j;
                printf("The destination cell is found\n");
                TracePath(cellDetails, dest, 1);
                foundDest = true;
                return;
            }

            else if (!closedList[i + 1][j] && IsUnBlocked(i + 1, j))
            {
                gNew = cellDetails[i][j].g + 1.0;
                cellDetails[i + 1][j].h = hNew;
                hNew = CalculateHValue(i + 1, j, dest);
                fNew = gNew + hNew;

                if (cellDetails[i + 1][j].f == FLT_MAX || cellDetails[i + 1][j].f > fNew)
                {
                    openList.insert(std::make_pair (fNew, std::make_pair (i + 1, j)));

                    cellDetails[i + 1][j].f = fNew;
                    cellDetails[i + 1][j].g = gNew;
                    cellDetails[i + 1][j].h = hNew;
                    cellDetails[i + 1][j].parent_i = i;
                    cellDetails[i + 1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------
        if (IsValid (i, j + 1))
        {
            if (IsDestination(i, j + 1, dest))
            {
                cellDetails[i][j + 1].parent_i = i;
                cellDetails[i][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                TracePath(cellDetails, dest, 1);
                foundDest = true;
                return;
            }

            else if (!closedList[i][j + 1] && IsUnBlocked (i, j + 1))
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = CalculateHValue (i, j + 1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i][j + 1].f == FLT_MAX || cellDetails[i][j + 1].f > fNew)
                {
                    openList.insert(std::make_pair(fNew, std::make_pair (i, j + 1)));

                    cellDetails[i][j + 1].f = fNew;
                    cellDetails[i][j + 1].g = gNew;
                    cellDetails[i][j + 1].h = hNew;
                    cellDetails[i][j + 1].parent_i = i;
                    cellDetails[i][j + 1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------
        if (IsValid(i, j - 1))
        {
            if (IsDestination(i, j - 1, dest))
            {
                cellDetails[i][j - 1].parent_i = i;
                cellDetails[i][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                TracePath(cellDetails, dest, 1);
                foundDest = true;
                return;
            }

            else if (!closedList[i][j - 1] && IsUnBlocked(i, j - 1))
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = CalculateHValue(i, j - 1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i][j - 1].f == FLT_MAX || cellDetails[i][j - 1].f > fNew)
                {
                    openList.insert( std::make_pair (fNew, std::make_pair (i, j - 1)));

                    cellDetails[i][j - 1].f = fNew;
                    cellDetails[i][j - 1].g = gNew;
                    cellDetails[i][j - 1].h = hNew;
                    cellDetails[i][j - 1].parent_i = i;
                    cellDetails[i][j - 1].parent_j = j;
                }
            }
        }

        //----------- 5th Successor (North-East) ------------
        if (IsValid(i - 1, j + 1))
        {
            if (IsDestination(i - 1, j + 1, dest))
            {
                cellDetails[i - 1][j + 1].parent_i = i;
                cellDetails[i - 1][j + 1].parent_j = j;
                printf ("The destination cell is found\n");
                TracePath (cellDetails, dest, 1);
                foundDest = true;
                return;
            }

            else if (!closedList[i - 1][j + 1] && IsUnBlocked(i - 1, j + 1))
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = CalculateHValue(i - 1, j + 1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i - 1][j + 1].f == FLT_MAX || cellDetails[i - 1][j + 1].f > fNew)
                {
                    openList.insert( std::make_pair (fNew, std::make_pair(i - 1, j + 1)));

                    cellDetails[i - 1][j + 1].f = fNew;
                    cellDetails[i - 1][j + 1].g = gNew;
                    cellDetails[i - 1][j + 1].h = hNew;
                    cellDetails[i - 1][j + 1].parent_i = i;
                    cellDetails[i - 1][j + 1].parent_j = j;
                }
            }
        }

        //----------- 6th Successor (North-West) ------------
        if (IsValid (i - 1, j - 1))
        {
            if (IsDestination (i - 1, j - 1, dest))
            {
                cellDetails[i - 1][j - 1].parent_i = i;
                cellDetails[i - 1][j - 1].parent_j = j;
                printf ("The destination cell is found\n");
                TracePath (cellDetails, dest, 1);
                foundDest = true;
                return;
            }

            else if (!closedList[i - 1][j - 1] && IsUnBlocked(i - 1, j - 1))
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = CalculateHValue(i - 1, j - 1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i - 1][j - 1].f == FLT_MAX || cellDetails[i - 1][j - 1].f > fNew)
                {
                    openList.insert( std::make_pair (fNew, std::make_pair (i - 1, j - 1)));

                    cellDetails[i - 1][j - 1].f = fNew;
                    cellDetails[i - 1][j - 1].g = gNew;
                    cellDetails[i - 1][j - 1].h = hNew;
                    cellDetails[i - 1][j - 1].parent_i = i;
                    cellDetails[i - 1][j - 1].parent_j = j;
                }
            }
        }
        if (IsValid (i + 1, j - 1))
        {
            if (IsDestination(i + 1, j - 1, dest))
            {
                cellDetails[i + 1][j - 1].parent_i = i;
                cellDetails[i + 1][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                TracePath(cellDetails, dest, 1);
                foundDest = true;
                return;
            }

            else if (!closedList[i + 1][j - 1] && IsUnBlocked(i + 1, j - 1))
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = CalculateHValue(i + 1, j - 1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i + 1][j - 1].f == FLT_MAX || cellDetails[i + 1][j - 1].f > fNew)
                {
                    openList.insert(std::make_pair(fNew, std::make_pair(i + 1, j - 1)));

                    cellDetails[i + 1][j - 1].f = fNew;
                    cellDetails[i + 1][j - 1].g = gNew;
                    cellDetails[i + 1][j - 1].h = hNew;
                    cellDetails[i + 1][j - 1].parent_i = i;
                    cellDetails[i + 1][j - 1].parent_j = j;
                }
            }
        }
        //----------- 7th Successor (South-East) ------------
        if (IsValid(i + 1, j + 1))
        {
            if (IsDestination(i + 1, j + 1, dest))
            {
                cellDetails[i + 1][j + 1].parent_i = i;
                cellDetails[i + 1][j + 1].parent_j = j;
                printf ("The destination cell is found\n");
                TracePath (cellDetails, dest, 1);
                foundDest = true;
                return;
            }

            else if (!closedList[i + 1][j + 1] &&
                    IsUnBlocked(i + 1, j + 1))
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = CalculateHValue(i + 1, j + 1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i + 1][j + 1].f == FLT_MAX || cellDetails[i + 1][j + 1].f > fNew)
                {
                    openList.insert(std::make_pair(fNew, std::make_pair (i + 1, j + 1)));

                    cellDetails[i + 1][j + 1].f = fNew;
                    cellDetails[i + 1][j + 1].g = gNew;
                    cellDetails[i + 1][j + 1].h = hNew;
                    cellDetails[i + 1][j + 1].parent_i = i;
                    cellDetails[i + 1][j + 1].parent_j = j;
                }
            }
        }

        //----------- 8th Successor (South-West) ------------
        if (IsValid (i + 1, j - 1))
        {
            if (IsDestination(i + 1, j - 1, dest))
            {
                cellDetails[i + 1][j - 1].parent_i = i;
                cellDetails[i + 1][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                TracePath(cellDetails, dest, 1);
                foundDest = true;
                return;
            }

            else if (!closedList[i + 1][j - 1] && IsUnBlocked(i + 1, j - 1))
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = CalculateHValue(i + 1, j - 1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i + 1][j - 1].f == FLT_MAX || cellDetails[i + 1][j - 1].f > fNew)
                {
                    openList.insert(std::make_pair(fNew, std::make_pair(i + 1, j - 1)));

                    cellDetails[i + 1][j - 1].f = fNew;
                    cellDetails[i + 1][j - 1].g = gNew;
                    cellDetails[i + 1][j - 1].h = hNew;
                    cellDetails[i + 1][j - 1].parent_i = i;
                    cellDetails[i + 1][j - 1].parent_j = j;
                }
            }
        }
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    if (!foundDest)
    {
        printf("Failed to find the Destination Cell\n");
    }
}

// Does the same as aStarSearch, but is less verbose
void pathing::AStarSearch(Pair src, Pair dest, int destType)
{

    // If the source is out of range
    if (!IsValid (src.first, src.second))
    {
        printf ("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (!IsValid (dest.first, dest.second))
    {
        printf ("Destination is invalid\n");
        return;
    }

    // Either the source or the destination is blocked
    if (!IsUnBlocked(src.first, src.second) || !IsUnBlocked(dest.first, dest.second))
    {
        printf ("Source or the destination is blocked\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (IsDestination(src.first, src.second, dest) == true)
    {
        printf ("We are already at the destination\n");
        return;
    }

    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));
    /*
    Generating all the 8 successor of this cell
     N.W  N  N.E
       \  |  /
        \ | /
    W----Cell----E
        / | \
       /  |  \
     S.W  S  S.E
    Cell-->Popped Cell  (i, j)
    N --> North         (i - 1, j)
    S --> South         (i + 1, j)
    E --> East          (i, j + 1)
    W --> West          (i, j - 1)
    N.E--> North-East   (i - 1, j + 1)
    N.W--> North-West   (i - 1, j - 1)
    S.E--> South-East   (i + 1, j + 1)
    S.W--> South-West   (i + 1, j - 1) */
    // Declare a 2D array of structure to hold the details
    //of that cell
    cell cellDetails[ROW][COL];

    int i, j;

    for (i = 0; i < ROW; i++)
    {
        for (j = 0; j < COL; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;
    /*
    Create an open list having information as-
    <f, <i, j>>
    where f = g + h,
    and i, j are the row and column index of that cell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    This open list is implenented as a set of pair of pair.*/
    std::set<pPair> openList;

    // Put the starting cell on the open list and set its
    // 'f' as 0
    openList.insert(std::make_pair (0.0, std::make_pair (i, j)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty())
    {
        pPair p = *openList.begin();

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        std::pair<int, int> neighbours[] =
        {
            {i - 1, j},
            {i + 1, j},
            {i, j + 1},
            {i, j - 1},
            {i - 1, j + 1},
            {i - 1, j - 1},
            {i + 1, j + 1},
            {i + 1, j - 1}
        };

        for(int x = 0; x < 8; x++)
        {
            // Get values of i and j for this iteration
            int nI = neighbours[x].first;
            int nJ = neighbours[x].second;

            // 1 for cardinals, sqrt(2) for diagonals
            double addValue = x < 4 ? 1.0 : 1.414;

            double gNew, hNew, fNew;

            if (IsValid (nI, nJ))
            {
                if (IsDestination(nI, nJ, dest))
                {
                    cellDetails[nI][nJ].parent_i = i;
                    cellDetails[nI][nJ].parent_j = j;
                    //printf("The destination cell is found\n");
                    TracePath(cellDetails, dest, destType);
                    foundDest = true;
                    return;
                }

                else if (!closedList[nI][nJ] && IsUnBlocked(nI, nJ))
                {
                    gNew = cellDetails[i][j].g + addValue;
                    hNew = CalculateHValue(nI, nJ, dest);
                    fNew = gNew + hNew;

                    if (cellDetails[nI][nJ].f == FLT_MAX || cellDetails[nI][nJ].f > fNew)
                    {
                        openList.insert(std::make_pair(fNew, std::make_pair(nI, nJ)));
                        cellDetails[nI][nJ].f = fNew;
                        cellDetails[nI][nJ].g = gNew;
                        cellDetails[nI][nJ].h = hNew;
                        cellDetails[nI][nJ].parent_i = i;
                        cellDetails[nI][nJ].parent_j = j;
                    }
                }
            }
        }
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    if (!foundDest)
    {
        printf("Failed to find the Destination Cell\n");
    }
}

void pathing::AStarMultiSearch(std::vector<int> src, std::vector<std::vector<int>> dest)
{
    Pair start = std::make_pair(src[0], src[1]);
    Pair goal = std::make_pair(dest[0][0], dest[0][1]);

    for(size_t i = 0; i < dest.size(); i++)
    {
        if(i != 0)
        {
            start = std::make_pair(dest[i - 1][0], dest[i - 1][1]);
            goal = std::make_pair(dest[i][0], dest[i][1]);
        }

        AStarSearch(start, goal, 1);
    }
}

void pathing::CreatePathToCurrentDestination()
{
    // Convert current destination and current position to pair form
    Pair destination = ConvertCoordsToPathingCoord(Globals::currentDestination.x, Globals::currentDestination.y);
    Pair currentPosition = ConvertCoordsToPathingCoord(Globals::lastPosition.posX, Globals::lastPosition.posY);

    // Clear current waypoints
    Globals::ClearWaypointQueue();

    // Make a new path
    AStarSearch(currentPosition, destination, 1);

    // Set a new next waypoint
    //Globals::NextWaypoint();
}

Pair pathing::ConvertCoordsToPathingCoord(double x, double y)
{
    return std::make_pair(x * 4.0 + 200, 200 - y * 4.0);
}
