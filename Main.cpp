/*
* I massively refactored the code from Red Blob Games and added comments to show
* that I understand how the algorithm works.
* 
* Sample code from https://www.redblobgames.com/pathfinding/a-star/
* Copyright 2014 Red Blob Games <redblobgames@gmail.com>
*
* Feel free to use this code in your own projects, including commercial projects
* License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
*/
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <queue>
#include <cstdlib>

using namespace std;

// This structure holds the x and y coordinates of a location on a 2 dimensional grid.
struct GridLocation 
{
    int x, y;
};

// implement hash function so we can put GridLocation into an unordered_set
template <> struct hash<GridLocation> 
{
    size_t operator()(const GridLocation& id) const noexcept 
    {
        return hash<int>()(id.x ^ (id.y << 16));
    }
};

// 
struct SquareGrid 
{
    // This property holds an array of 4 grid locations explanation is in the definition below
    static array<GridLocation, 4> DIRS;
    // Height and width of the grid
    int width, height;
    // Set used to contain all of the gridlocations that have walls
    unordered_set<GridLocation> walls;

    // Constructor for a SquareGrid object
    SquareGrid(int width_, int height_)
        : width(width_), height(height_) {}

    // Function used to check the bounds of the grid
    bool in_bounds(GridLocation id) const 
    {
        return 0 <= id.x && id.x < width
            && 0 <= id.y && id.y < height;
    }

    // Function used to test if a point on the grid is a wall
    bool passable(GridLocation id) const 
    {
        return walls.find(id) == walls.end();
    }

    /* function that uses the DIRS array to calculate the neighbors
    * of the GridLocation passed. Does not return out of bounds or walls*/
    vector<GridLocation> neighbors(GridLocation id) const 
    {
        vector<GridLocation> results;

        for (GridLocation dir : DIRS) 
        {
            GridLocation next{ id.x + dir.x, id.y + dir.y };
            if (in_bounds(next) && passable(next))
            {
                results.push_back(next);
            }
        }
        return results;
    }
};

/* This array is used to calculate neighbors of a point by adding or subtracting 
* the values from the coordinates on the GridLocation being calculated.
*/
array<GridLocation, 4> SquareGrid::DIRS = 
{
    /* East,  West,
    *  North, South */
    GridLocation{1, 0}, GridLocation{-1, 0},
    GridLocation{0, -1}, GridLocation{0, 1}
};

/* Helpers for GridLocation
* operator overloads for GridLocation
*/

bool operator == (GridLocation a, GridLocation b) 
{
    return a.x == b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b) 
{
    return !(a == b);
}

bool operator < (GridLocation a, GridLocation b) 
{
    return tie(a.x, a.y) < tie(b.x, b.y);
}

basic_iostream<char>::basic_ostream& operator<<(basic_iostream<char>::basic_ostream& out, const GridLocation& loc) 
{
    out << '(' << loc.x << ',' << loc.y << ')';
    return out;
}

/* This outputs a grid.Pass in a distances map if you want to print
* the distances, or pass in a point_to map if you want to print
* arrows that point to the parent location, or pass in a path vector
* if you want to draw the path.*/
template<class Graph>
void draw_grid(const Graph& graph,
    unordered_map<GridLocation, double>* distances = nullptr,
    unordered_map<GridLocation, GridLocation>* point_to = nullptr,
    vector<GridLocation>* path = nullptr,
    GridLocation* start = nullptr,
    GridLocation* goal = nullptr) 
{
    const int field_width = 3;
    cout << string(field_width * graph.width, '_') << '\n';
    for (int y = 0; y != graph.height; ++y) 
    {
        for (int x = 0; x != graph.width; ++x) 
        {
            if (x == 0)
            {
                cout << '|';
            }
            GridLocation id{ x, y };
            if (graph.walls.find(id) != graph.walls.end()) 
            {
                cout << " # ";
            }
            else if (start && id == *start) 
            {
                cout << " A ";
            }
            else if (goal && id == *goal) 
            {
                cout << " Z ";
            }
            else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) 
            {
                cout << " @ ";
            }
            else if (point_to != nullptr && point_to->count(id)) 
            {
                GridLocation next = (*point_to)[id];
                if (next.x == x + 1) { cout << " < "; }
                else if (next.x == x - 1) { cout << " > "; }
                else if (next.y == y + 1) { cout << " ^ "; }
                else if (next.y == y - 1) { cout << " v "; }
                else { cout << " * "; }
            }
            else if (distances != nullptr && distances->count(id)) 
            {
                cout << ' ' << left << setw(field_width - 1) << (*distances)[id];
            }
            else {
                cout << " . ";
            }
            if (x == graph.width - 1)
            {
                cout << '|';
            }
        }
        cout << '\n';
    }
    cout << string(field_width * graph.width, '~') << '\n';
}

/* GridWithWeights adds weaight values to SquareGrid structs
* The cost function will assign a random value between 2 and 5 
* to all of the nodes in the forest set.*/
struct GridWithWeights : SquareGrid 
{
    unordered_set<GridLocation> forests;
    GridWithWeights(int w, int h) : SquareGrid(w, h) {}
    double cost(GridLocation from_node, GridLocation to_node) const {
        int cost = rand() % 4 + 2;
        return forests.find(to_node) != forests.end() ? cost : 1;
    }
};

/* This function will randomly determine if any given cell is a forest or a wall
* it protects the start and end points from being a wall or forest*/
void add_features(GridWithWeights& grid, int width, int height, GridLocation start, GridLocation goal)
{
    for (int x = 0; x < width; ++x) 
    {
        for (int y = 0; y < height; ++y) 
        {
            if (!(x == start.x && y == start.y) && !(x == goal.x && y == goal.y))
            {
                // The constant in this if statement needs to be less than the constant in the following else if.
                // Changing these constants changes the frequency of walls and forests.
                if (rand() % 100 <= 40)
                {
                    grid.walls.insert(GridLocation{ x, y });
                }
                else if (rand() % 100 <= 66)
                {
                    grid.forests.insert(GridLocation{ x, y });
                }
            }
        }
    }
}

/*This function creates the grid and adds the features that make up the pathing field*/
GridWithWeights make_diagram(int width, int height, GridLocation start, GridLocation goal) 
{

    GridWithWeights grid(width, height);
    add_features(grid, width, height, start, goal);
    return grid;
}

/* This is a template that allows a priority queue to hold and organize Locations*/
template<typename T, typename priority_t>
struct PriorityQueue
{
    typedef pair<priority_t, T> PQElement;
    priority_queue<PQElement, vector<PQElement>, greater<PQElement>> elements;

    inline bool empty() const 
    {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) 
    {
        elements.emplace(priority, item);
    }

    T get() 
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

/* This function is used after the A* algorithm has been run
* it backtracks through the map of locations where each location
* came from so that the path that is taken can be visualized*/
template<typename Location>
vector<Location> reconstruct_path(
    Location start, Location goal,
    unordered_map<Location, Location> came_from,
    int height, int width
) {
    vector<Location> path;
    Location current = goal;
    
    while (current != start) 
    {
        path.push_back(current);
        current = came_from[current];
        /* When the path beomes larger than where the path came from
        * it means that there is no valid path.*/
        if (came_from.size() < path.size())
        {
            cout << "~~~~~~~~~~~~~~~No Path!~~~~~~~~~~~~~~~~" << endl;
            break;
        }
    }
    path.push_back(start); // optional
    reverse(path.begin(), path.end());
    return path;
}


/* The heuristic is what differentiates A* from Dijkstras algorithm
* this heuristic uses the Manhattan distance formula to determine
* straight line distance from start to finish. The heuristic must be
* admissable meaning that it never overestimates the true cost to the goal
* and it must be consistent meaning it never overestimates the actual 
* step cost.*/
inline double heuristic(GridLocation a, GridLocation b)
{
    return abs(a.x - b.x) + abs(a.y - b.y);
}

/* This is the actual A* algorithm. It requires the weighted grid that needs to
* be mapped, the start, and end points. It also takes a reference to a map that is
* used to reconstruct the path for visualization and a map that stores cost so far
* at each location
*/
template<typename Location, typename Graph>
void a_star_search
(Graph graph,
    Location start,
    Location goal,
    unordered_map<Location, Location>& came_from,
    unordered_map<Location, double>& cost_so_far)
{
    // The frontier is the adjacent nodes to all of the visited nodes.
    PriorityQueue<Location, double> frontier;
    // The frontier starts out only containing the start location
    frontier.put(start, 0);
    // start is the origin of the came from map
    came_from[start] = start;
    // cost at start will always be zero because no movement has occurred yet
    cost_so_far[start] = 0;

    /* The frontier will grow while there are unvisited nodes that are
    * passable that have not yet been visited and been assigned a cost.
    * If the frontier empties without finding the goal then there is 
    * no viable path. 
    */
    while (!frontier.empty()) 
    {
        // Get the current location from the frontier
        Location current = frontier.get();
        // End search when goal is reached.
        if (current == goal) 
        {
            break;
        }

        // Loop through the neighbors of the current node
        for (Location next : graph.neighbors(current)) 
        {
            /* Adds the current cost to the cost of the neighbor using the the
            * GridWithWeights cost function*/
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            /* Checks that the next locations cost has not already been calculated in
            * a previous iteration or that the new cost is less than the previously calculated
            * value that was reached in a prior iteration. */
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                // Set the running cost at the next node to the new calculated cost
                cost_so_far[next] = new_cost;
                // Calculate the Heuristic value to set the priority of the next node.
                double priority = new_cost + heuristic(next, goal);
                // Load the next node into the frontier for future iterations.
                frontier.put(next, priority);
                // Set the current node in the came from map for visualization
                came_from[next] = current;
            }
        }
    }
}


int main() {

    // Random generation of Grid size, start and end locations and output of this data
    srand((int)time(NULL));
    int height = rand() % 20 + 10;
    int width = rand() % 20 + 10;
    GridLocation start = { rand() % width , rand() % height };
    GridLocation goal = { rand() % width , rand() % height };
    cout << "Height: " << height << ", " << "width: " << width << "\n" 
        << "Start: (" << start.x << ", " << start.y << ")\n" 
        << "Goal: (" << goal.x << ", " << goal.y << ")" << endl;

    // Randomly generate grid using make_diagram
    GridWithWeights grid = make_diagram(width, height, start, goal);
    // Create map used to visualize the path taken.
    unordered_map<GridLocation, GridLocation> came_from;
    // Create map to store cumulative costs in the grid.
    unordered_map<GridLocation, double> cost_so_far;
    // Run the search
    a_star_search(grid, start, goal, came_from, cost_so_far);
    // Reconstruct path for visualization
    vector<GridLocation> path = reconstruct_path(start, goal, came_from, height, width);
    /* Output Visualizers first shows Path using '@' overlaid on the comulative cost
    * second shows just the cumulative costs so you can compare the values the path 
    * took to reach the destination. The third outputs a flow map from the origin to the goal. */
    draw_grid(grid, &cost_so_far, nullptr, &path, &start, &goal);
    cout << "\n";
    draw_grid(grid, &cost_so_far, nullptr, nullptr, &start, &goal);
    cout << "\n";
    draw_grid(grid, nullptr, &came_from, nullptr, &start, &goal);
}