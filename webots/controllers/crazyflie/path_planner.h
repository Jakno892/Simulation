#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "spiral_search.h"

#define GRID_SIZE 30 // Size of the grid (NxNxN)
#define INF 1e10 // A large number to represent infinity

typedef struct Node {
    coord_t coord;
    double gCost, hCost, fCost;
    struct Node* parent;
} Node;

double heuristic(coord_t a, coord_t b);
int isInBounds(coord_t coord);
Node* createNode(coord_t coord, double gCost, double hCost, Node* parent);
double AStar(coord_t start, coord_t goal, Node* path[], int* pathLength);
void printPath(Node* path[], int pathLength);
coord_t scaleCoords(coord_t realCoord, coord_t start, coord_t goal);
coord_t rescaleCoords(coord_t gridCoord, coord_t start, coord_t goal);

#endif