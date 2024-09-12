#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "spiral_search.h"

#define SAFETY_MARGIN 0.2
#define GRID_SIZE_X 20
#define GRID_SIZE_Y 20
#define GRID_SIZE_Z 10
#define GRID_CELL_SIZE 0.2
#define INF 1e9
#define OBSTACLE_PENALTY 1000.0
#define GRID_CHAR '.'
#define OBSTACLE_CHAR 'O'
#define OBSTACLE_MARGIN_CHAR '#'
#define PATH_CHAR '*'
#define START_CHAR 'S'
#define GOAL_CHAR 'G'

typedef struct Node {
    coord_t coord;
    float gCost, hCost, fCost;
    struct Node* parent;
} Node;

typedef struct Obstacle {
    coord_t min, max;
    float margin;
} Obstacle;

float heuristic(coord_t a, coord_t b);
int isInBounds(coord_t coord);
Node* createNode(coord_t coord, float gCost, float hCost, Node* parent);
int isBlocked(coord_t* node, Obstacle* obstacle);
float calculateObstaclePenalty(coord_t* node, Obstacle* obstacle);
float AStar(coord_t start, coord_t goal, Node* path[], int* pathLength, Obstacle* obstacles[], int numObstacles);
void printPath(Node* path[], int pathLength);
void drawPath(Node* path[], int pathLength, Obstacle* obstacles[], int numObstacles);
void scaleCoords(coord_t *realCoord);
void rescaleCoords(coord_t *realCoord);
void scaleObstacle(Obstacle* obstacle);
void rescaleObstacle(Obstacle* obstacle);

#endif