#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "path_planner.h"

#define _USE_MATH_DEFINES

enum obstacle_check
{
    OUTSIDE,
    MARGIN,
    BLOCKED
};

// Heuristic function: Euclidean distance
float heuristic(coord_t a, coord_t b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Check if coordinates are within grid bounds
int isInBounds(coord_t coord)
{
    return (coord.x >= 0 && coord.x < GRID_SIZE_X &&
            coord.y >= 0 && coord.y < GRID_SIZE_Y &&
            coord.z >= 0 && coord.z < GRID_SIZE_Z);
}

// Create a new node
Node* createNode(coord_t coord, float gCost, float hCost, Node* parent)
{
    Node* newNode = (Node*)malloc(sizeof(Node));
    newNode->coord = coord;
    newNode->gCost = gCost;
    newNode->hCost = hCost;
    newNode->fCost = gCost + hCost;
    newNode->parent = parent;
    return newNode;
}

// Scale real-world coordinates to grid coordinates
void scaleCoords(coord_t *realCoord)
{
    realCoord->x = round((realCoord->x) / GRID_CELL_SIZE);
    realCoord->y = round((realCoord->y) / GRID_CELL_SIZE);
    realCoord->z = round((realCoord->z) / GRID_CELL_SIZE);
}

// Rescale grid coordinates back to real-world coordinates
void rescaleCoords(coord_t *gridCoord)
{
    gridCoord->x *= GRID_CELL_SIZE;
    gridCoord->y *= GRID_CELL_SIZE;
    gridCoord->z *= GRID_CELL_SIZE;
}
void scaleObstacle(Obstacle* obstacle)
{
    scaleCoords(&obstacle->min);
    scaleCoords(&obstacle->max);
    obstacle->margin /= GRID_CELL_SIZE;
}
void rescaleObstacle(Obstacle* obstacle)
{
    rescaleCoords(&obstacle->min);
    rescaleCoords(&obstacle->max);
    obstacle->margin *= GRID_CELL_SIZE;
}

// Calculate whether a node is close to or inside an obstacle
int isBlocked(coord_t* node, Obstacle* obstacle)
{
    float margin = obstacle->margin;

    if (node->x >= obstacle->min.x && node->x <= obstacle->max.x &&
        node->y >= obstacle->min.y && node->y <= obstacle->max.y &&
        node->z >= obstacle->min.z && node->z <= obstacle->max.z)
    {
        return BLOCKED;
    }
    else if (node->x >= obstacle->min.x - margin && node->x <= obstacle->max.x + margin &&
        node->y >= obstacle->min.y - margin && node->y <= obstacle->max.y + margin &&
        node->z >= obstacle->min.z - margin && node->z <= obstacle->max.z + margin)
    {
        return MARGIN;
    }

    return OUTSIDE;
}

float calculateObstaclePenalty(coord_t* node, Obstacle* obstacle)
{
    float dx = 0, dy = 0, dz = 0;
    float penalty = OBSTACLE_PENALTY;
    float margin = obstacle->margin;

    // Compute the shortest distance in the x-axis
    if (node->x < obstacle->min.x - margin)
    {
        dx = node->x - obstacle->min.x + margin;
    }
    else if (node->x > obstacle->max.x + margin)
    {
        dx = node->x - obstacle->max.x - margin;
    }

    // Compute the shortest distance in the y-axis
    if (node->y < obstacle->min.y - margin)
    {
        dy = node->y - obstacle->min.y + margin;
    }
    else if (node->y > obstacle->max.y + margin)
    {
        dy = node->y - obstacle->max.y - margin;
    }

    // Compute the shortest distance in the z-axis
    if (node->z < obstacle->min.z - margin)
    {
        dz = node->z - obstacle->min.z + margin;
    }
    else if (node->z > obstacle->max.z + margin)
    {
        dz = node->z - obstacle->max.z - margin;
    }

    // Calculate Euclidean distance to the obstacle boundary including the margin
    float distance = sqrt(dx*dx + dy*dy + dz*dz);

    // If the node is within the margin but outside the obstacle, apply a scaled penalty
    // Scale the penalty within the margin zone (closer to obstacle -> higher penalty)
    float distanceToMargin = margin - distance;
    if (distanceToMargin > 0.0)
    {
        penalty *= (1.0 - (distanceToMargin / margin));  // Scale penalty linearly
    }

    return penalty;
}

// Implement A* pathfinding algorithm
float AStar(coord_t start, coord_t goal, Node* path[], int* pathLength, Obstacle* obstacles[], int numObstacles)
{
    // Scale start and goal points to grid
    coord_t scaledStart = start;
    coord_t scaledGoal = goal;
    scaleCoords(&scaledStart);
    scaleCoords(&scaledGoal);
    
    // Scale obstacles' boundaries and margins
    for (int j = 0; j < numObstacles; j++)
    {
        scaleObstacle(obstacles[j]);
    }

    Node* openList[GRID_SIZE_X * GRID_SIZE_Y * GRID_SIZE_Z]; // 
    Node* closedList[GRID_SIZE_X * GRID_SIZE_Y * GRID_SIZE_Z];
    int openListSize = 0;
    int closedListSize = 0;

    // Initialize lists
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y * GRID_SIZE_Z; i++)
    {
        openList[i] = NULL;
        closedList[i] = NULL;
    }

    // Create start node
    Node* startNode = createNode(scaledStart, 0, heuristic(scaledStart, scaledGoal), NULL);
    openList[openListSize++] = startNode;

    while (openListSize > 0)
    {
        // Find node with the smallest fCost
        int currentIndex = 0;
        for (int i = 1; i < openListSize; i++)
        {
            if (openList[i]->fCost < openList[currentIndex]->fCost)
            {
                currentIndex = i;
            }
        }
        Node* currentNode = openList[currentIndex];
        openList[currentIndex] = openList[--openListSize];

        // Check if goal is reached
        if (currentNode->coord.x == scaledGoal.x &&
            currentNode->coord.y == scaledGoal.y &&
            currentNode->coord.z == scaledGoal.z)
        {
            // Backtrack to find path
            *pathLength = 0;
            while (currentNode != NULL)
            {
                path[(*pathLength)++] = currentNode;
                currentNode = currentNode->parent;
            }
            // Reverse the path array
            for (int i = 0; i < *pathLength / 2; i++)
            {
                Node* temp = path[i];
                path[i] = path[*pathLength - i - 1];
                path[*pathLength - i - 1] = temp;
            }

            // Rescale the path coordinates and adjust the relative yaw value 
            // depending on changes in the direction of travel
            // 

            float prev_yaw = path[0]->coord.yaw;
            rescaleCoords(&path[0]->coord);
            
            for (int i = 1; i < *pathLength - 1; i++)
            {
                // Rescale the obstacle coordinates
                rescaleCoords(&path[i]->coord);

                // Calculate the direction vector between the current node and the next node
                float dx = path[i]->coord.x - path[i - 1]->coord.x;
                float dy = path[i]->coord.y - path[i - 1]->coord.y;
                // Calculate the yaw angle (direction) in radians, using atan2 to get the angle between the nodes
                float yaw_difference = atan2(dy, dx) - prev_yaw;

                if(fabs(yaw_difference) < 1e-3)
                {
                    path[i]->coord.yaw = 0;
                }
                else
                {
                    path[i]->coord.yaw = yaw_difference;
                }

                prev_yaw = atan2(dy, dx);
            }
            path[*pathLength - 1]->coord.yaw = goal.yaw - prev_yaw;


            for (int j = 0; j < numObstacles; j++)
            {
                rescaleObstacle(obstacles[j]);
            }

            printf("Path found with total cost: %f\n", path[*pathLength - 1]->gCost); // Debugging line
            return path[*pathLength - 1]->gCost; // Return the total cost
        }

        // Add current node to closed list
        closedList[closedListSize++] = currentNode;

        // Generate neighbors (6-connectivity)
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0) continue; // Skip the current node
                    coord_t neighborCoord = {
                        currentNode->coord.x + dx,
                        currentNode->coord.y + dy,
                        currentNode->coord.z + dz
                    };

                    if (isInBounds(neighborCoord))
                    {
                        // Check if neighbor is in closed list
                        int inClosedList = 0;
                        for (int j = 0; j < closedListSize; j++)
                        {
                            if (closedList[j]->coord.x == neighborCoord.x &&
                                closedList[j]->coord.y == neighborCoord.y &&
                                closedList[j]->coord.z == neighborCoord.z)
                            {
                                inClosedList = 1;
                                break;
                            }
                        }
                        if (inClosedList) continue;

                        // Calculate gCost, hCost, and create new node
                        float gCost = currentNode->gCost + sqrt(dx*dx + dy*dy + dz*dz); // Assuming a step cost based on Euclidean distance

                        // Calculate penalty based on all obstacles
                        float penalty = 0;
                        for (int j = 0; j < numObstacles; j++)
                        {
                            int obstacleStatus = isBlocked(&neighborCoord, obstacles[j]);
                            if (obstacleStatus == BLOCKED)
                            {
                                penalty = OBSTACLE_PENALTY; // Full penalty if blocked
                            }
                            else if (obstacleStatus == MARGIN)
                            {
                                // pintf("Obstacle margin detected! Node: (%f, %f, %f)\n", neighborCoord.x, neighborCoord.y, neighborCoord.z);
                                // penalty = calculateObstaclePenalty(&neighborCoord, obstacles[j]); // Scaled penalty
                                penalty = 100;
                            }
                        }

                        gCost += penalty; // Update gCost with penalty

                        float hCost = heuristic(neighborCoord, scaledGoal);
                        Node* neighborNode = createNode(neighborCoord, gCost, hCost, currentNode);

                        // Check if neighbor is in open list
                        int inOpenList = 0;
                        for (int j = 0; j < openListSize; j++)
                        {
                            if (openList[j]->coord.x == neighborCoord.x &&
                                openList[j]->coord.y == neighborCoord.y &&
                                openList[j]->coord.z == neighborCoord.z)
                            {
                                inOpenList = 1;
                                if (neighborNode->gCost < openList[j]->gCost)
                                {
                                    openList[j]->gCost = neighborNode->gCost;
                                    openList[j]->fCost = openList[j]->gCost + openList[j]->hCost;
                                    openList[j]->parent = currentNode;
                                }
                                free(neighborNode);
                                break;
                            }
                        }
                        if (!inOpenList)
                        {
                            openList[openListSize++] = neighborNode;
                        }
                    }
                }
            }
        }
    }
    for (int j = 0; j < numObstacles; j++)
    {
        rescaleObstacle(obstacles[j]);
    }
    // No path found
    printf("No path found.\n"); // Debugging line
    return INF;
}

// Print the path from the end node to the start node
void printPath(Node* path[], int pathLength) {
    printf("Path :\n");
    for (int i = 0; i < pathLength; i++)
    {
        printf("Node %d: (%f, %f, %f, %f)\n", i + 1, path[i]->coord.x, path[i]->coord.y, path[i]->coord.z, path[i]->coord.yaw*180/M_PI);
    }
    printf("End\n");
}

// Function to draw the path on a 2D grid
void drawPath(Node* path[], int pathLength, Obstacle* obstacles[], int numObstacles)
{
    // Create a 2D grid with center origin
    char grid[GRID_SIZE_Y][GRID_SIZE_X];
    coord_t start = {round(path[0]->coord.x), round(path[0]->coord.y)};
    coord_t goal = {round(path[pathLength-1]->coord.x), round(path[pathLength-1]->coord.y)};
    coord_t gridcoord = {0};
    for(int i = 0; i < numObstacles; i++)
    {
        scaleObstacle(obstacles[i]);
        obstacles[i]->min.x += GRID_SIZE_X/2 - 1;
        obstacles[i]->min.y += GRID_SIZE_Y/2 - 1;
        obstacles[i]->max.x += GRID_SIZE_X/2 - 1;
        obstacles[i]->max.y += GRID_SIZE_Y/2 - 1;
        // printf("Scaled min(%f, %f, %f)\n", obstacles[i]->min.x, obstacles[i]->min.y, obstacles[i]->min.z);
        // printf("Scaled max(%f, %f, %f)\n", obstacles[i]->max.x, obstacles[i]->max.y, obstacles[i]->max.z);

    }

    // Initialize the grid
    for (int x = 0; x < GRID_SIZE_X; x++)
    {

        for (int y = 0; y < GRID_SIZE_Y; y++)
        {
            gridcoord.x = x;
            gridcoord.y = y;
            grid[y][x] = '.';

            // Draw out obstacles
            for(int i = 0; i < numObstacles; i++)
            {
                switch (isBlocked(&gridcoord, obstacles[i]))
                {
                    case BLOCKED:
                        grid[y][x] = 'O';
                        break;
                    case MARGIN:
                        grid[y][x] = '#';
                        break;
                    case OUTSIDE:
                        break;
                }
            }
            if(gridcoord.x == start.x + GRID_SIZE_X/2 - 1 && gridcoord.y == start.y + GRID_SIZE_Y/2 - 1 )
            {
                grid[y][x] = 'S';
            }
            else if(gridcoord.x == goal.x + GRID_SIZE_X/2 - 1 && gridcoord.y == goal.y + GRID_SIZE_Y/2 - 1)
            {
                grid[y][x] = 'G';
            }   
            for (int i = 1; i < pathLength - 1; i++)
            {
                int path_x = round(path[i]->coord.x/GRID_CELL_SIZE);
                int path_y = round(path[i]->coord.y/GRID_CELL_SIZE);
                if(gridcoord.x == path_x + GRID_SIZE_X/2 - 1 && gridcoord.y == path_y + GRID_SIZE_Y/2 - 1)
                {
                    grid[y][x] = '*';
                }
            }              
        }
    }
    
    for(int i = 0; i < numObstacles; i++)
    {
        obstacles[i]->min.x -= (GRID_SIZE_X/2 - 1);
        obstacles[i]->min.y -= (GRID_SIZE_Y/2 - 1);
        obstacles[i]->max.x -= (GRID_SIZE_X/2 - 1);
        obstacles[i]->max.y -= (GRID_SIZE_Y/2 - 1);
        rescaleObstacle(obstacles[i]);
    }
    // Print the grid (inverted y-axis for better visual)
    printf("Grid representation:\n");
    for (int x = GRID_SIZE_X - 1; x >= 0; x--) {
        for (int y = 0; y < GRID_SIZE_Y; y++) {
            printf("%c ", grid[y][x]);
        }
        printf("\n");
    }
}

float maxArray(float array[], size_t size)
{
    float maxValue = array[0];
    for (size_t i = 1; i < size; i++)
    {
        if (array[i] > maxValue)
        {
            maxValue = array[i];
        }
    }
    return maxValue;
}
