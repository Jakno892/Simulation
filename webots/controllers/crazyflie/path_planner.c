#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "path_planner.h"

// Heuristic function: Euclidean distance considering yaw
double heuristic(coord_t a, coord_t b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    // Adjust heuristic with yaw difference
    double yaw_diff = fabs(a.yaw - b.yaw);
    return distance + yaw_diff;
}

// Check if coordinates are within grid bounds
int isInBounds(coord_t coord) {
    return (coord.x >= 0 && coord.x < GRID_SIZE &&
            coord.y >= 0 && coord.y < GRID_SIZE &&
            coord.z >= 0 && coord.z < GRID_SIZE);
}

// Create a new node
Node* createNode(coord_t coord, double gCost, double hCost, Node* parent) {
    Node* newNode = (Node*)malloc(sizeof(Node));
    newNode->coord = coord;
    newNode->gCost = gCost;
    newNode->hCost = hCost;
    newNode->fCost = gCost + hCost;
    newNode->parent = parent;
    return newNode;
}

// Scale real-world coordinates to grid coordinates
coord_t scaleCoords(coord_t realCoord, coord_t start, coord_t goal) {
    coord_t scaled;
    scaled.x = ((realCoord.x - start.x) / (goal.x - start.x)) * (GRID_SIZE - 1);
    scaled.y = ((realCoord.y - start.y) / (goal.y - start.y)) * (GRID_SIZE - 1);
    scaled.z = ((realCoord.z - start.z) / (goal.z - start.z)) * (GRID_SIZE - 1);
    scaled.yaw = realCoord.yaw;  // Yaw doesn't need scaling
    return scaled;
}

// Rescale grid coordinates back to real-world coordinates
coord_t rescaleCoords(coord_t gridCoord, coord_t start, coord_t goal) {
    coord_t rescaled;
    rescaled.x = start.x + (gridCoord.x / (GRID_SIZE - 1)) * (goal.x - start.x);
    rescaled.y = start.y + (gridCoord.y / (GRID_SIZE - 1)) * (goal.y - start.y);
    rescaled.z = start.z + (gridCoord.z / (GRID_SIZE - 1)) * (goal.z - start.z);
    rescaled.yaw = gridCoord.yaw;  // Yaw doesn't need rescaling
    return rescaled;
}

// Implement A* pathfinding algorithm
double AStar(coord_t start, coord_t goal, Node* path[], int* pathLength) {
    // Scale start and goal points to grid
    coord_t scaledStart = scaleCoords(start, start, goal);
    coord_t scaledGoal = scaleCoords(goal, start, goal);

    Node* openList[GRID_SIZE * GRID_SIZE * GRID_SIZE * 10]; // Adjust size for yaw variations
    Node* closedList[GRID_SIZE * GRID_SIZE * GRID_SIZE * 10];
    int openListSize = 0;
    int closedListSize = 0;

    // Initialize
    for (int i = 0; i < GRID_SIZE * GRID_SIZE * GRID_SIZE * 10; i++) {
        openList[i] = NULL;
        closedList[i] = NULL;
    }

    // Create start node
    Node* startNode = createNode(scaledStart, 0, heuristic(scaledStart, scaledGoal), NULL);
    openList[openListSize++] = startNode;

    while (openListSize > 0) {
        // Find node with the smallest fCost
        int currentIndex = 0;
        for (int i = 1; i < openListSize; i++) {
            if (openList[i]->fCost < openList[currentIndex]->fCost) {
                currentIndex = i;
            }
        }
        Node* currentNode = openList[currentIndex];
        openList[currentIndex] = openList[--openListSize];

        // Check if goal is reached
        if (currentNode->coord.x == scaledGoal.x &&
            currentNode->coord.y == scaledGoal.y &&
            currentNode->coord.z == scaledGoal.z) {
            // Backtrack to find path
            *pathLength = 0;
            while (currentNode != NULL) {
                path[(*pathLength)++] = currentNode;
                currentNode = currentNode->parent;
            }
            // Reverse the path array
            for (int i = 0; i < *pathLength / 2; i++) {
                Node* temp = path[i];
                path[i] = path[*pathLength - i - 1];
                path[*pathLength - i - 1] = temp;
            }

            // Interpolate yaw values linearly
            double yaw_initial = start.yaw;
            double yaw_final = goal.yaw;
            double yaw_diff = yaw_final - yaw_initial;
            for (int i = 0; i < *pathLength; i++) {
                path[i]->coord.yaw = yaw_initial + i * yaw_diff / (*pathLength - 1);
            }

            // Rescale the path coordinates back to the original real-world coordinates
            for (int i = 0; i < *pathLength; i++) {
                path[i]->coord = rescaleCoords(path[i]->coord, start, goal);
            }

            printf("Path found with total cost: %f\n", path[0]->gCost); // Debugging line
            return path[0]->gCost; // Return the total cost
        }

        // Add current node to closed list
        closedList[closedListSize++] = currentNode;

        // Generate neighbors (6-connectivity with yaw variations)
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dy == 0 && dz == 0) continue; // Skip the current node
                    for (int dyaw = -1; dyaw <= 1; dyaw++) { // Vary yaw by ±1
                        coord_t neighborCoord = {
                            currentNode->coord.x + dx,
                            currentNode->coord.y + dy,
                            currentNode->coord.z + dz,
                            fmod(currentNode->coord.yaw + dyaw, 360.0) // Normalize yaw
                        };

                        if (isInBounds(neighborCoord)) {
                            // Check if neighbor is in closed list
                            int inClosedList = 0;
                            for (int j = 0; j < closedListSize; j++) {
                                if (closedList[j]->coord.x == neighborCoord.x &&
                                    closedList[j]->coord.y == neighborCoord.y &&
                                    closedList[j]->coord.z == neighborCoord.z &&
                                    fabs(closedList[j]->coord.yaw - neighborCoord.yaw) < 1e-6) {
                                    inClosedList = 1;
                                    break;
                                }
                            }
                            if (inClosedList) continue;

                            // Calculate gCost, hCost, and create new node
                            double gCost = currentNode->gCost + sqrt(dx * dx + dy * dy + dz * dz); // Assuming a step cost based on Euclidean distance
                            double hCost = heuristic(neighborCoord, scaledGoal);
                            Node* neighborNode = createNode(neighborCoord, gCost, hCost, currentNode);

                            // Check if neighbor is in open list
                            int inOpenList = 0;
                            for (int j = 0; j < openListSize; j++) {
                                if (openList[j]->coord.x == neighborCoord.x &&
                                    openList[j]->coord.y == neighborCoord.y &&
                                    openList[j]->coord.z == neighborCoord.z &&
                                    fabs(openList[j]->coord.yaw - neighborCoord.yaw) < 1e-6) {
                                    inOpenList = 1;
                                    if (neighborNode->gCost < openList[j]->gCost) {
                                        openList[j]->gCost = neighborNode->gCost;
                                        openList[j]->fCost = openList[j]->gCost + openList[j]->hCost;
                                        openList[j]->parent = currentNode;
                                    }
                                    free(neighborNode);
                                    break;
                                }
                            }
                            if (!inOpenList) {
                                openList[openListSize++] = neighborNode;
                            }
                        }
                    }
                }
            }
        }
    }

    // No path found
    printf("No path found.\n"); // Debugging line
    return INF;
}

// Print the path from the end node to the start node
void printPath(Node* path[], int pathLength) {
    printf("Path :\n");
    for (int i = 0; i < pathLength; i++) {
        printf("Node %d: (%f, %f, %f, %f)\n", i + 1, path[i]->coord.x, path[i]->coord.y, path[i]->coord.z, path[i]->coord.yaw);
    }
    printf("End\n");
}

float maxArray(float array[], size_t size)
{
  int i;
  float maxValue;
  maxValue = array[0];
  //find the largest no
  for (i = 0; i < size; i++)
  {
    if (array[i]>maxValue)
    {
      maxValue = array[i];
    }
  }   
  return maxValue;
}