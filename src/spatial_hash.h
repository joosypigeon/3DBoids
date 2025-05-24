#ifndef SPATIAL_HASH_H
#define SPATIAL_HASH_H

#include <stdbool.h>
#include "boids.h"

#define HASH_SIZE 10007
#define CELL_SIZE 50
#define CELL_WIDTH (SCREEN_WIDTH / CELL_SIZE)
#define CELL_HEIGHT (SCREEN_HEIGHT / CELL_SIZE)


#define INITIAL_MAX_BOIDS_PER_CELL 1024 // Tweak as needed

typedef struct {
    Vector2 alignment;
    Vector2 cohesion;
    Vector2 separation;
    int neighborCount;
    int nearNeighborCount;
} FlockForces;

typedef struct {
    int length;
    int max_length;
    Boid** boids;  // dynamically allocated array
} HashCell;

extern HashCell hash_table[HASH_SIZE];

void init_spatial_hash(void);
void clear_spatial_hash(void);
unsigned int hash_cell(int cell_x, int cell_y);

FlockForces ComputeFlockForces(Boid *boid);
Vector2 PreditorAjustment();
void DrawNearestNeighbor(Boid *boid);

Vector2 Vector2SubtractTorus(Vector2 a, Vector2 b);
float DistanceOnTorus(Vector2 a, Vector2 b);
float DistanceOnTorusSquared(Vector2 a, Vector2 b);

Boid *FindNearestBoid(Vector2 position);

void DrawCells(Vector2 position);
#endif // SPATIAL_HASH_H

