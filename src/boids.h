#ifndef BOIDS_H
#define BOIDS_H
#include <stdbool.h>

#include "raylib.h"
#include "raymath.h"

#define MAX_BOIDS 1000
#define PREDATOR_INDEX MAX_BOIDS 
#define MOUSE_INDEX (MAX_BOIDS + 1)


#define NEIGHBOR_RADIUS 50.0f
#define PROTECTED_RADIUS 10.0f
#define PREDATOR_RADIUS 50.0f
#define PREDATOR_VISUAL_RADIUS (PREDATOR_RADIUS * 3.0f)
#define MOUSE_RADIUS 120.0f

#define AVOID_FACTOR 0.15f
#define MATCH_FACTOR 0.1f
#define CENTER_FACTOR 0.001f
#define TURN_FACTOR 0.2f
#define PREDATOR_AVOID_FACTOR 25.0f
#define MOUSE_ATTRACTION_FACTOR 0.5f

#define MAX_SPEED 4.5f
#define MIN_SPEED 1.0f
#define PREDATOR_SPEED 7.0f

#define BOID_RADIUS 2.0f

#define WRAP_MOD(a, m) (((a) % (m) + (m)) % (m))

extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;
extern float HALF_SCREEN_WIDTH;
extern float HALF_SCREEN_HEIGHT;

extern Model dart;

extern bool drawFullGlyph;
extern bool drawDensity;
extern bool nearestNeighboursNetwork;


// Boid structure
typedef struct Boid {
    Vector2 position;
    Vector2 velocity;
    Vector2 position_update;
    Vector2 velocity_update;
    int neighborCount;
    int nearNeighborCount;
    bool predated;
    bool isPredator;
} Boid;

extern Boid *debugBoid;

extern Boid boids[MAX_BOIDS+2];

typedef struct BoidNode {
    Boid* boid;
    struct BoidNode* next;
} BoidNode;

void init_spatial_hash(void);
void clear_spatial_hash(void);
void insert_boid(Boid* p);

void InitBoids(void);
void UpdateBoids(float alignmentWeight, float cohesionWeight, float separationWeight);
void DrawBoids(void);
void DrawBoids3D(void);
void DrawNearestNeighborNetwork(void);
Vector3 Vector2ToVector3(Vector2 v);
Vector3 Shift(Vector3 position);

extern int number_drawn;
extern bool mousePressed;
extern Model transparentSphere;
#endif // BOIDS_H
