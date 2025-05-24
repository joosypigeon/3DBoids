
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <omp.h>

#include "boids.h"
#include "torus.h"
#include "spatial_hash.h"
#include "normal_random.h"



Boid boids[MAX_BOIDS + 1]; // +1 for predator

Vector2 Vector2SubtractTorus(Vector2 a, Vector2 b) {
    Vector2 diff = { a.x - b.x, a.y - b.y };

    if (diff.x >  HALF_SCREEN_WIDTH) diff.x -= SCREEN_WIDTH;
    if (diff.x < -HALF_SCREEN_WIDTH) diff.x += SCREEN_WIDTH;

    if (diff.y >  HALF_SCREEN_HEIGHT) diff.y -= SCREEN_HEIGHT;
    if (diff.y < -HALF_SCREEN_HEIGHT) diff.y += SCREEN_HEIGHT;

    return diff;
}

float DistanceOnTorus(Vector2 a, Vector2 b)
{
    float dx = fabsf(a.x - b.x);
    float dy = fabsf(a.y - b.y);

    if (dx > HALF_SCREEN_WIDTH) dx = SCREEN_WIDTH - dx;
    if (dy > HALF_SCREEN_HEIGHT) dy = SCREEN_HEIGHT - dy;

    return sqrtf(dx * dx + dy * dy);
}

float DistanceOnTorusSquared(Vector2 a, Vector2 b)
{
    float dx = fabsf(a.x - b.x);
    float dy = fabsf(a.y - b.y);

    if (dx > HALF_SCREEN_WIDTH) dx = SCREEN_WIDTH - dx;
    if (dy > HALF_SCREEN_HEIGHT) dy = SCREEN_HEIGHT - dy;

    return dx * dx + dy * dy;
}

void InitBoids() {
    // Initialize spatial hash
    init_spatial_hash();

    // Initialize boids
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].index = i;
        boids[i].position = (Vector2){ GetRandomValue(0, SCREEN_WIDTH), GetRandomValue(0, SCREEN_HEIGHT) };
        float angle = GetRandomValue(0, 360) * DEG2RAD;
        float speed = random_normal(4.0f, 3.0f);
        boids[i].velocity = Vector2Scale((Vector2){ cosf(angle), sinf(angle) }, speed);
        boids[i].isPredator = false;
        boids[i].neighborCount = -1;
        boids[i].nearNeighborCount = -1;
        insert_boid(&boids[i]);
    }
    // Predator
    boids[PREDATOR_INDEX].position = (Vector2){ HALF_SCREEN_WIDTH, HALF_SCREEN_HEIGHT };
    printf("Predator position: (%.2f, %.2f)\n", boids[PREDATOR_INDEX].position.x, boids[PREDATOR_INDEX].position.y);
    boids[PREDATOR_INDEX].velocity = (Vector2){ PREDATOR_SPEED, PREDATOR_SPEED };
    boids[PREDATOR_INDEX].isPredator = true;
}

Vector2 Vector2Wrap(Vector2 v, float width, float height)
{
    if (v.x < 0) v.x += width;
    else if (v.x >= width) v.x -= width;

    if (v.y < 0) v.y += height;
    else if (v.y >= height) v.y -= height;

    return v;
}

void UpdateBoids(float alignmentWeight, float cohesionWeight, float separationWeight)
{
    // Parallel update stage
    #pragma omp parallel for schedule(static)
    for (size_t boid_index = 0; boid_index < MAX_BOIDS; boid_index++) { 
        Boid* self = &boids[boid_index];

        // Initialize updates
        self->velocity_update = self->velocity;
        self->predated = false;

        // Compute flocking forces
        // ComputeFlockForces() is a function that computes the alignment, cohesion, and separation forces
        FlockForces forces = ComputeFlockForces(self);
        self->neighborCount = forces.neighborCount;
        self->nearNeighborCount = forces.nearNeighborCount;

        // Apply flocking behaviour
        if (forces.neighborCount > 0) {
            Vector2 align_force = Vector2Subtract(forces.alignment, self->velocity);
            self->velocity_update = Vector2Add(self->velocity_update, Vector2Scale(align_force, MATCH_FACTOR * alignmentWeight));

            Vector2 cohesion_force = Vector2Subtract(forces.cohesion, self->position);
            self->velocity_update = Vector2Add(self->velocity_update, Vector2Scale(cohesion_force, CENTER_FACTOR * cohesionWeight));
        }
        self->velocity_update = Vector2Add(self->velocity_update, Vector2Scale(forces.separation, AVOID_FACTOR * separationWeight));
    }

    // Adjust predator to move towards densest nearby area of boids.
    // Also adjust boids to avoid predator.
    boids[PREDATOR_INDEX].velocity = Vector2ClampValue(
                                        Vector2Add(boids[PREDATOR_INDEX].velocity, PreditorAjustment()),
                                        MIN_SPEED, PREDATOR_SPEED);
    boids[PREDATOR_INDEX].position = Vector2Wrap(
                                        Vector2Add(
                                            boids[PREDATOR_INDEX].position,
                                            Vector2Scale(
                                                boids[PREDATOR_INDEX].velocity,
                                                GetFrameTime() * 60.0f)),
                                        SCREEN_WIDTH, SCREEN_HEIGHT);

    // Commit updates and rebuild spatial hash (serial)
    clear_spatial_hash();
    for (size_t i = 0; i < MAX_BOIDS; i++) {
        boids[i].velocity = Vector2ClampValue(boids[i].velocity_update, MIN_SPEED, MAX_SPEED);
        float frameTime = GetFrameTime();
        if (frameTime == 0.0f) {
            frameTime = 1.0f / 60.0f; // fix wierd behaviour at startup
        }
        boids[i].position = Vector2Wrap(
                                Vector2Add(
                                    boids[i].position,
                                    Vector2Scale(
                                        boids[i].velocity,
                                        frameTime * 60.0f)),
                                SCREEN_WIDTH, SCREEN_HEIGHT);
        insert_boid(&boids[i]);
    }
    insert_boid(&boids[MAX_BOIDS]);

}

int number_drawn = 0;
Vector3 Vector2ToVector3(Vector2 v) {
    return (Vector3){ v.x, 0.0f, v.y};
}

Vector3 Shift(Vector3 position)
{
    return (Vector3){position.x - HALF_SCREEN_WIDTH, position.y + 100.0f, position.z - HALF_SCREEN_HEIGHT};  
}


void DrawBoid3D(Boid *boid) {
    number_drawn++;
    Vector3 position = Shift(Vector2ToVector3(boid->position));
    Vector3 velocity = Vector2ToVector3(boid->velocity);
    Vector3 dir = Vector3Normalize(velocity);

    Vector3 forward = {1, 0, 0};

    // Cross product gives the rotation axis
    Vector3 axis = Vector3CrossProduct(forward, dir);
    float angle = acosf(Vector3DotProduct(forward, dir));

    if (Vector3Length(axis) < 0.001f) axis = (Vector3){ 0, 1, 0 }; // fallback

    DrawModelEx(dart, position, axis, RAD2DEG * angle, (Vector3){ 3.0f, 3.0f, 3.0f }, WHITE);
}

void DrawBoid3DTorus(Boid *boid) {
    number_drawn++;
    float scale = 3.0f;
    dart.transform = get_torus_transform(boid, scale);
    DrawModel(dart, (Vector3){0, 0, 0}, 1.0f, WHITE);
}

void DrawPreditor3D() {
    number_drawn++;

    Boid *predator = &boids[MAX_BOIDS];

    Vector3 position = Shift(Vector2ToVector3(predator->position));
    Vector3 velocity = Vector2ToVector3(predator->velocity);
    Vector3 dir = Vector3Normalize(velocity);

    Vector3 forward = {1, 0, 0};

    // Cross product gives the rotation axis
    Vector3 axis = Vector3CrossProduct(forward, dir);
    float angle = acosf(Vector3DotProduct(forward, dir));

    if (Vector3Length(axis) < 0.001f) axis = (Vector3){ 0, 1, 0 }; // fallback

    DrawModelEx(dart, position, axis, RAD2DEG * angle, (Vector3){ 10.0f, 10.0f, 10.0f }, RED);
}

void DrawPreditor3DTorus() {
    number_drawn++;

    Boid *predator = &boids[MAX_BOIDS];
    float scale = 10.0f;
    dart.transform = get_torus_transform(predator, scale);
    DrawModel(dart, (Vector3){0, 0, 0}, 1.0f, RED);
}

void DrawMouse(Boid boid) {
    // Draw main circle
    DrawCircleLinesV(boid.position, MOUSE_RADIUS, BLUE);

    // Draw center dot
    DrawCircleV(boid.position, BOID_RADIUS, RED);
}

void DrawBoids3D() {
    number_drawn = 0;
    Matrix transform = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    dart.transform = transform;
    for (int i = 0; i < MAX_BOIDS; i++) DrawBoid3D(&boids[i]);
    DrawPreditor3D();
    //if (mousePressed) DrawMouse(boids[MOUSE_INDEX]);
}

void DrawBoids3DTorus() {
    number_drawn = 0;
    for (int i = 0; i < MAX_BOIDS; i++) DrawBoid3DTorus(&boids[i]);
    DrawPreditor3DTorus();
    //if (mousePressed) DrawMouse(boids[MOUSE_INDEX]);
}

void DrawNearestNeighborNetwork(){
    for (int i = 0; i < MAX_BOIDS; i++) DrawNearestNeighbor(&boids[i]);
}

