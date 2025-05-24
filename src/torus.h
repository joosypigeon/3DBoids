#ifndef TORUS_H
#define TORUS_H

#include "raylib.h"
#include "raymath.h"
#include <math.h>
#include "boids.h"

extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;

void SetTorusDimensions(float major, float minor);
Mesh MyGenTorusMesh(int rings, int sides);


Vector3 get_torus_position(float u, float v);
Vector3 get_torus_normal(float u, float v);
Vector3 get_phi_tangent(float u, float v);
Vector3 get_theta_tangent(float u, float v);

void set_torus_coords(float u, float v);
Vector3 get_torus_position_fast();
Vector3 get_torus_normal_fast();
Vector3 get_theta_tangent_fast();
Vector3 get_phi_tangent_fast();
Matrix get_torus_transform(Boid *boid, float scale);

#endif // TORUS_H