#ifndef TORUS_H
#define TORUS_H

#include "raylib.h"
#include "raymath.h"
#include <math.h>

Mesh GenTorusMesh(float R, float r, int rings, int sides);

#endif // TORUS_H