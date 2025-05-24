#include "torus.h"

typedef struct TorusCoords {
    float theta, phi;
    float cosTheta, sinTheta;
    float cosPhi, sinPhi;
} TorusCoords;

static TorusCoords torusCoords = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};

static float R = -1.0f;
static float r = -1.0f;

void SetTorusDimensions(float major, float minor) {
    R = major;
    r = minor;
}


Mesh MyGenTorusMesh(int rings, int sides) {
    int vertexCount = rings * sides * 6;
    Vector3 *vertices = (Vector3 *)MemAlloc(vertexCount * sizeof(Vector3));
    Vector3 *normals = (Vector3 *)MemAlloc(vertexCount * sizeof(Vector3));
    Vector2 *texcoords = (Vector2 *)MemAlloc(vertexCount * sizeof(Vector2));

    int index = 0;
    for (int i = 0; i < rings; i++) {
        float theta1 = ((float)i / rings) * 2.0f * PI;
        float theta2 = ((float)(i + 1) / rings) * 2.0f * PI;

        for (int j = 0; j < sides; j++) {
            float phi1 = ((float)j / sides) * 2.0f * PI;
            float phi2 = ((float)(j + 1) / sides) * 2.0f * PI;

            Vector3 p[4];
            Vector3 n[4];
            Vector2 t[4];

            for (int k = 0; k < 4; k++) {
                float theta = (k < 2) ? theta1 : theta2;
                float phi   = (k % 2 == 0) ? phi1 : phi2;

                float cosTheta = cosf(theta), sinTheta = sinf(theta);
                float cosPhi   = cosf(phi),   sinPhi   = sinf(phi);

                float x = (R + r * cosPhi) * cosTheta;
                float y = r * sinPhi;
                float z = (R + r * cosPhi) * sinTheta;

                float nx = cosPhi * cosTheta;
                float ny = sinPhi;
                float nz = cosPhi * sinTheta;

                p[k] = (Vector3){ x, y, z };
                n[k] = (Vector3){ nx, ny, nz };
                t[k] = (Vector2){ (float)i / rings, (float)j / sides };
            }

            // Two triangles per quad
            vertices[index] = p[0]; normals[index] = n[0]; texcoords[index++] = t[0];
            vertices[index] = p[1]; normals[index] = n[1]; texcoords[index++] = t[1];
            vertices[index] = p[2]; normals[index] = n[2]; texcoords[index++] = t[2];

            vertices[index] = p[2]; normals[index] = n[2]; texcoords[index++] = t[2];
            vertices[index] = p[1]; normals[index] = n[1]; texcoords[index++] = t[1];
            vertices[index] = p[3]; normals[index] = n[3]; texcoords[index++] = t[3];
        }
    }

    Mesh mesh = { 0 };
    mesh.vertexCount = vertexCount;
    mesh.triangleCount = vertexCount / 3;
    mesh.vertices = (float *)vertices;
    mesh.normals = (float *)normals;
    mesh.texcoords = (float *)texcoords;

    UploadMesh(&mesh, false);
    return mesh;
}

float get_theta(float u) {
        return 2 * PI * u / SCREEN_WIDTH;
}

float get_phi(float v) {
        return 2 * PI * v / SCREEN_HEIGHT;
}

Vector3 get_torus_normal(float u, float v) {
    float theta = get_theta(u);
    float phi = get_phi(v);

    float nx = cosf(phi) * cosf(theta);
    float ny = sinf(phi);
    float nz = cosf(phi) * sinf(theta);

    return (Vector3){ nx, ny, nz };
}

Vector3 get_phi_tangent(float u, float v) {
    float theta = get_theta(u);
    float phi = get_phi(v);

    float tx = -sinf(phi) * cosf(theta);
    float ty = cosf(phi);
    float tz = -sinf(phi) * sinf(theta);

    return (Vector3){ tx, ty, tz };
}

Vector3 get_theta_tangent(float u, float v) {
    float theta = get_theta(u);

    float tx = -sinf(theta);
    float ty = 0.0f;
    float tz =  cosf(theta);

    return (Vector3){ tx, ty, tz };
}

Vector3 get_torus_position(float u, float v) {
    float theta = get_theta(u);
    float phi = get_phi(v);

    float x = (R + r * cosf(phi)) * cosf(theta);
    float y = r * sinf(phi);
    float z = (R + r * cosf(phi)) * sinf(theta);

    return (Vector3){ x, y, z };
}

void set_torus_coords(float u, float v) {
    torusCoords.theta = get_theta(u);
    torusCoords.phi = get_phi(v);
    torusCoords.cosTheta = cosf(torusCoords.theta);
    torusCoords.sinTheta = sinf(torusCoords.theta);
    torusCoords.cosPhi = cosf(torusCoords.phi);
    torusCoords.sinPhi = sinf(torusCoords.phi);
}

Vector3 get_torus_position_fast() {
    return (Vector3){ (R + r * torusCoords.cosPhi) * torusCoords.cosTheta,
                      r * torusCoords.sinPhi,
                      (R + r * torusCoords.cosPhi) * torusCoords.sinTheta };
}

Vector3 get_torus_normal_fast() {
    return (Vector3){ torusCoords.cosPhi * torusCoords.cosTheta,
                      torusCoords.sinPhi,
                      torusCoords.cosPhi * torusCoords.sinTheta };
}
Vector3 get_theta_tangent_fast() {
    return (Vector3){ -torusCoords.sinTheta, 0.0f, torusCoords.cosTheta };
}
Vector3 get_phi_tangent_fast() {
    return (Vector3){ -torusCoords.sinPhi * torusCoords.cosTheta,
                      torusCoords.cosPhi,
                      -torusCoords.sinPhi * torusCoords.sinTheta };
}

Matrix get_torus_transform(Boid *boid, float scale) {
    set_torus_coords(boid->position.x, boid->position.y);
    Vector3 position = Vector3Add(
        get_torus_position_fast(),
        Vector3Scale(get_torus_normal_fast(), BOID_HEIGHT));

    Vector3 velocity = Vector3Add(
        Vector3Scale(get_theta_tangent_fast(), boid->velocity.x),
        Vector3Scale(get_phi_tangent_fast(), boid->velocity.y));

    Vector3 forward = Vector3Normalize(velocity);
    Vector3 up = get_torus_normal_fast();
    Vector3 right = Vector3CrossProduct(forward, up);

    Matrix transform = {
        forward.x * scale, up.x * scale, right.x * scale, position.x,
        forward.y * scale, up.y * scale, right.y * scale, position.y,
        forward.z * scale, up.z * scale, right.z * scale, position.z,
        0.0f ,      0.0f,   0.0f,        1.0f
    };

    return transform;
}