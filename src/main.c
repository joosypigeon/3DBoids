#include <stdbool.h>

#include "raylib.h"

#include "raymath.h"

#include "rlgl.h"
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

#if defined(PLATFORM_DESKTOP)
    #define GLSL_VERSION            330
#else   // PLATFORM_ANDROID, PLATFORM_WEB
    #define GLSL_VERSION            100
#endif

#include <stdlib.h>
#include <omp.h>
#include "boids.h"
#include "spatial_hash.h"
#define RAYGUI_IMPLEMENTATION

#include "raygui.h"

int SCREEN_WIDTH;
int SCREEN_HEIGHT;
float HALF_SCREEN_WIDTH;
float HALF_SCREEN_HEIGHT;
bool drawFullGlyph = false;
bool drawDensity = false;
bool mousePressed = false;
bool nearestNeighboursNetwork = false;
bool pauseSimulation = false;
Boid *debugBoid = NULL;

void UpdateCameraManual(Camera3D *camera)
{
    static float cameraYaw = 0.0f;
    static float cameraPitch = PI / 2.0f;  // Looking straight down at the origin
    static float cameraDistance = 1000.0f;
    static Vector3 target = { 0.0f, 0.0f, 0.0f };

    float wheel = GetMouseWheelMove();
    cameraDistance -= wheel * 100.0f;
    if (cameraDistance < 10.0f) cameraDistance = 10.0f;

    if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
        Vector2 delta = GetMouseDelta();
        cameraYaw -= delta.x * 0.01f;
        cameraPitch += delta.y * 0.01f;

        // Clamp pitch to avoid flipping
        if (cameraPitch > PI/2.0f - 0.01f) cameraPitch = PI/2.0f - 0.01f;
        if (cameraPitch < -PI/2.0f + 0.01f) cameraPitch = -PI/2.0f + 0.01f;
    }

    if (IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
        Vector2 delta = GetMouseDelta();
        float panSpeed = cameraDistance * 0.001f;

        Vector3 right = (Vector3){
            cosf(cameraYaw), 0.0f, -sinf(cameraYaw)
        };
        Vector3 up = (Vector3){ 0.0f, 1.0f, 0.0f };

        target = Vector3Add(target, Vector3Scale(right, -delta.x * panSpeed));
        target = Vector3Add(target, Vector3Scale(up, delta.y * panSpeed));
    }

    Vector3 offset = {
        cameraDistance * cosf(cameraPitch) * sinf(cameraYaw),
        cameraDistance * sinf(cameraPitch),
        cameraDistance * cosf(cameraPitch) * cosf(cameraYaw)
    };

    camera->position = Vector3Add(target, offset);
    camera->target = target;
    camera->up = (Vector3){ 0.0f, 1.0f, 0.0f };
}



Model dart;
Model transparentSphere;  // <-- global scope, outside of main()

int main(void)
{
    printf("Linked Raylib version: %s\n", RAYLIB_VERSION);
    const int glslVer = rlGetVersion();
    printf("GL version: %i\n", glslVer);

    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitWindow(0, 0, "Fullscreen at Desktop Resolution");

    // Get the primary monitor's resolution before window creation
    int monitor = GetCurrentMonitor();
    SCREEN_HEIGHT = GetMonitorHeight(monitor);
    SCREEN_WIDTH = GetMonitorWidth(monitor);
    printf("Monitor %d: %d x %d\n", monitor, SCREEN_WIDTH, SCREEN_HEIGHT);
    SCREEN_WIDTH = (SCREEN_WIDTH/CELL_SIZE)*CELL_SIZE;
    SCREEN_HEIGHT = (SCREEN_HEIGHT/CELL_SIZE)*CELL_SIZE;
    printf("Monitor %d: %d x %d\n", monitor, SCREEN_WIDTH, SCREEN_HEIGHT);
    HALF_SCREEN_WIDTH = SCREEN_WIDTH / 2.0f;
    HALF_SCREEN_HEIGHT = SCREEN_HEIGHT / 2.0f;


    //Vector3 corner01 = { 0.0f, 0.0f, -300.0f };
    //Vector3 corner02 = { SCREEN_WIDTH, 0.0f, -300.0f };
    //Vector3 corner03 = { SCREEN_WIDTH, SCREEN_HEIGHT, -300.0f };
    //Vector3 corner04 = { 0.0f, SCREEN_HEIGHT, -300.0f };

    SetTargetFPS(60);

    InitBoids();

    static float alignmentWeight = 1.0f;
    static float cohesionWeight = 1.0f;
    static float separationWeight = 1.0f;

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 0.0f, 0.0f, 0.0f};  // Positioned out along +Z axis
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };       // Looking at the quad at origin
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };           // Standard up direction
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;




    // Load basic lighting shader
    Shader shader = LoadShader(TextFormat("/home/jerry/raylib/examples/shaders/resources/shaders/glsl330/lighting.vs", GLSL_VERSION),
                               TextFormat("/home/jerry/raylib/examples/shaders//resources/shaders/glsl330/lighting.fs", GLSL_VERSION));
    // Get some required shader locations
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    // NOTE: "matModel" location name is automatically assigned on shader loading, 
    // no need to get the location again if using that uniform name
    //shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    
    // Ambient light level (some basic lighting)
    int ambientLoc = GetShaderLocation(shader, "ambient");
    SetShaderValue(shader, ambientLoc, (float[4]){ 0.1f, 0.1f, 0.1f, 1.0f }, SHADER_UNIFORM_VEC4);

    // Create lights
    Light lights[MAX_LIGHTS] = { 0 };
    lights[0] = CreateLight(LIGHT_POINT, (Vector3){ -HALF_SCREEN_WIDTH, 200, -HALF_SCREEN_HEIGHT }, Vector3Zero(), YELLOW, shader);
    lights[1] = CreateLight(LIGHT_POINT, (Vector3){ HALF_SCREEN_WIDTH, 200, HALF_SCREEN_HEIGHT }, Vector3Zero(), RED, shader);
    lights[2] = CreateLight(LIGHT_POINT, (Vector3){ -HALF_SCREEN_WIDTH, 200, HALF_SCREEN_HEIGHT }, Vector3Zero(), GREEN, shader);
    lights[3] = CreateLight(LIGHT_POINT, (Vector3){ HALF_SCREEN_WIDTH, 200, -HALF_SCREEN_HEIGHT }, Vector3Zero(), BLUE, shader);

    // Load dart model exported from Blender or dart_export
    dart = LoadModel("shaders/3DBoids/blender_dart.obj");
    if (dart.meshCount == 0 || dart.meshes == NULL) {
        printf("Failed to load model\n");
        exit(0);   
     }
     dart.materials[0].shader = shader;  // <== Required for lighting to take effect

    if (dart.meshes[0].normals == NULL) {
        exit(0);
        GenMeshTangents(&dart.meshes[0]);
    }

        // Create a transparent blue sphere mesh + model
    Mesh sphereMesh = GenMeshSphere(1.0f, 16, 16);
    transparentSphere = LoadModelFromMesh(sphereMesh);
    transparentSphere.materials[0].shader = LoadShader(0, 0);  // use built-in default
    transparentSphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = (Color){0, 0, 255, 100};  // Alpha < 25




    //int number_of_frame = 0;
    while (!WindowShouldClose())
    {


        // Update camera
        //UpdateCameraManual(&camera);
        //UpdateCamera(&camera, CAMERA_THIRD_PERSON);
        UpdateCameraManual(&camera);

        // Update the shader with the camera view vector (points towards { 0.0f, 0.0f, 0.0f })
        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        // Check key inputs to enable/disable lights
        if (IsKeyPressed(KEY_Y)) { lights[0].enabled = !lights[0].enabled; }
        if (IsKeyPressed(KEY_R)) { lights[1].enabled = !lights[1].enabled; }
        if (IsKeyPressed(KEY_G)) { lights[2].enabled = !lights[2].enabled; }
        if (IsKeyPressed(KEY_B)) { lights[3].enabled = !lights[3].enabled; }
        
        // Update light values (actually, only enable/disable them)
        for (int i = 0; i < MAX_LIGHTS; i++) UpdateLightValues(shader, lights[i]);


        if (IsKeyPressed(KEY_SPACE)) pauseSimulation = !pauseSimulation;
        if (!pauseSimulation) UpdateBoids(alignmentWeight, cohesionWeight, separationWeight);

        if(IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)){
            if(debugBoid) debugBoid = NULL;
            else debugBoid = FindNearestBoid(GetMousePosition());
        }

        BeginDrawing();


            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                BeginShaderMode(shader);
                DrawPlane(Vector3Zero(), (Vector2) { SCREEN_WIDTH, SCREEN_HEIGHT }, WHITE);
                    DrawBoids3D();
                EndShaderMode();

                // Draw spheres to show where the lights are
                for (int i = 0; i < MAX_LIGHTS; i++)
                {
                    if (lights[i].enabled) DrawSphereEx(lights[i].position, 10.0f, 8, 8, lights[i].color);
                    else DrawSphereWires(lights[i].position, 10.0f, 8, 8, ColorAlpha(lights[i].color, 0.3f));
                }

            EndMode3D();


            DrawText("Boids with Predator Simulation", 20, 10, 20, DARKGRAY);
            DrawText("Current Resolution:", 20, 30, 20, DARKGRAY);
            DrawText(TextFormat("%d x %d", SCREEN_WIDTH, SCREEN_HEIGHT), 20, 50, 30, BLUE);
            DrawText(TextFormat("Boids drawn: %d", number_drawn), 20, 80, 30, BLUE);
            DrawText(TextFormat("Frame Time: %0.2f ms", GetFrameTime() * 1000), 20, 110, 30, BLUE);
            DrawText(TextFormat("OpenMP threads: %d", omp_get_max_threads()), 20, 140, 30, BLUE);

            int oldTextSize = GuiGetStyle(DEFAULT, TEXT_SIZE);
            GuiSetStyle(DEFAULT, TEXT_SIZE, 24);
            GuiCheckBox((Rectangle){ 500, 10, 28, 28 }, "Draw Full Boid Glyph", &drawFullGlyph);
            GuiCheckBox((Rectangle){ 500, 40, 28, 28 }, "Show density", &drawDensity);
            GuiCheckBox((Rectangle){ 500, 70, 28, 28 }, "Show nearest neighbours", &nearestNeighboursNetwork);

            GuiSetStyle(DEFAULT, TEXT_SIZE, oldTextSize);  // Restore to avoid breaking other widgets
            
            DrawFPS(SCREEN_WIDTH - 100, 10);

            // Start the sliders below the text stats
            Rectangle sliderBounds = { 500, 140, 300, 30 };
            float sliderSpacing = 50;

            // Optional: Draw a heading in larger font
            DrawText("Boid Behaviour Weights", sliderBounds.x, sliderBounds.y - 40, 28, DARKGRAY);

            // Make font size larger manually
            //int labelFontSize = 22;
            //int valueFontSize = 22;

            GuiSlider(sliderBounds,
                TextFormat("Alignment (%.2f)", alignmentWeight),
                NULL,
                &alignmentWeight, 0.0f, 10.0f);
            sliderBounds.y += sliderSpacing;

            GuiSlider(sliderBounds,
                TextFormat("Cohesion (%.2f)", cohesionWeight),
                NULL,
                &cohesionWeight, 0.0f, 10.0f);
            sliderBounds.y += sliderSpacing;

            GuiSlider(sliderBounds,
                TextFormat("Separation (%.2f)", separationWeight),
                NULL,
                &separationWeight, 0.0f, 10.0f);

        

        EndDrawing();
    }

    CloseWindow();

    return 0;
}