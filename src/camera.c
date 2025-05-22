
#include "camera.h"

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

