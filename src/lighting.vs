#version 330

in vec3 vertexPosition;
in vec3 vertexNormal;

uniform mat4 mvp;      // model * view * projection
uniform mat4 matModel; // model matrix
uniform mat4 matNormal; // normal matrix (transpose(inverse(matModel)))

out vec3 fragNormal;
out vec3 fragPosition;

void main() {
    vec4 worldPos = matModel * vec4(vertexPosition, 1.0);
    fragPosition = worldPos.xyz;
    fragNormal = mat3(matNormal) * vertexNormal;  // transform normal to world space

    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
