#version 330

in vec3 fragNormal;     // world-space normal
in vec3 fragPosition;   // world-space position

uniform vec3 lightDir;     // world-space light direction (e.g. sun)
uniform vec4 lightColor;   // light colour (e.g. white = vec4(1.0))

out vec4 finalColor;

void main() {
    vec3 norm = normalize(fragNormal);
    vec3 light = normalize(-lightDir);  // direction *toward* the surface

    // Diffuse lighting
    float diffuse = max(dot(norm, light), 0.0);
    diffuse = pow(diffuse, 0.6);  // optional gamma boost

    // Ambient term to prevent full black
    vec3 ambient = vec3(0.1);

    // Optional: specular lighting (simple Phong)
    vec3 viewDir = normalize(-fragPosition); // assumes camera near origin
    vec3 reflectDir = reflect(-light, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = vec3(0.5) * spec;  // grey specular

    // Base object colour (can be uniform or texture later)
    vec3 baseColor = vec3(1.0);  // white dart

    vec3 lighting = (ambient + diffuse + specular) * baseColor * lightColor.rgb;

    finalColor = vec4(lighting, 1.0);
}
