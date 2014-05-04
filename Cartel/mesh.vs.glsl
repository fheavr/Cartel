#version 330

layout (location = 0) in vec3 Position;
layout (location = 1) in vec3 Normal;
layout (location = 2) in vec2 Texcoords;

out vec3 vLightIntensity;
out vec2 vTexCoord;

struct LightInfo
{
    vec4 Position; // we are going to treat this as a direction to acheive directional lighting
    vec3 La;       // ambient light
    vec3 Ld;       // diffuse light
    vec3 Ls;       // specular light
};
uniform LightInfo Light0;

struct MaterialInfo
{
    vec3 Ka;
    vec3 Kd;
    vec3 Ks;
    float Shininess;
};
uniform MaterialInfo Material;

uniform mat4 ModelViewMatrix;
uniform mat3 NormalMatrix;      // we keep a MV matrix without the translation component to apply to vectors
uniform mat4 ProjectionMatrix;
uniform mat4 MVP;               // ModelViewProjection Matrix

void main()
{
    // determine vertex color
    vec3 tnorm     = normalize( NormalMatrix * Normal );
    vec3 s         = normalize( vec3(Light0.Position) ); // incident vector
    vLightIntensity = dot(s, tnorm) * Material.Ka * 0.2 + Material.Ka * 0.8;

    vTexCoord = Texcoords;
    gl_Position = MVP * vec4(Position, 1.0);
}