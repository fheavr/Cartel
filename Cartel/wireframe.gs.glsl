#version 330

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

in  vec3 vLightIntensity[3];
in  vec2 vTexCoord[3];
out vec3 LightIntensity;
out vec2 TexCoord;

in gl_PerVertex {
    vec4 gl_Position;
    float gl_PointSize;
    float gl_ClipDistance[];
} gl_in[];

uniform           vec2 scale;
noperspective out vec3 dist;

void main(void)
{
    // heavily modified from 'http://strattonbrazil.blogspot.ca/2011/09/single-pass-wireframe-rendering_10.html'
    vec2 p0 = scale * gl_in[0].gl_Position.xy/gl_in[0].gl_Position.w;
    vec2 p1 = scale * gl_in[1].gl_Position.xy/gl_in[1].gl_Position.w;
    vec2 p2 = scale * gl_in[2].gl_Position.xy/gl_in[2].gl_Position.w;
    vec2 v0 = p2 - p1;
    vec2 v1 = p2 - p0;
    vec2 v2 = p1 - p0;
    float area = abs(v1.x * v2.y - v1.y * v2.x) / 2;

    dist = vec3(area/length(v0),0,0);
    LightIntensity = vLightIntensity[0];
    TexCoord = vTexCoord[0];
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();

    dist = vec3(0,area/length(v1),0);
    LightIntensity = vLightIntensity[1];
    TexCoord = vTexCoord[1];
    gl_Position = gl_in[1].gl_Position;
    EmitVertex();

    dist = vec3(0,0,area/length(v2));
    LightIntensity = vLightIntensity[2];
    TexCoord = vTexCoord[2];
    gl_Position = gl_in[2].gl_Position;
    EmitVertex();

    EndPrimitive();
}