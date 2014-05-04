#version 330

layout (location = 0) out vec4 FragColor;

in vec3 LightIntensity;
noperspective in vec3 dist; // distances from edges

// 0x1 -> faces
// 0x2 -> edges
// 0x4 -> verts
uniform int view_mode = 3; // default view mesh and edges

struct LightInfo
{
    vec4 LPosition; // we are going to treat this as a direction to achieve directional lighting
    vec3 La;       // ambient light
    vec3 Ld;       // diffuse light
    vec3 Ls;       // specular light
};
uniform LightInfo Light;

struct MaterialInfo
{
    vec3 Ka;            // Ambient reflectivity
    vec3 Kd;            // Diffuse reflectivity
    vec3 Ks;            // Specular reflectivity
    float Shininess;    // Specular shininess factor
};
uniform MaterialInfo Material;

void main()
{
    float threshold = 1;

    float tmp;
    vec3  dst = dist;
    if (dst[2] < dst[1]) {
        dst[1] = dist[2];
        dst[2] = dist[1];
    }
    if (dst[1] < dst[0]) {
        tmp = dst[0];
        dst[0] = dst[1];
        dst[1] = tmp;
    }

    // calculate edges
    float edgeIntensity = smoothstep(0, 1, dst[0]);

    // calculate verts
    float vert_dist = dst[0]*dst[0] + dst[1]*dst[1];
    float vertIntensity = (vert_dist < threshold)
                                  ? 0 : smoothstep(0, 1, 3*vert_dist);
                                  //: 0;

    vec4 out_color = vec4(0, 0, 0, 0);

    if ((view_mode & 0x1) != 0) { // faces
        out_color = vec4(LightIntensity, 1.0);
    }
    if ((view_mode & 0x2) != 0) { // edges
        out_color = mix(vec4(1, 1, 1, 1.0), out_color, edgeIntensity);
    }
    if ((view_mode & 0x4) != 0) { // verts
        out_color = mix(vec4(1, 0, 0, 1.0), out_color, vertIntensity);
    }

    FragColor = out_color;
}