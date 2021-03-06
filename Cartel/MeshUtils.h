/* Copyright (c) Russell Gillette
 * December 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* == MeshUtils.h ==
 * 
 * Utility and helper functions that operate on meshes or mesh data without a
 * specified Mesh instance.
 */

#ifndef MESH_UTILS_H
#define MESH_UTILS_H

#include "Mesh.h"
#include "DrawMesh.h"
#include "EditMesh.h"
#include "OBJLoader.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>

// load an Edit Mesh object from an OBJ file.
EditMesh *loadEditMeshFromFile(string file_name)
{
    EditMesh *m = NULL;

    // parse mesh
    ObjLoader obj_parser(file_name);
    //obj_parser.generateNormals(); // don't use normals atm so no point in doing this

    // gather data from the parser to construct mesh
    GLubyte *data = NULL;
    int *indices;
    int data_size, num_indices, num_attr;
    obj_attrib_info *parsed_attr_info;

    // export parser data
    // NOTE: data is owned by the obj_parser
    obj_parser.objExportGLSeparate(data_size, data, num_indices, indices, num_attr, parsed_attr_info);

    if (parsed_attr_info == NULL)
    {
        printf("Mesh Not Found: Failed to load\n");
        return NULL;
    }

    std::vector<double> xyzPositions;
    std::vector<std::size_t> triangleIndices;

    // populate indices
    for (int i = 0; i < num_indices; i++)
        triangleIndices.push_back(indices[i]);
    
    // populate vertices from byte array (so lots of pointer pushing)
    int attr_end = data_size;
    int v_stride = parsed_attr_info[0].data_stride;
    int v_offset = parsed_attr_info[0].data_offset;

    if (v_stride == 0)
    for (int i = 0; i < num_attr; i++)
    {
        int off = parsed_attr_info[i].data_offset;
        if (off < attr_end && off > v_offset)
            attr_end = off;
    }

    int      attrib_size = parsed_attr_info[0].attrib_size;
    int      elem_size = attrib_size / parsed_attr_info[0].num_comp;

    GLubyte *pAttrib = data;
    GLubyte *pEnd    = data + attr_end;

    // TODO: safety check on number of elements per attribute
    // (there should never be less than 3 but who knows)
    // if (sizeof(float) != elem_size)
    //    unhandled as of right now

    // not particularily safe...
    for (; pAttrib < pEnd; pAttrib += elem_size)
    {
        double tmp = (double)*(float*)pAttrib;
        xyzPositions.push_back(tmp);
    }

    m = new EditMesh();
    m->init(xyzPositions, triangleIndices);

    return m;
}

// load a Mesh object from an OBJ file.
// NOTE: this mesh is packaged for drawing, and thus difficult to modify
//    Mesh for editable mesh
DrawMesh *loadDrawMeshFromFile(RenderState &state, string file_name)
{
    DrawMesh *m = NULL;

    // parse mesh
    ObjLoader obj_parser(file_name);
    obj_parser.generateNormals();

    // gather data from the parser to construct mesh
    GLubyte *data = NULL;
    int *indices;
    int data_size, num_indices, num_attr;
    obj_attrib_info *parsed_attr_info;
    attrib_info     *attr_info;

    // export parser data
    // NOTE: data is owned by the obj_parser
    obj_parser.objExportGLSeparate(data_size, data, num_indices, indices, num_attr, parsed_attr_info);

    attr_info = new attrib_info[num_attr];
    for (int j = 0; j < num_attr; j++)
    {
        attr_info[j].attrib_number = parsed_attr_info[j].attrib_number;
        attr_info[j].attrib_size   = parsed_attr_info[j].attrib_size;
        attr_info[j].data_offset   = parsed_attr_info[j].data_offset;
        attr_info[j].data_stride   = parsed_attr_info[j].data_stride;
        attr_info[j].num_comp      = parsed_attr_info[j].num_comp;
    }

    m = new DrawMesh();
    m->init(1, state);
    m->loadVBuffer(0, data_size, data, 0, num_attr, attr_info);
    m->loadIBuffer(num_indices, sizeof(int), indices);

    delete [] attr_info;

    return m;
}

Mesh *loadMeshFromFile(RenderState &state, string file_name)
{
    EditMesh *em = loadEditMeshFromFile(file_name);
	/*EditMesh *em = new EditMesh;
	em->add_vertex( 0,0,0 );
	em->add_vertex( 1,0,0 );
	em->add_vertex( 0,1,0 );
	em->add_vertex( 0,0,1 );
	em->add_face( 0, 1, 2 );
	em->add_face( 2, 1, 3 );
	em->add_face( 3, 1, 0 );
	em->add_face( 3, 0, 2 );*/
	//std::size_t v = em->split_face_center( 0 );
	//em->set_vertex( v, em->get_vertex( v ) + em->get_vnormal( v ) );
	//std::ofstream fout( "D:/Darcy/Development/CPSC 524 Project/Cartel/debug.obj" );
	//em->write_to_obj_stream( fout );
	//fout.close();
    Mesh *m = new Mesh(*em);
    m->init(state);
    return m;
}

DrawMesh *createAxis(RenderState & state, float scale)
{
    DrawMesh *m;
    attrib_info details[2];

    // NOTE: vertices are duplicated, as theu have different normals
    // and this is not possible to represent in opengl
    float data[] = 
    {
        // vertices
         0.0f,  1.0f,  0.0f,  -0.04f, 0.0f,  0.04f,  0.04f, 0.0f,  0.04f,
         0.0f,  1.0f,  0.0f,  -0.04f, 0.0f, -0.04f, -0.04f, 0.0f,  0.04f,
         0.0f,  1.0f,  0.0f,   0.04f, 0.0f, -0.04f, -0.04f, 0.0f, -0.04f,
         0.0f,  1.0f,  0.0f,   0.04f, 0.0f,  0.04f,  0.04f, 0.0f, -0.04f,
         0.04f, 0.0f,  0.04f,  0.04f, 0.0f, -0.04f, -0.04f, 0.0f, -0.04f,
        -0.04f, 0.0f, -0.04f, -0.04f, 0.0f,  0.04f,  0.04f, 0.0f,  0.04f,
        // start normals: this is a hack to approximate reasonable normals
         0.0f,  0.0f,  1.0f,   0.0f, 0.0f,   1.0f,   0.0f,  0.0f,  1.0f,
        -1.0f,  0.0f,  0.0f,  -1.0f, 0.0f,   0.0f,  -1.0f,  0.0f,  0.0f,
         0.0f,  0.0f, -1.0f,   0.0f, 0.0f,  -1.0f,   0.0f,  0.0f, -1.0f,
         1.0f,  0.0f,  0.0f,   1.0f, 0.0f,   0.0f,   1.0f,  0.0f,  0.0f,
         0.0f, -1.0f,  0.0f,   0.0f, -1.0f,  0.0f,   0.0f, -1.0f,  0.0f,
         0.0f, -1.0f,  0.0f,   0.0f, -1.0f,  0.0f,   0.0f, -1.0f,  0.0f 
    };

    // scale the vertex locations relative to the scale
    // parameter (note I only go to 54, the num verts)
    for (int i = 0; i < 54; i++)
    {
        data[i] *= scale;
    }

    // since we have redefined our vertices, the indices
    // are just a linear ordering
    int indices[18] = {0,1,2,3,4,5,6,7,8,9,10,11,12,
        13,14,15,16,17};

    // attribute information for vertices
    details[0].attrib_number = 0;
    details[0].attrib_size   = 3 * sizeof(float); // size of one vertex
    details[0].data_offset   = 0; // byte offset to vertices
    details[0].data_stride   = 0; // space between vertices
    details[0].num_comp      = 3;

    // attribute information for normals
    details[1].attrib_number = 1;
    details[1].attrib_size   = 3 * sizeof(float); // size of one normal
    details[1].data_offset   = 18 * 3 * sizeof(float); // byte offset to normals
    details[1].data_stride   = 0; // space between normals
    details[1].num_comp      = 3;

    m = new DrawMesh();
    m->init(1, state);
    m->loadVBuffer(0, sizeof(data), (GLubyte*)data, 0, 2, details);
    m->loadIBuffer(18, sizeof(int), indices);

    return m;
}

DrawMesh *createGem(RenderState & state, float scale)
{
    DrawMesh *m;
    attrib_info details[2];

    // NOTE: vertices are duplicated, as theu have different normals
    // and this is not possible to represent in opengl
    float data[] = 
    {
         // vertices
         0.5f, -0.5f,  0.0f, //v1
         0.5f,  0.5f,  0.0f, //v2
         0.0f,  0.0f,  1.0f, //v5

         0.5f,  0.5f,  0.0f, //v2
        -0.5f,  0.5f,  0.0f, //v3
         0.0f,  0.0f,  1.0f, //v5

        -0.5f,  0.5f,  0.0f, //v3
        -0.5f, -0.5f,  0.0f, //v4
         0.0f,  0.0f,  1.0f, //v5

        -0.5f, -0.5f,  0.0f, //v4
         0.5f, -0.5f,  0.0f, //v1
         0.0f,  0.0f,  1.0f, //v5

         0.5f,  0.5f,  0.0f, //v2
         0.5f, -0.5f,  0.0f, //v1
         0.0f,  0.0f, -1.0f, //v6

        -0.5f,  0.5f,  0.0f, //v3
         0.5f,  0.5f,  0.0f, //v2
         0.0f,  0.0f, -1.0f, //v6

        -0.5f, -0.5f,  0.0f, //v4
        -0.5f,  0.5f,  0.0f, //v3
         0.0f,  0.0f, -1.0f, //v6

         0.5f, -0.5f,  0.0f, //v1
        -0.5f, -0.5f,  0.0f, //v4
         0.0f,  0.0f, -1.0f, //v6

         // normals
         0.894427191f,  0.0f,  0.447213595f, //n1
         0.894427191f,  0.0f,  0.447213595f, //n1
         0.894427191f,  0.0f,  0.447213595f, //n1

         0.0f,  0.894427191f,  0.447213595f, //n2
         0.0f,  0.894427191f,  0.447213595f, //n2
         0.0f,  0.894427191f,  0.447213595f, //n2

        -0.894427191f,  0.0f,  0.447213595f, //n3
        -0.894427191f,  0.0f,  0.447213595f, //n3
        -0.894427191f,  0.0f,  0.447213595f, //n3

         0.0f, -0.894427191f,  0.447213595f, //n4
         0.0f, -0.894427191f,  0.447213595f, //n4
         0.0f, -0.894427191f,  0.447213595f, //n4

         0.894427191f,  0.0f, -0.447213595f, //n5
         0.894427191f,  0.0f, -0.447213595f, //n5
         0.894427191f,  0.0f, -0.447213595f, //n5

         0.0f,  0.894427191f, -0.447213595f, //n6
         0.0f,  0.894427191f, -0.447213595f, //n6
         0.0f,  0.894427191f, -0.447213595f, //n6

        -0.894427191f,  0.0f, -0.447213595f, //n7
        -0.894427191f,  0.0f, -0.447213595f, //n7
        -0.894427191f,  0.0f, -0.447213595f, //n7

         0.0f, -0.894427191f, -0.447213595f, //n8
         0.0f, -0.894427191f, -0.447213595f, //n8
         0.0f, -0.894427191f, -0.447213595f, //n8
    };

    // scale the vertex locations relative to the scale
    // parameter (note I only go to 72, the num verts)
    for (int i = 0; i < 72; i++)
    {
        data[i] *= scale;
    }

    // since we have redefined our vertices, the indices
    // are just a linear ordering
    int indices[24] = {0,1,2,3,4,5,6,7,8,9,10,11,12,
        13,14,15,16,17,18,19,20,21,22,23};

    // attribute information for vertices
    details[0].attrib_number = 0;
    details[0].attrib_size   = 3 * sizeof(float); // size of one vertex
    details[0].data_offset   = 0; // byte offset to vertices
    details[0].data_stride   = 0; // space between vertices
    details[0].num_comp      = 3;

    // attribute information for normals
    details[1].attrib_number = 1;
    details[1].attrib_size   = 3 * sizeof(float); // size of one normal
    details[1].data_offset   = 24 * 3 * sizeof(float); // byte offset to normals
    details[1].data_stride   = 0; // space between normals
    details[1].num_comp      = 3;

    m = new DrawMesh();
    m->init(1, state);
    m->loadVBuffer(0, sizeof(data), (GLubyte*)data, 0, 2, details);
    m->loadIBuffer(24, sizeof(int), indices);

    return m;
}

void drawBox(int x1, int y1, int x2, int y2)
{
    // box has zero area
    if (x1 == x2 && y1 == y2)
        return;

    // safety check to avoid triangle flipping
    if (x1 > x2)
    {
        int tmp = x2;
        x2 = x1;
        x1 = tmp;
    }
    if (y1 > y2)
    {
        int tmp = y2;
        y2 = y1;
        y1 = tmp;
    }
    float fx1 = ((float)x1/c_state.width)*2 - 1;
    float fx2 = ((float)x2/c_state.width)*2 - 1;
    float fy1 = ((float)y1/c_state.height)*2 - 1;
    float fy2 = ((float)y2/c_state.height)*2 - 1;

    GLfloat vertices[] = { fx1, fy1, 0.5f,
                           fx2, fy1, 0.5f,
                           fx1, fy2, 0.5f,
                           fx1, fy2, 0.5f,
                           fx2, fy1, 0.5f,
                           fx2, fy2, 0.5f };

    // activate and specify pointer to vertex array
    // this could be much more efficient if I wasn't lazy
    GLuint tmp;
    glGenBuffers(1, &tmp);
    glBindBuffer(GL_ARRAY_BUFFER, tmp);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*18, vertices, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    // draw box
    glDrawArrays(GL_TRIANGLES, 0, 18);

    // deactivate vertex arrays after drawing
    glDisableVertexAttribArray(0);
    glDeleteBuffers(1, &tmp);
}

// returns a color indicating which quadtrant the modelview tranform has
// translated the point (0,0,0) to.
glm::vec3 identifyQuadrant(glm::mat4 modelview)
{
    glm::vec3 out = glm::vec3();
    out.r = (modelview[3].r > 0) ? 1 : 0;
    out.g = (modelview[3].g > 0) ? 1 : 0;
    out.b = (modelview[3].b > 0) ? 1 : 0;
    out *= 0.7;
    out += glm::vec3(0.3);
    return out;
}
#endif // MESH_UTILS_H