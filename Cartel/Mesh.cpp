/* Copyright (c) Darcy Harisson, Russell Gillette
 * April 2014
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
#include "Mesh.h"

Mesh::Mesh(EditMesh &em) : last_drawn(-1), m_em(&em)
{}

Mesh::~Mesh()
{}

void Mesh::init(RenderState &state)
{
    m_dm.init(1, state);
}

void Mesh::drawMesh()
{
    if (last_drawn != m_em->get_edit_count())
        updateDrawMesh();

    m_dm.drawMesh();
}

void Mesh::updateDrawMesh()
{
    int i_size = 3*m_em->get_face_size();
    int v_size = 3*i_size;//m_em->get_vert_size();

    float *v_data = new float[2*v_size];
    int   *i_data = new int[i_size];

    m_em->get_draw_data(v_data, i_data);
    m_em->get_draw_normals(&v_data[v_size]);

    // i don't want to allocate on the heap, nor do I want to
    // hard code all the values so this is my disgusting solution
    #define DRAW_MESH_NUM_ATTR 2

    attrib_info attr_info[DRAW_MESH_NUM_ATTR];
    attr_info[0].attrib_number = 0; // vertices are 0
    attr_info[0].attrib_size   = sizeof(float);
    attr_info[0].data_offset   = 0; // data starts at beginning of array
    attr_info[0].data_stride   = 0; // data is tightly packed
    attr_info[0].num_comp      = 3; // there are 3 components per vertex position

    attr_info[1].attrib_number = 1; // normals are 1
    attr_info[1].attrib_size   = sizeof(float);
    attr_info[1].data_offset   = v_size * sizeof(float); // data starts after vertices
    attr_info[1].data_stride   = 0; // data is tightly packed
    attr_info[1].num_comp      = 3; // there are 3 components per vertex normal

    m_dm.loadVBuffer(0, sizeof(float)*2*v_size, (GLubyte*)v_data, 0, DRAW_MESH_NUM_ATTR, attr_info);
    m_dm.loadIBuffer(i_size, sizeof(int), i_data);

    #undef DRAW_MESH_NUM_ATTR
    delete [] v_data;
    delete [] i_data;

    last_drawn = m_em->get_edit_count();
}