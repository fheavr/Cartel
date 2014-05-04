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

#include "DrawMesh.h"
#include <fstream>
#include "glm/glm.hpp"
#include "Utils.h"

DrawMesh::~DrawMesh()
{
    if (m_vbos != NULL)
        delete [] m_vbos;
}

void DrawMesh::init(int num_buffers, RenderState &state)
{
    state.bindVAO();

    m_vbos = new VBuffer[num_buffers];
    if (!m_vbos)
        fprintf(stderr, "Could not allocate buffer\n");

    // set the buffer IDs that the buffer objects will use
    m_ibo.setRender(state.getBufferID(0), state.getBufferSize(0));
    for (int i = 0; i < num_buffers; i++)
        m_vbos[i].setRender(state.getBufferID(i+1), state.getBufferSize(i+1));

    m_num_vbos = num_buffers;
    m_state = &state;
}

void DrawMesh::drawMesh()
{
    m_state->bindVAO();
    syncGPU(0, 0);
    glDrawElements(GL_TRIANGLES, m_num_elem, GL_UNSIGNED_INT, 0);
}

void DrawMesh::loadVBuffer(int buffer_num, int data_size, GLubyte *data, int data_offset, int num_attr, attrib_info *attr_info)
{
    if (buffer_num < 0 && buffer_num > m_num_vbos)
        return;

    m_vbos[buffer_num].loadBuffer(data_size, data, data_offset, num_attr, attr_info);
}

void DrawMesh::loadIBuffer(int num_elem, int elem_size, int *data)
{
    // no need to bind vertex array buffer, since this is an index buffer
    m_ibo.loadBuffer(num_elem, elem_size, data);

    m_num_elem = num_elem;
}

void DrawMesh::analyzeAttr(int attrib_num, float *&attrib_ptr, int &stride)
{
    for (int i = 0; i < m_num_vbos; i++)
    {
        m_vbos[i].analyzeAttr(attrib_num, attrib_ptr, stride);
        if (attrib_ptr != NULL)
            break;
    }
}

void DrawMesh::analyzeMesh(int &num_elements, int **indices)
{
    num_elements = m_num_elem;

    if (indices)
        *indices = (int*)m_ibo.m_local_data;
}

// by having a range, we can choose to only update the buffers we have changed
// extent is the number of buffers after the base index to update
void DrawMesh::syncGPU(int base, int extent)
{
    if (base + extent > m_num_vbos ||
        extent == 0)
        extent = m_num_vbos - base;

    // bind the vertex array object to store all the vertex settings
    m_state->bindVAO();
    for (int i=base; i < extent; i++)
    {
        m_vbos[i].SyncBuffer();
    }

    m_ibo.SyncBuffer();
}