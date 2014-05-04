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

#include "VBuffer.h"
#include <stdio.h>
#include <string.h>

void VBuffer::resizeBuffer(int size)
{
    if (size <= m_size)
        return;

    float *tmp = (float*) new GLubyte[size];

    if (m_local_data)
    {
        memcpy(tmp, m_local_data, m_size);
        delete [] m_local_data;
    }

    m_size = size;
    m_local_data = tmp;

    // nothing is actually loaded to the GPU until draw time to
    // avoid redundant updates
}

/* Each vertex has a set of attributes (such as position, normals, texture coords, etc)
 * Each attribute has a set of components (such as the x, y, and z coordinate of the position)
 *
 * They may be laid out in video memory as follows:
 *
 * Interlaced:
 *   ________________Vertex1_______________ _______Vertex2_ ...
 *  | Attribute1 | Attribute2 | Attribute3 | Attribute1 | At...
 *  | x  y  z  w | x  y  z  w | x  y  z  w | x  y  z  w | x ...
 *
 * Separate:
 *   ___Vertex1__ ___Vertex2__ ___Vertex3__ ...___Vertex1__ ___Ver...
 *  | Attribute1 | Attribute1 | Attribute1 |... Attribute2 | Attri...
 *  | x  y  z  w | x  y  z  w | x  y  z  w |... x  y  z  w | x y z...
 *
 * attr_offset is the distance to offset from the beginning of the array to get to the first
 * instance of the given attribute.
 *
 * attr_stride is the space between the start of one vertex and the start of another
 * ie from the start of one attribute and the start to the next instance of that same attribute.
 *
 * This can be even further complicated by the fact that the stride can be 0 on all arrays
 * but yet have multiple attributes by having everything within different arrays and binding
 * a new array before calling the appropriate glVertexAttribPointer call.
 * THIS IS NOT CURRENTLY HANDLED (setting up and managing the order of attrib arrays is complex)
 *
 * buffer_type specifies whether you are binding an index buffer "GL_ELEMENT_ARRAY_BUFFER" or a
 * vertex buffer "GL_ARRAY_BUFFER"
 */
void VBuffer::loadBuffer(int data_size, GLubyte *data, int data_offset, int num_attr, attrib_info *attr_info)
{
    // does nothing if data is already allocated
    resizeBuffer(data_size + data_offset);

    // save parameters for future updates
    if (m_num_attr < num_attr)
    {
        delete [] m_attr_info;
        m_attr_info = NULL;
    }
    m_num_attr = num_attr;

    if (!m_attr_info)
        m_attr_info = new attrib_info[num_attr];

    // update our internal attribute info
    memcpy(m_attr_info, attr_info, num_attr * sizeof(attrib_info));

    // update our internal data
    memcpy(m_local_data + data_offset, data, data_size);

#ifdef DEBUG
    GLenum err =  glGetError();
    if (err != GL_NO_ERROR)
    {
        fprintf(stderr, "Buffer Assignment Failed: %08x\n", err);
    }
#endif
}

void VBuffer::analyzeAttr(int attrib_num, float *&attrib_ptr, int &stride)
{
    int i;
    for (i = 0; i < m_num_attr; i++)
    {
        if (m_attr_info[i].attrib_number == attrib_num)
            break;
    }
    if (i == m_num_attr)
    {
        attrib_ptr = NULL;
        stride = 0;
        return;
    }
    stride = m_attr_info[i].data_stride / sizeof(float); // stride is originally in bytes, not sizeof float
    attrib_ptr = (float*)(((char*)m_local_data) + m_attr_info[i].data_offset);

    if (stride == 0)
        stride = m_attr_info[i].attrib_size / sizeof(float); // assume tightly packed
}

// copy all local data to the GPU
// buffer: buffer to render to
// the buffer's current size, can use 0 to always reallocate
void VBuffer::SyncBuffer(GLuint buffer, GLuint *size)
{
    // bind the correct VBO
    glBindBuffer(GL_ARRAY_BUFFER, buffer);

    // ensure that the GPU has enough memory allocated
    if (m_size > *size)
    {
        glBufferData(GL_ARRAY_BUFFER, m_size, m_local_data, GL_DYNAMIC_DRAW);
        *size = m_size;
    }
    else
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_size, m_local_data);

    for (int i = 0; i < m_num_attr; ++i)
    {
        glEnableVertexAttribArray(m_attr_info[i].attrib_number);
        glVertexAttribPointer(m_attr_info[i].attrib_number, m_attr_info[i].num_comp, GL_FLOAT, GL_FALSE,
                              m_attr_info[i].data_stride, (GLvoid *) m_attr_info[i].data_offset);
    }
}

// copy all local data to the GPU using
// stored buffer ID
void VBuffer::SyncBuffer()
{
    if (m_renderID == 0)
    {
        fprintf(stderr, "Default Render Information not Populated, aborting\n");
        return;
    }
    SyncBuffer(m_renderID, (m_renderSize == NULL) ? 0 : m_renderSize);
}