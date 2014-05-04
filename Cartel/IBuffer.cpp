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

#include "IBuffer.h"

#include <string.h>
#include <cstdio>

void IBuffer::resizeBuffer(int size)
{
    if (size <= m_size)
        return;

    GLubyte *tmp = new GLubyte[size];
    if (m_local_data)
    {
        memcpy(tmp, m_local_data, m_size);
        delete [] m_local_data;
    }

    m_local_data = tmp;
    m_size = size;

    if (!m_local_data)
    {
        fprintf(stderr, "unable to allocate Ibuffer");
    }
}

void IBuffer::loadBuffer(int num_elem, int elem_size, int *data, int data_offset)
{
    if (data == NULL)
        return;

    m_elem_size = elem_size;
    resizeBuffer(num_elem * elem_size + data_offset); // does nothing if data is already allocated and large enough
    memcpy(m_local_data, data, num_elem * elem_size);     // note: don't use m_size. This method allows partial copies

#ifdef DEBUG
    GLenum err =  glGetError();
    if (err != GL_NO_ERROR)
    {
        fprintf(stderr, "Buffer Assignment Failed: %08x\n", err);
    }
#endif
}

// copy all local data to the GPU
void IBuffer::SyncBuffer(GLuint bufferID, GLuint *size)
{
    if (!m_local_data)
    {
        fprintf(stderr, "No Index bound\n");
        return;
    }
    // bind the GPU buffer that we want to setup for use with our local data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferID);
    
    if (m_size > *size)
    {
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_size, m_local_data, GL_DYNAMIC_DRAW);
        *size = m_size;
    }
    else
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, m_size, m_local_data);
}

// copy all local data to the GPU
void IBuffer::SyncBuffer()
{
    if (m_renderID == 0)
    {
        fprintf(stderr, "Default Render Information not Populated, aborting\n");
        return;
    }
    SyncBuffer(m_renderID, (m_renderSize == NULL) ? 0 : m_renderSize);
}
