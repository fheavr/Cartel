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

#pragma once
#ifndef RENDER_STATE_H
#define RENDER_STATE_H

#ifdef _WIN32
#  include "GL/glew.h"
#  include "GLFW/glfw3.h"
# elif __APPLE__
#  include <GL/glew.h>
#  include <GLFW/glfw3.h>
#else
#  include <GL/glew.h>
#  include <GLFW/glfw3.h>
#endif

/**
 * A RenderState object stores a collection of GPU vertex buffer objects to be used to render a
 * single mesh at a time. It also stores all of the associated state setup for
 * those buffers, such as attribute stride and offset within a vertex
 * array object. 
 */
class RenderState
{
public:
    RenderState(unsigned int num_bufs)
    {
        glGenVertexArrays(1, &m_vaoID);

        m_vboIDs = new GLuint[num_bufs];
        glGenBuffers(num_bufs, m_vboIDs);

        m_size = new GLuint[num_bufs];
        for (int i = 0; i < num_bufs; i++)
            m_size[i] = 0;

        m_num_buffers = num_bufs;
    }
    ~RenderState()
    {
        glDeleteVertexArrays(1, &m_vaoID);
        glDeleteBuffers(m_num_buffers, m_vboIDs);
        delete [] m_vboIDs;
        delete [] m_size;
    }
    void bindVAO()
    {
        glBindVertexArray(m_vaoID);
    }
    GLuint getBufferID(unsigned int i)
    {
        return m_vboIDs[i];
    }
    GLuint *getBufferSize(unsigned int i)
    {
        return &m_size[i];
    }
private:
    GLuint  m_vaoID;
    GLuint  m_num_buffers;
    GLuint *m_vboIDs;
    GLuint *m_size; // used to avoid needless resizing of buffers
};

#endif // RENDER_STATE_H