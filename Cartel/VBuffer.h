/* Copyright (c) Russell Gillette
 * December 2013
 *
 * CPSC Computer Graphics 2014
 * University of British Columbia
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

#ifndef VBUFFER_H
#define VBUFFER_H

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

#define MAX_NUM_ATTRIB 4

struct attrib_info
{
    unsigned int attrib_number;  // which attribute within the mesh this info is for
    unsigned int attrib_size;    // total size in bytes of one attribute
    unsigned int num_comp;       // the number of components each of these atributes holds
    unsigned int data_offset;    // the number of bytes from the start of the array to the first instance of this attribute
    unsigned int data_stride;    // the number of bytes between instances of this attribute
};

/**
 * CPU side storage of the data to be stored in one VBO on the GPU. This object can handle
 * many different vertex attribute layouts.
 */
class VBuffer
{
public:
    // we dont want the vbuffer owning the gl buffer id, beause then multiple
    // animations would ech require a different id (or deleting one would
    // inherently delete the gl id its working with)

    VBuffer():m_renderID(0), m_renderSize(NULL), m_local_data(NULL),
              m_attr_info(NULL), m_num_attr(0), m_size(0)
    {}
    VBuffer(GLuint rID, GLuint *rSize):m_renderID(rID), m_renderSize(rSize), m_local_data(NULL),
                                       m_attr_info(NULL), m_num_attr(0), m_size(0)
    {}
    ~VBuffer()
    {
        if (m_local_data)
            delete [] m_local_data;
        if (m_attr_info)
            delete [] m_attr_info;
    }

    void setRender(GLuint buffID, GLuint *size_ptr)
    {
        m_renderID = buffID;
        m_renderSize = size_ptr;
    }

    void resizeBuffer(int size);
    // IMPORTANT: you must load the correct VAO prior to calling this function. Since the
    // buffer object has no notion of VAO it is just setting its parameters on whatever
    // VAO is currently bound
    void loadBuffer(int data_size, GLubyte *data, int data_offset, int num_attr, attrib_info *attr_info);
    void SyncBuffer();
    void SyncBuffer(GLuint size);
    void SyncBuffer(GLuint buffer, GLuint *size);

    // WARNING: this may be very very slow depending on how vertex attributes are packed
    void analyzeAttr(int attrib_num, float *&attrib_ptr, int &stride);

    int          m_size;           // the size in BYTEs of our local data store
    int          m_num_attr;       // the number of attributes per vertex
    attrib_info *m_attr_info;

    float       *m_local_data;     // our local copy of the mesh data that we can modify and copy to the gpu

    // default render values, if m_renderSize is NULL then the memory on the GPU
    // will be reallocated on any Sync call that does not specify a size
    GLuint  m_renderID;
    GLuint *m_renderSize; // size of the buffer on the GPU (points to render state)
};

#endif // VBUFFER_H