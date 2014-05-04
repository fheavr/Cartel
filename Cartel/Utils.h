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


/* == Utils.h ==
 *
 * General use common/helper functions
 */

#ifndef UTILS_H
#define UTILS_H

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

#include <stdio.h>
#include <string>
#include <glm/glm.hpp>

/* Takes the location and file name of the file to be processed 
 * and returns the contents stored within a GLchar array
 */
inline const GLchar* loadFileAsString(char* path, GLint *size)
{
    GLchar* file_contents = NULL;
    int file_size = 0;

    FILE* f = fopen(path, "rb");

    fseek(f, 0, SEEK_END);
    file_size = ftell(f);

    file_contents = new GLchar[ file_size + 1 ];
    fseek(f, 0, SEEK_SET);

    fread(file_contents, file_size, 1, f);

    fclose(f);
    file_contents[file_size] = 0; // null terminate the file

    if (size != NULL)
        *size = file_size + 1;

    return file_contents;
}

inline void realign_triangle(glm::vec3 &vert1, glm::vec3 &vert2, glm::vec3 &vert3)
{
    glm::vec3 vec[2];    // triangle bounds used for stretch calculation
    double    edge[2];   // edge lengths of the two triangles, used to determine the 2d representation
    double    angle;     // angle between to edge lengths to complete the triangle representation

    // calculate edge vectors of triangles
    vec[0] = vert1 - vert2;
    vec[1] = vert1 - vert3;

    // calculate the edge lengths
    edge[0] = glm::length(vec[0]);
    edge[1] = glm::length(vec[1]);

    // calculate the angles between edge vectors
    angle = glm::dot(vec[0], vec[1]) / (edge[0] * edge[1]); // cos(theta) to avoid costly calculations

    // realign the triangles to be horizontal
    vert1 = glm::vec3(0, 0, 0);
    vert2 = glm::vec3(0, edge[0], 0);
    vert3 = glm::vec3(edge[1]*(glm::sqrt(1 - angle*angle)), edge[1]*angle, 0); // (e_2*cos(90 - theta), e_2*sin(90 - theta))
}

#endif // UTILS_H