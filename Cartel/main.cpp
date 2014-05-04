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

#define GLFW_INCLUDE_GLU

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

#include <iostream>

#include <stdio.h>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/ext.hpp>

#include "ShaderUtils.h"
#include "ControlState.h"
#include "WorldState.h"
#include "RenderState.h"
#include "DrawMesh.h"
#include "MeshUtils.h"
#include "TextureUtils.h"
#include "TextureTypes.h"

#include "EditMesh.h"

WorldState *w_state;
RenderState *r_state[2];
// NOTE: defined in ControlState.h 
// ControlState c_state;

int mesh_file_size = 4;
int mesh_curr = 0;
char *mesh_files[] = {"Mesh/horse.obj",
                      "Mesh/camel.obj",
                      "Mesh/cow_head.obj",
                      "Mesh/cow1.obj"};
Mesh *g_mesh;
DrawMesh *g_axis; // NOTE: only a single axis

// the display loop, where all of the code that actually
// changes what you see goes
void display()
{
    /* limit framerate to 60 fps */
    double curr = 0;
    if ((curr = glfwGetTime()) < 0.016666667) // curr < ~ 1/60
        return;

    // start counting over
    glfwSetTime(0.0);

    // Clear the buffer we will draw into.
    glClearColor(0.549, 0.47, 0.937, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Setup camera projection.
    w_state->projection = glm::perspective(50.0f, c_state.aspectRatio(), 0.1f, 40.0f);

    // Setup camera position/orientation.
    w_state->view = glm::lookAt(
        c_state.viewDepth * glm::vec3(0.0f, 2.5f, 10.0f), // eye
                            glm::vec3(0.0f, 0.0f,  0.0f), // centre
                            glm::vec3(0.0f, 1.0f,  0.0f)  // up
    );

    /* this would let you rotate the view with arrow keys
     *  c_state.updateView(
     *      c_state.deltaArrLR() * DEGREES_PER_SECOND * curr,
     *      0,
     *      c_state.deltaArrUD() * DEPTH_PER_SECOND * curr - c_state.mouseScroll + 0.00001 );
     */
    c_state.updateView(0, 0, -c_state.mouseScroll );
    c_state.mouseScroll = 0;

    w_state->view = glm::rotate(w_state->view, c_state.viewPhi, glm::vec3(1, 0, 0));
    w_state->view = glm::rotate(w_state->view, c_state.viewTheta, glm::vec3(0, 1, 0));

    /***********************************
     * Apply pending filter
     ***********************************/
    switch(c_state.op)
    {
        case EDIT_SQRT3_SUBDIV:
            g_mesh->subdivide_sqrt3();
            c_state.op = EDIT_NONE;
            break;
        case EDIT_RELOCATE_VERTS:
            g_mesh->remesh_relocate();
            c_state.op = EDIT_NONE;
            break;
		case EDIT_COLLAPSE_EDIT:
		case EDIT_COLLAPSE_FAST_EDIT:
			{
				std::size_t count = 5;
				if( c_state.op == EDIT_COLLAPSE_FAST_EDIT )
					count = 1000;

				static std::size_t total = 0;
				for( std::size_t i = 0; i < count; ++i, ++total )
					g_mesh->collapse_edge();
				std::clog << "Count: " << total << std::endl;
			}
			c_state.op = EDIT_NONE;
			break;
        case EDIT_AREA_RELOCATION:
            g_mesh->remesh_areabased();
            c_state.op = EDIT_NONE;
            break;
        case EDIT_DELAUNAY:
            g_mesh->remesh_delaunay();
            c_state.op = EDIT_NONE;
            break;
        case EDIT_REMESH:
            g_mesh->remesh();
            c_state.op = EDIT_NONE;
            break;
        case EDIT_PRETTY:
            g_mesh->remesh_pretty();
            c_state.op = EDIT_NONE;
            break;
		case EDIT_DEBUG:
			{
			std::ofstream fout( "debug.obj" );
			g_mesh->get_edit_mesh().write_to_obj_stream( fout );
			fout.close();
			}
        case EDIT_NONE:
        default:
            break;
    }

    if (c_state.reload)
    {
        delete g_mesh;
        g_mesh = loadMeshFromFile(*r_state[0], mesh_files[mesh_curr]);
        mesh_curr = (mesh_curr + 1) % mesh_file_size;
        c_state.reload = false;
    }

    /***********************************
     * XYZ Axis Code
     ***********************************/
    w_state->useProgram(0);
    w_state->loadLights();

    //Draw X axis in red
    w_state->loadColorMaterial(glm::vec4(1, 0, 0, 1));
    w_state->loadObjectTransforms(glm::rotate(glm::mat4(),-90.0f, glm::vec3(0, 0, 1)));
    g_axis->drawMesh();

    //Draw Y axis in green
    w_state->loadColorMaterial(glm::vec4(0, 1, 0, 1));
    w_state->loadTransforms();
    g_axis->drawMesh();

    //Draw Z axis in blue
    w_state->loadColorMaterial(glm::vec4(0, 0, 1, 1));
    w_state->loadObjectTransforms(glm::rotate(glm::mat4(),90.0f, glm::vec3(1, 0, 0)));
    g_axis->drawMesh();

    /***********************************
     * Mesh Code
     ***********************************/
    w_state->useProgram(1);
    // load values into shader
    glm::vec2 screen(c_state.width, c_state.height);

    glUniform1i(glGetUniformLocation(w_state->getCurrentProgram(), "view_mode"), c_state.view_mode);
    glUniform2fv(glGetUniformLocation(w_state->getCurrentProgram(), "scale"), 1, glm::value_ptr(screen));
    w_state->loadTransforms();
    w_state->loadMaterials();
    w_state->loadLights();
    w_state->loadTextures();
    g_mesh->drawMesh();
    
    /*************************************
     * Draw Selection Box
     *************************************/
    w_state->useProgram(2);
    glm::vec3 s_bl, s_tr;
    c_state.getMouseSelection(s_bl, s_tr);

    glUniform3fv(glGetUniformLocation(w_state->getCurrentProgram(),"bot_left"), 1, glm::value_ptr(s_bl));
    glUniform3fv(glGetUniformLocation(w_state->getCurrentProgram(),"top_right"), 1, glm::value_ptr(s_tr));
    drawBox(s_bl.x, s_bl.y, s_tr.x, s_tr.y);

    glfwSwapBuffers(c_state.window);
    glfwPollEvents();
}

// setup
int main(int argc, char *argv[])
{
	EditMesh::test();

    GLenum err = 0;
    /*********************************************
     * GLFW SETUP
     *********************************************/
    err = glfwInit();
    if (!err)
    {
        fputs("Failed to load the GLFW library", stderr);
        exit(EXIT_FAILURE);
    }

#ifdef __APPLE__
     glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	 glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	 glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	 glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
    
    /*********************************************
     * STATE SETUP (initialize gl context)
     *********************************************/
    // must be setup before glew so that a valid openGL
    // context exists (created with the window)

    w_state = new WorldState();
    c_state.init(*w_state);

    /*********************************************
     * GLEW SETUP
     *********************************************/
#ifdef __APPLE__
     glewExperimental = GL_TRUE;
#endif
    err = glewInit();
    if (err != GLEW_OK)
    {
        fputs("Failed to initialize the GLEW library", stderr);
        exit(EXIT_FAILURE);
    }

    /*********************************************
     * STATE SETUP (construct render states)
     *********************************************/
    // must be setup after glew so that GL array
    // objects exist

    r_state[0] = new RenderState(3);
    r_state[1] = new RenderState(3);

    /*********************************************
     * SHADER SETUP
     *********************************************/
    // read default shaders from file
    GLuint shaderProgram[3] = {0};
    GLuint shaders[3] = {0};

    buildShader(GL_VERTEX_SHADER, "axes.vs.glsl", shaders[0]);
    buildShader(GL_FRAGMENT_SHADER, "default.fs.glsl", shaders[1]);

    // create axis shader program
    shaderProgram[0] = buildProgram(2, shaders);

    // create the shaders for the mesh
    buildShader(GL_VERTEX_SHADER,   "mesh.vs.glsl", shaders[0]);
    buildShader(GL_GEOMETRY_SHADER, "wireframe.gs.glsl", shaders[2]);
    buildShader(GL_FRAGMENT_SHADER, "wireframe.fs.glsl", shaders[1]);
    shaderProgram[1] = buildProgram(3, shaders);

    // load shaders to render selection
    buildShader(GL_VERTEX_SHADER,   "passthrough.vs.glsl", shaders[0]);
    buildShader(GL_FRAGMENT_SHADER, "select.fs.glsl", shaders[1]);
    shaderProgram[2] = buildProgram(2, shaders);

    // bind shader program
    w_state->setProgram(0, shaderProgram[0]);
    w_state->setProgram(1, shaderProgram[1]);
    w_state->setProgram(2, shaderProgram[2]);
    w_state->useProgram(0);

    // setup the transform matrices and uniform variables
    w_state->loadTransforms();
    w_state->loadLights();
    w_state->loadMaterials();

    /*********************************************
     * LOAD MESH
     *********************************************/
	//g_mesh = loadMeshFromFile(*r_state[0], "Mesh/cube.obj");
	//g_mesh = loadMeshFromFile(*r_state[0], "debug.obj");
    g_mesh = loadMeshFromFile(*r_state[0], "Mesh/camel.obj");
	//g_mesh = loadMeshFromFile(*r_state[0], "Mesh/cow1.obj");
	//g_mesh = loadMeshFromFile(*r_state[0], "Mesh/cow_head.obj");
	//g_mesh = loadMeshFromFile(*r_state[0], "Mesh/cow_ear.obj");
    //g_mesh = loadMeshFromFile(*r_state[0], "Mesh/camel_simple.obj");
	//g_mesh = loadMeshFromFile(*r_state[0], "Mesh/cow2.obj");
	//g_mesh = loadMeshFromFile(*r_state[0], "Mesh/octopus.obj");
    //g_mesh = loadMeshFromFile(*r_state[0], "Mesh/camel.obj");
	//g_mesh = loadMeshFromFile(*r_state[0], "D:/Darcy/Development/CPSC 524 Project/Cartel/debug.obj");

    g_axis = createAxis(*r_state[1], 1);

    /*********************************************
     * SET GL STATE
     *********************************************/
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    /*********************************************
     * RENDER LOOP
     *********************************************/
    printHelp();
    glfwSetTime(0.0);
    while (!glfwWindowShouldClose(c_state.window))
        display();

    /*********************************************
     * CLEAN UP
     *********************************************/
    delete g_mesh;
    delete g_axis;

    glfwTerminate();

    exit(EXIT_SUCCESS);
}