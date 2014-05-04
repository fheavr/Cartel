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

#include "ControlState.h"
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

ControlState c_state = ControlState();

ControlState::~ControlState()
{
    glfwDestroyWindow(window);
}

int ControlState::init(WorldState &w)
{
    this->w = &w;

    width  = 640;
    height = 480;
    /* As of right now we only have one window */
    window = glfwCreateWindow(width, height, "Window", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        fputs("failed to initialize window", stderr);
        return 1; // error
    }
    glfwMakeContextCurrent(window);

    // bind all callbacks
    glfwSetKeyCallback(window, key_callback);
    glfwSetFramebufferSizeCallback(window, reshape_callback);
    glfwSetCursorPosCallback(window, mousePos_callback);
    glfwSetCursorEnterCallback(window, mouseEnter_callback);
    glfwSetMouseButtonCallback(window, mouseBtn_callback);
    glfwSetScrollCallback(window, mouseScroll_callback);

    return 0;
}

int ControlState::deltaArrLR()
{
    return arrR - arrL;
}

int ControlState::deltaArrUD()
{
    return arrD - arrU;
}

void ControlState::updateView(float dTheta, float dPhi, float dDepth)
{
    viewTheta = fmod(viewTheta + 360 + float(dTheta), 360);
    viewPhi   = std::min(90.0f, std::max(-90.0f, viewPhi + dPhi));
    viewDepth += dDepth;
}

void printHelp()
{
    printf("==== Help ====\n"
           "___Control___\n"
           "m - switch between modes (view and select)\n\n"
           "___Command___\n"
           "q, esc - exit\n"
           "h - help (you have already figured this out)\n\n"
           "___View___\n"
           "left click and drag - adjusts view\n"
           "scroll wheel        - zoom\n");
}

/*****************************************************************************
 * Passive Callback functions
 *****************************************************************************/
// error callback for GLFW
static void error_callback(int error, const char* desc)
{
    fputs(desc, stderr);
}

// callback when window is resized
void reshape_callback(GLFWwindow* window, int w, int h)
{
    c_state.height = h;
    c_state.width  = w;

    glViewport( 0, 0, (GLint)w, (GLint)h );
}

/*****************************************************************************
 * Active Callback functions
 *****************************************************************************/
// callback when a key is pressed
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    /* key code goes here */

    switch(key)
    {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window, GL_TRUE);
        break;
    case GLFW_KEY_Q:
        glfwSetWindowShouldClose(window, GL_TRUE);
        break;
    case GLFW_KEY_H:
        printHelp();
        break;
    case GLFW_KEY_LEFT:
        c_state.arrL = (action == GLFW_RELEASE) ? 0 : 1;
        break;
    case GLFW_KEY_RIGHT:
        c_state.arrR = (action == GLFW_RELEASE) ? 0 : 1;
        break;
    case GLFW_KEY_UP:
        c_state.arrU = (action == GLFW_RELEASE) ? 0 : 1;
        break;
    case GLFW_KEY_DOWN:
        c_state.arrD = (action == GLFW_RELEASE) ? 0 : 1;
        break;
    case GLFW_KEY_L:
        c_state.reload = (action == GLFW_RELEASE) ? c_state.reload : 1;
        break;
    case GLFW_KEY_M:
        if (action == GLFW_RELEASE)
            c_state.mode = (RENDER_MODE)((c_state.mode + 1) % MODE_MAX);
        break;
    case GLFW_KEY_N:
        if (action == GLFW_RELEASE) {
            int tmp = (c_state.view_mode + 1);
            c_state.view_mode = (tmp > VIEW_ALL || tmp < VIEW_FACES) ? VIEW_FACES : tmp;
        }
        break;
	case GLFW_KEY_D:
		if( action == GLFW_RELEASE )
			c_state.op = EDIT_DEBUG;
		break;
    case GLFW_KEY_0:
        if (action == GLFW_RELEASE)
            c_state.op = EDIT_SQRT3_SUBDIV;
        break;
    case GLFW_KEY_9:
        if (action == GLFW_RELEASE)
            c_state.op = EDIT_RELOCATE_VERTS;
        break;
	case GLFW_KEY_8:
        if (action == GLFW_RELEASE)
            c_state.op = (mode & GLFW_MOD_SHIFT) ? EDIT_COLLAPSE_FAST_EDIT : EDIT_COLLAPSE_EDIT;
        break;
    case GLFW_KEY_7:
        if (action == GLFW_RELEASE)
            c_state.op = EDIT_AREA_RELOCATION;
        break;
    case GLFW_KEY_6:
        if (action == GLFW_RELEASE)
            c_state.op = EDIT_DELAUNAY;
        break;
    case GLFW_KEY_5:
        if (action == GLFW_RELEASE)
            c_state.op = EDIT_REMESH;
        break;
    case GLFW_KEY_4:
        if (action == GLFW_RELEASE)
            c_state.op = EDIT_PRETTY;
        break;
    }
    
}

// callback when a mouse button is pressed
static void mouseBtn_callback(GLFWwindow* win, int button, int action, int mod)
{
    /* TODO: any controls relative to pressing the mouse buttons goes here */

    if (button == GLFW_MOUSE_BUTTON_LEFT)
        c_state.mouseBtnL = (action == GLFW_PRESS) ? 1 : 0;
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        c_state.mouseBtnR = (action == GLFW_PRESS) ? 1 : 0;
    else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        c_state.mouseBtnC = (action == GLFW_PRESS) ? 1 : 0;

    if (action == GLFW_PRESS)
        c_state.mouseEvent = true;
    else
    {
        c_state.select_start = glm::vec3(0, 0, 0);
        c_state.select_curr = glm::vec3(0, 0, 0);
    }
}

// callback when the mouse is moved. This will be called
// ALOT keep it as light as possible!!
static void mousePos_callback(GLFWwindow* win, double x, double y)
{
    // screen Y coords are inverted.
    y = c_state.height - y;

    // currently used to update camera angles if mouse pressed
    if (c_state.mouseBtnL)
    {
        // Calculate change from last known mouse positon.
        int dx = x - c_state.mouseX;
        int dy = y - c_state.mouseY;
    
        // Update viewing angles.
        c_state.viewTheta = int(c_state.viewTheta + 360 + float(dx) / 2) % 360;
        c_state.viewPhi   = std::min(90.0f, std::max(-90.0f, c_state.viewPhi - dy));
    }

    c_state.mouseX = x;
    c_state.mouseY = y;

    if (c_state.mouseBtnL && c_state.mouseEvent)
    {
        c_state.select_start = glm::vec3(x, y, 0);
        c_state.select_curr = c_state.select_start;
        c_state.mouseEvent = false;
    }
    else if (c_state.mouseBtnL)
        c_state.select_curr = glm::vec3(x, y, 0);
}

static void mouseScroll_callback(GLFWwindow* win, double x_offset, double y_offset)
{
    c_state.mouseScroll = y_offset/20;
}

static void mouseEnter_callback(GLFWwindow* win, int entered)
{
    c_state.mouseInWindow = entered;
}