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

#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include "WorldState.h"

/* due to GLFW/glut/most windowing systems being c based it is infeasible to store 
 * all the control state within our class and pass member function pointers (the c 
 * interface does not allow it) thus the callback functions must be defined external
 * to the class
 */
#define DEGREES_PER_SECOND 100
#define DEPTH_PER_SECOND 3
#define STEP_PER_SECOND 0.03f

#define VIEW_FACES 0x1
#define VIEW_EDGES 0x2
#define VIEW_VERTS 0x4
#define VIEW_ALL (VIEW_FACES | VIEW_EDGES | VIEW_VERTS)

enum RENDER_MODE
{
    MODE_VIEW=0,
    MODE_SELECT,
    MODE_MAX,
};

enum EDIT_OPERATION
{
    EDIT_NONE = 0,
    EDIT_SQRT3_SUBDIV,
    EDIT_RELOCATE_VERTS,
	EDIT_COLLAPSE_EDIT,
    EDIT_AREA_RELOCATION,
    EDIT_DELAUNAY,
    EDIT_REMESH,
	EDIT_COLLAPSE_FAST_EDIT,
    EDIT_PRETTY,
	EDIT_DEBUG,
    EDIT_MAX,
};

class ControlState
{
public:
    float viewTheta;
    float viewPhi;
    float viewDepth;

    // the current state of arrow keys, updated by the
    bool arrL;
    bool arrR;
    bool arrU;
    bool arrD;

    // the current mouse position, updated by the mousePose_callback function
    int mouseX;
    int mouseY;
    bool mouseEvent; // did the mouse state just change, or is it held
    bool mouseBtnL;
    bool mouseBtnC;
    bool mouseBtnR;

    // unprocessed mouse scroll amount
    float mouseScroll;
    bool mouseInWindow;

    // the current dimensions of the window
    int height;
    int width;

    // window that this control state is for
    GLFWwindow* window;
    WorldState* w;

    //program specific control scheme
    glm::vec3 select_start;
    glm::vec3 select_curr;
    glm::vec3 select_end;
    RENDER_MODE mode;
    EDIT_OPERATION op;
    int view_mode;
    bool reload;

    ControlState()
        : viewTheta(0),
          viewPhi(0),
          viewDepth(1),
          arrL(0), arrR(0),
          arrU(0), arrD(0),
          mouseX(0), mouseY(0),
          mouseBtnL(0), mouseBtnC(0),
          mouseBtnR(0),
          window(NULL), w(NULL),
          mode(MODE_SELECT),
          view_mode(VIEW_FACES | VIEW_EDGES),
          op(EDIT_NONE)
    {}

    ~ControlState();
    int init(WorldState &w);

    float aspectRatio()
    {return (float)width/height;}

    void splitViewportLeft()
    {glViewport(0, 0, width/2, height);}
    void splitViewportRight()
    {glViewport(width/2, 0, width/2, height);}
    void splitViewportTop()
    {glViewport(0, height/2, width, height/2);}
    void splitViewportBottom()
    {glViewport(0, 0, width, height/2);}
    void viewportFull()
    {glViewport(0, 0, width, height);}

    int deltaArrLR();
    int deltaArrUD();
    void updateView(float dTheta, float dPhi, float dDepth);

    void getMouseSelection (glm::vec3 &bottomleft, glm::vec3 &topright)
    {
        if (select_start.x > select_curr.x)
        { bottomleft.x = select_curr.x;
          topright.x = select_start.x; }
        else
        { bottomleft.x = select_start.x;
          topright.x = select_curr.x; }
        if (select_start.y > select_curr.y)
        { bottomleft.y = select_curr.y;
          topright.y = select_start.y; }
        else
        { bottomleft.y = select_start.y;
          topright.y = select_curr.y; }
    }
};
extern ControlState c_state;

extern void printHelp();

/* GLFW callback funtions */
static void error_callback(int error, const char* desc);
static void reshape_callback(GLFWwindow* win, int w, int h);
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
static void mouseBtn_callback(GLFWwindow* win, int button, int action, int mod);
static void mousePos_callback(GLFWwindow* win, double x, double y);
static void mouseScroll_callback(GLFWwindow* win, double x_offset, double y_offset);
static void mouseEnter_callback(GLFWwindow* win, int entered);

#endif // CONTROL_STATE_H