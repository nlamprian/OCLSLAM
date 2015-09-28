/*! \file glut_viewer.cpp
 *  \brief Defines the functions necessary for setting up the `GLUT` library.
 *  \author Nick Lamprianidis
 *  \version 0.1.0
 *  \date 2015
 *  \copyright The MIT License (MIT)
 *  \par
 *  Copyright (c) 2015 Nick Lamprianidis
 *  \par
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  \par
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *  \par
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */

#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <ctime>
#include <glut_viewer.hpp>
#include <ocl_processing.hpp>


// Window parameters
const int gl_win_width = 640;
const int gl_win_height = 480;
int glWinId;

// Model parameters
int mouseX = -1, mouseY = -1;
float dx = 0.f, dy = 0.f;
float angleX = 0.f, angleY = 0.f;
float zoom = 1.0f;

// OpenGL buffer parameters
GLuint glPC4DBuffer, glRGBABuffer;

// Point cloud parameters
const int width = 640;
const int height = 480;

// OpenCL parameters
extern OCLSLAM<ICP::ICPStepConfigT::POWER_METHOD, 
    ICP::ICPStepConfigW::WEIGHTED> *slam;

extern std::mutex glMtx;  // Controls access to OpenGL buffers
extern std::mutex mapMtx;  // Controls access to the map


void drawGLScene ()
{
    if (!glMtx.try_lock ()) return;

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // glPointSize(2.f);

    glBindBuffer (GL_ARRAY_BUFFER, glPC4DBuffer);
    glVertexPointer (4, GL_FLOAT, 0, NULL);
    glEnableClientState (GL_VERTEX_ARRAY);
    
    glBindBuffer (GL_ARRAY_BUFFER, glRGBABuffer);
    glColorPointer (4, GL_FLOAT, 0, NULL);
    glEnableClientState (GL_COLOR_ARRAY);

    glDrawArrays (GL_POINTS, 0, slam->timeStep * width * height);

    glDisableClientState (GL_VERTEX_ARRAY);
    glDisableClientState (GL_COLOR_ARRAY);
    glBindBuffer (GL_ARRAY_BUFFER, 0);

    // Draw the world coordinate frame
    glLineWidth (2.f);
    glBegin (GL_LINES);
    glColor3ub (255, 0, 0);
    glVertex3i (  0, 0, 0);
    glVertex3i ( 50, 0, 0);

    glColor3ub (0, 255, 0);
    glVertex3i (0,   0, 0);
    glVertex3i (0,  50, 0);

    glColor3ub (0, 0, 255);
    glVertex3i (0, 0,   0);
    glVertex3i (0, 0,  50);
    glEnd ();

    // Position the camera
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();
    glScalef (zoom, zoom, 1);
    gluLookAt ( -7*angleX, -7*angleY, -1000.0,
                      0.0,       0.0,  2000.0,
                      0.0,      -1.0,     0.0 );
    glTranslatef (dx, dy, 0.f);

    glutSwapBuffers ();

    glMtx.unlock ();
}


void idleGLScene ()
{
    glutPostRedisplay ();
}


void resizeGLScene (int width, int height)
{
    glViewport (0, 0, width, height);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective (70.0, width / (float) height, 900.0, 11000.0);
    glMatrixMode (GL_MODELVIEW);
}


/*! \details The timestamp has the following format YYYYMMDDHHMMSS.
 *
 *  \param[in] type file type.
 *  \return A complete filename with the current timestamp.
 */
std::string setFilename (const char *type)
{
    time_t now = time (0);
    tm *ltm = localtime (&now);

    std::ostringstream dt;
    dt << "map_" << 1900 + ltm->tm_year << 1 + ltm->tm_mon << ltm->tm_mday 
       << ltm->tm_hour << ltm->tm_min << ltm->tm_sec << "." << type;

    return dt.str ();
}


void keyPressed (unsigned char key, int x, int y)
{
    switch (key)
    {
        case 0x1B:  // ESC
        case  'Q':
        case  'q':
            glMtx.lock ();
            mapMtx.lock ();
            glutDestroyWindow (glWinId);
            break;
        case '1':
            slam->toggleGFRGBStatus ();
            std::cout << "RGB Guided Filter " << slam->getGFRGBStatus () << std::endl;
            break;
        case '2':
            slam->toggleGFDStatus ();
            std::cout << "Depth Guided Filter " << slam->getGFDStatus () << std::endl;
            break;
        case '3':
            slam->toggleRGBNormalization ();
            std::cout << "RGB Normalization " << slam->getRGBNormalization () << std::endl;
            break;
        case 'S':
        case 's':
            slam->toggleSLAMStatus ();
            std::cout << "SLAM " << slam->getSLAMStatus () << std::endl;
            break;
        case 'I':
        case 'i':
            std::thread ([&] { slam->init (); }).detach ();
            break;
        case 'K':
        case 'k':
            std::thread ([&] { slam->registerPointCloud (); }).detach ();
            break;
        case 'W':
        case 'w':
            std::thread ([&] { slam->write (setFilename ("ot")); }).detach ();
            break;
        case 'B':
        case 'b':
            std::thread ([&] { slam->writeBinary (setFilename ("bt")); }).detach ();
            break;
    }
}


void arrowPressed (int key, int x, int y)
{
    switch (key)
    {
        case GLUT_KEY_RIGHT:
            dx -= 200;
            break;
        case GLUT_KEY_LEFT:
            dx += 200;
            break;
        case GLUT_KEY_DOWN:
            dy -= 200;
            break;
        case GLUT_KEY_UP:
            dy += 200;
            break;
    }
}


void mouseMoved (int x, int y)
{
    if (mouseX >= 0 && mouseY >= 0)
    {
        angleX += x - mouseX;
        angleY += y - mouseY;
    }

    mouseX = x;
    mouseY = y;
}


void mouseButtonPressed (int button, int state, int x, int y)
{
    if (state == GLUT_DOWN)
    {
        switch (button)
        {
            case GLUT_LEFT_BUTTON:
                mouseX = x;
                mouseY = y;
                break;
            case 3:  // Scroll Up
                zoom *= 1.2f;
                break;
            case 4:  // Scroll Down
                zoom /= 1.2f;
                break;
        }
    }
    else if (state == GLUT_UP && button == GLUT_LEFT_BUTTON)
    {
        mouseX = -1;
        mouseY = -1;
    }
}


void initGL (int argc, char **argv)
{
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA);
    glutInitWindowSize (gl_win_width, gl_win_height);
    glutInitWindowPosition ((glutGet (GLUT_SCREEN_WIDTH) - gl_win_width) / 2,
                            (glutGet (GLUT_SCREEN_HEIGHT) - gl_win_height) / 2 - 70);
    glWinId = glutCreateWindow ("OCLSLAM");

    glutDisplayFunc (&drawGLScene);
    glutIdleFunc (&idleGLScene);
    glutReshapeFunc (&resizeGLScene);
    glutKeyboardFunc (&keyPressed);
    glutSpecialFunc (&arrowPressed);
    glutMotionFunc (&mouseMoved);
    glutMouseFunc (&mouseButtonPressed);

    glewInit ();

    glClearColor (0.7f, 0.7f, 0.7f, 1.f);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable (GL_ALPHA_TEST);
    glAlphaFunc (GL_GREATER, 0.f);
    glEnable (GL_DEPTH_TEST);
    glShadeModel (GL_SMOOTH);
}
