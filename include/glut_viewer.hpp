/*! \file glut_viewer.hpp
 *  \brief Declares the functions necessary for setting up the `GLUT` library.
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

#ifndef GLUT_VIEWER_HPP
#define GLUT_VIEWER_HPP

#include <vector>
#include <GL/glew.h>

#if defined(__APPLE__) || defined(__MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


/*! \brief Display callback for the window. */
void drawGLScene ();
/*! \brief Idle callback for the window. */
void idleGLScene ();
/*! \brief Reshape callback for the window. */
void resizeGLScene (int width, int height);
/*! \brief Creates a filename with a timestamp suffix. */
std::string setFilename (const char *type);
/*! \brief Keyboard callback for the window. */
void keyPressed (unsigned char key, int x, int y);
/*! \brief Arrow key callback for the window. */
void arrowPressed (int key, int x, int y);
/*! \brief Mouse callback for the window. */
void mouseMoved (int x, int y);
/*! \brief Mouse button callback for the window. */
void mouseButtonPressed (int button, int state, int x, int y);
/*! \brief Initializes GLUT. */
void initGL (int argc, char **argv);

#endif  // GLUT_VIEWER_HPP
