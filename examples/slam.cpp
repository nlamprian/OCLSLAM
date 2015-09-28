/*! \file slam.cpp
 *  \brief An example presenting the process of setting up the `SLAM` pipeline.
 *  \details It accepts RGB-D data from Kinect, performs registration on the GPU, 
 *           visualizes the resulting point clouds on the screen, and creates 
 *           an Octomap map that can be saved on disk.
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
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <GL/glew.h>  // Add before CLUtils.hpp
#include <CLUtils.hpp>
#include <glut_viewer.hpp>
#include <freenect_rgbd.hpp>
#include <ocl_processing.hpp>


// Sensor parameters
Freenect::Freenect freenect;
Kinect *kinect = &freenect.createDevice<Kinect> (0);

// Map parameters    
double res = 0.1;  /*!< Map resolution in meters. */
octomap::OcTree map (res);  /*!< Map. */
// octomap::ColorOcTree map (res);

// OpenCL parameters
// Remember to change the corresponding arguments in `glut_viewer.cpp`
// const ICP::ICPStepConfigT CR = ICP::ICPStepConfigT::EIGEN;
const ICP::ICPStepConfigT CR = ICP::ICPStepConfigT::POWER_METHOD;
// const ICP::ICPStepConfigW CW = ICP::ICPStepConfigW::REGULAR;
const ICP::ICPStepConfigW CW = ICP::ICPStepConfigW::WEIGHTED;
OCLSLAM<CR, CW> *slam;


/*! \brief Displays the available controls. */
void printInfo ()
{
    std::cout << "\nAvailable Controls:\n";
    std::cout << "===================\n";
    std::cout << "  1. Autonomous SLAM, On/Off     :  S\n";
    std::cout << "  2. Manual SLAM                 :   \n";
    std::cout << "  2.1. Initialize SLAM           :  I\n";
    std::cout << "  2.2. Register a Point Cloud    :  K\n";
    std::cout << "  3. RGB Guided Filter, On/Off   :  1\n";
    std::cout << "  4. Depth Guided Filter, On/Off :  2\n";
    std::cout << "  5. RGB Normalization, On/Off   :  3\n";
    std::cout << "  6. Save Occupancy Map          :  W\n";
    std::cout << "  7. Save Binary Map             :  B\n";
    std::cout << "  8. Translate Camera            :  Arrows Keys\n";
    std::cout << "  9. Rotate Camera               :  Left Mouse Button\n";
    std::cout << " 10. Zoom In/Out                 :  Mouse Wheel\n";
    std::cout << " 11. Quit                        :  Q or Esc\n\n";
}


int main (int argc, char **argv)
{
    try
    {
        printInfo ();

        initGL (argc, argv);

        // The OpenCL environment must be created after the OpenGL environment 
        // has been initialized and before OpenGL starts rendering
        slam = new OCLSLAM<CR, CW> (kinect, map);

        glutMainLoop ();

        delete slam;

        return 0;
    }
    catch (const std::runtime_error &error)
    {
        std::cerr << "Kinect: " << error.what () << std::endl;
    }
    catch (const cl::Error &error)
    {
        std::cerr << error.what ()
                  << " (" << clutils::getOpenCLErrorCodeString (error.err ()) 
                  << ")"  << std::endl;
    }
    exit (EXIT_FAILURE);
}
