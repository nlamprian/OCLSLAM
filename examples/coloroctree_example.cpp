/*! \file coloroctree_example.cpp
 *  \brief An example demonstrating the integration of a 6-D point cloud 
 *         in an `octomap::ColorOcTree`.
 *  \note **Command line arguments**:
 *  \note `res`: Octree leaf resolution in meters (defaults to 10 cm).
 *  \note **Usage example**:
 *  \note `./bin/oclslam_coloroctree_example 0.01`
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
#include <vector>
#include <string>
#include <CLUtils.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>


/*! \brief Reads in a binary file.
 *  
 *  \param[in] path path to the file.
 *  \param[out] data array that receives the data.
 *  \param[in] n number of bytes to read.
 */
void fread (const char *path, char *data, size_t n)
{
    std::ifstream f (path, std::ios::binary);
    f.read (data, n);
    f.close ();
}


int main(int argc, char** argv)
{
    std::string filename = std::string ("../data/pc8d.bin");  // Coordinates in mm
    const int n = 640 * 480;
    std::vector<cl_float8> pc8d (n);

    std::cout << std::endl << "Reading point cloud data from " << filename << std::endl;
    fread (filename.c_str (), (char *) pc8d.data (), n * sizeof (cl_float8));

    double res = (argc > 1) ? std::stod (argv[1]) : 0.1;  // m
    std::cout << "Creating empty octree with leaf resolution of " << 100.0 * res << " cm" << std::endl;
    octomap::ColorOcTree tree (res);

    std::cout << "Initializing point cloud structure and color information" << std::endl;
    octomap::Pointcloud pc;
    std::vector<octomap::ColorOcTreeNode::Color> c;
    size_t m = 0;
    for (cl_float8 &pp : pc8d)
    {
        cl_float *p = (cl_float *) &pp;
        if (p[0] != 0.f && p[1] != 0.f && p[2] != 0.f)
        {
            pc.push_back (p[0] / 1000.f, p[1] / 1000.f, p[2] / 1000.f);
            c.emplace_back ((char) (255 * p[4]), (char) (255 * p[5]), (char) (255 * p[6]));
            m++;
        }
    }
    std::cout << "Done. Initialized with " << m << "/" << n << " valid points" << std::endl;

    clutils::CPUTimer<double, std::milli> cTimer;
    std::cout << "Populating octree with point cloud and setting node colors" << std::endl;
    cTimer.start ();
    
    tree.insertPointCloud (pc, octomap::point3d (0, 0, 0), -1, true, true);
    
    for (size_t i = 0; i < m; ++i)
    {
        octomap::OcTreeKey key = tree.coordToKey (pc[i]);
        octomap::ColorOcTreeNode *node = tree.search (key);
        node->setColor (c[i]);
    }

    tree.updateInnerOccupancy ();

    cTimer.stop ();
    std::cout << "Done. Elapsed time is ";
    if (cTimer.duration () < 1000.0)
        std::cout << cTimer.duration () << " ms" << std::endl;
    else
        std::cout << cTimer.duration () / 1000.0 << " s" << std::endl;

    std::cout << "Writing octree in ./coloroctree.ot" << std::endl;
    tree.write ("coloroctree.ot");
    // tree.writeBinary ("coloroctree.bt");
    std::cout << "=================================================================="  << std::endl;
    std::cout << "Visualize the tree in octovis by running: octovis ./coloroctree.ot"  << std::endl << std::endl;
}
