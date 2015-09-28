/*! \file testsOCLSLAM.cpp
 *  \brief Google Test Unit Tests for the `%OCLSLAM` kernels.
 *  \note Use the `--profiling` flag to enable profiling of the kernels.
 *  \note The benchmarks in these tests are against naive CPU implementations 
 *        of the associated algorithms. They are used only for testing purposes, 
 *        and not for examining the performance of their GPU alternatives.
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
#include <algorithm>
#include <chrono>
#include <random>
#include <limits>
#include <cmath>
#include <gtest/gtest.h>
#include <CLUtils.hpp>
#include <RBC/data_types.hpp>
#include <oclslam/algorithms.hpp>
#include <oclslam/tests/helper_funcs.hpp>


// Kernel filenames
const std::string kernel_filename_oclslam { "kernels/oclslam/slam_kernels.cl" };

// Uniform random number generators
namespace oclslam
{
    extern std::function<unsigned char ()> rNum_0_255;
    extern std::function<unsigned short ()> rNum_0_10000;
    extern std::function<float ()> rNum_R_0_1;
}

bool profiling;  // Flag to enable profiling of the kernels (--profiling)


/*! \brief Tests the **splitPC8D_octomap** kernel.
 *  \details The kernel splits an 8-D point cloud into 
 *           3-D coordinates (in meters) and 8-bit RGB values.
 */
TEST (OCLSLAM, splitPC8D_octomap)
{
    try
    {
        const unsigned int width = 640, height = 480;
        const unsigned int points = width * height;
        const unsigned int bufferInSize = points * sizeof (cl_float8);
        const unsigned int bufferOutPC3DSize = points * 3 * sizeof (cl_float);
        const unsigned int bufferOutRGBSize = points * 3 * sizeof (cl_uchar);

        // Setup the OpenCL environment
        clutils::CLEnv clEnv;
        clEnv.addContext (0);
        clEnv.addQueue (0, 0, CL_QUEUE_PROFILING_ENABLE);
        clEnv.addProgram (0, kernel_filename_oclslam);

        // Configure kernel execution parameters
        clutils::CLEnvInfo<1> info (0, 0, 0, { 0 }, 0);
        cl_algo::oclslam::SplitPC8D sp8D (clEnv, info);
        sp8D.init (points);

        // Initialize data (writes on staging buffer directly)
        std::generate (sp8D.hPtrIn, sp8D.hPtrIn + points, oclslam::rNum_R_0_1);
        // oclslam::printBufferF ("Original:", sp8D.hPtrIn, 8, points, 3);
        
        // Copy data to device
        sp8D.write ();

        sp8D.run ();  // Execute kernels (104 us)
        
        // Copy results to host
        cl_float *pc3d = (cl_float *) sp8D.read (cl_algo::oclslam::SplitPC8D::Memory::H_OUT_PC3D, CL_FALSE);
        cl_uchar *rgb = (cl_uchar *) sp8D.read (cl_algo::oclslam::SplitPC8D::Memory::H_OUT_RGB);
        // oclslam::printBufferF ("Received PC3D:", pc3d, 3, points, 5);
        // oclslam::printBuffer ("Received RGB:", rgb, 3, points);

        // Produce reference 8D feature points
        cl_float *refPC3D = (cl_float *) new cl_float[3 * points];
        cl_uchar *refRGB = (cl_uchar *) new cl_uchar[3 * points];
        oclslam::cpuSplitPC8D (sp8D.hPtrIn, refPC3D, refRGB, points);
        // oclslam::printBufferF ("Expected PC3D:", refPC3D, 3, points, 5);
        // oclslam::printBuffer ("Expected RGB:", refRGB, 3, points);

        // Verify the sets of points
        float eps = 42 * std::numeric_limits<float>::epsilon ();  // 5.00679e-06
        for (uint k = 0; k < 3 * points; k += 3)
        {
            for (uint j = 0; j < 3; ++j)
            {
                ASSERT_LT (std::abs (refPC3D[k + j] - pc3d[k + j]), eps);
                ASSERT_LT (std::abs (refRGB[k + j] - rgb[k + j]), eps);
            }
        }

        // Profiling ===========================================================
        if (profiling)
        {
            const int nRepeat = 1;  /* Number of times to perform the tests. */

            // CPU
            clutils::CPUTimer<double, std::milli> cTimer;
            clutils::ProfilingInfo<nRepeat> pCPU ("CPU");
            for (int i = 0; i < nRepeat; ++i)
            {
                cTimer.start ();
                oclslam::cpuSplitPC8D (sp8D.hPtrIn, refPC3D, refRGB, points);
                pCPU[i] = cTimer.stop ();
            }
            
            // GPU
            clutils::GPUTimer<std::milli> gTimer (clEnv.devices[0][0]);
            clutils::ProfilingInfo<nRepeat> pGPU ("GPU");
            for (int i = 0; i < nRepeat; ++i)
                pGPU[i] = sp8D.run (gTimer);

            // Benchmark
            pGPU.print (pCPU, "splitPC8D_octomap");
        }

    }
    catch (const cl::Error &error)
    {
        std::cerr << error.what ()
                  << " (" << clutils::getOpenCLErrorCodeString (error.err ()) 
                  << ")"  << std::endl;
        exit (EXIT_FAILURE);
    }
}


int main (int argc, char **argv)
{
    profiling = oclslam::setProfilingFlag (argc, argv);

    ::testing::InitGoogleTest (&argc, argv);

    return RUN_ALL_TESTS ();
}
