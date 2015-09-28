/*! \file helper_funcs.hpp
 *  \brief Declarations of helper functions for testing.
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

#ifndef OCLSLAM_HELPERFUNCS_HPP
#define OCLSLAM_HELPERFUNCS_HPP

#include <cassert>
#include <algorithm>
#include <functional>
#include <RBC/data_types.hpp>

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/cl.hpp>
#else
#include <CL/cl.hpp>
#endif


/*! \brief Offers functions that are serial `CPU` implementations of 
 *         the relevant algorithms in the `%OCLSLAM` pipeline.
 */
namespace oclslam
{

    /*! \brief Checks the command line arguments for the profiling flag, `--profiling`. */
    bool setProfilingFlag (int argc, char **argv);


    /*! \brief Returns the first power of 2 greater than or equal to the input.
     *
     *  \param[in] num input number.
     *  \return The first power of 2 >= num.
     */
    template <typename T>
    uint64_t nextPow2 (T num)
    {
        assert (num >= 0);

        uint64_t pow;
        for (pow = 1; pow < (uint64_t) num; pow <<= 1) ;

        return pow;
    }


    /*! \brief Prints an array of an integer type to standard output.
     *
     *  \tparam T type of the data to be printed.
     *  \param[in] title legend for the output.
     *  \param[in] ptr array that is to be displayed.
     *  \param[in] width width of the array.
     *  \param[in] height height of the array.
     */
    template <typename T>
    void printBuffer (const char *title, T *ptr, uint32_t width, uint32_t height)
    {
        std::cout << title << std::endl;

        for (int row = 0; row < height; ++row)
        {
            for (int col = 0; col < width; ++col)
            {
                std::cout << std::setw (3 * sizeof (T)) << +ptr[row * width + col] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << std::endl;
    }


    /*! \brief Prints an array of floating-point type to standard output.
     *
     *  \tparam T type of the data to be printed.
     *  \param[in] title legend for the output.
     *  \param[in] ptr array that is to be displayed.
     *  \param[in] width width of the array.
     *  \param[in] height height of the array.
     *  \param[in] prec the number of decimal places to print.
     */
    template <typename T>
    void printBufferF (const char *title, T *ptr, uint32_t width, uint32_t height, uint32_t prec)
    {
        std::ios::fmtflags f (std::cout.flags ());
        std::cout << title << std::endl;
        std::cout << std::fixed << std::setprecision (prec);

        for (int row = 0; row < height; ++row)
        {
            for (int col = 0; col < width; ++col)
            {
                std::cout << std::setw (5 + prec) << ptr[row * width + col] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << std::endl;
        std::cout.flags (f);
    }


    /*! \brief Splits an 8-D point cloud into 3-D coordinates 
     *         (in meters) and 8-bit RGB values.
     *  \details It is just a naive serial implementation.
     *
     *  \param[in] pc8d array with 8-D points (homogeneous coordiates + RGBA values).
     *  \param[out] pc3d array with 3-D coordinates (in meters).
     *  \param[out] rgb array with 8-bit RGB values.
     *  \param[in] n number of points in the 8-D point cloud.
     */
    template <typename T>
    void cpuSplitPC8D (T *pc8d, T *pc3d, cl_uchar *rgb, uint32_t n)
    {
        for (uint k = 0; k < n; ++k)
        {
            cl_float *point = pc8d + (k << 3);

            for (uint j = 0; j < 3; ++j)
            {
                pc3d[3 * k + j] = point[j] * 0.001;
                rgb[3 * k + j] = (cl_uchar) (point[4 + j] * 255);
            }
        }
    }

}

#endif  // OCLSLAM_HELPERFUNCS_HPP
