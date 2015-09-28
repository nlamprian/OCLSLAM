/*! \file slam_kernels.cl
 *  \brief Kernels for the `%OCLSLAM` pipeline.
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


/*! \brief Splits an 8-D point cloud into 3-D coordinates (in meters) 
 *         and 8-bit RGB values.
 *  \note The global workspace should be one-dimensional. The **x** dimension 
 *        of the global workspace, \f$ gXdim \f$, should be equal to the number 
 *        of points in the point cloud. The local workspace is irrelevant.
 *
 *  \param[in] pc8d array with 8-D points (homogeneous coordinates + RGBA values).
 *  \param[out] pc3d array with 3-D coordinates (in meters).
 *  \param[out] rgb array with 8-bit RGB values.
 */
kernel
void splitPC8D_octomap (global float8 *pc8d, global float *pc3d, global uchar *rgb)
{
    uint gX = get_global_id (0);

    float8 point = pc8d[gX];
    vstore3 (point.s012 * 0.001f, gX, pc3d);
    vstore3 (convert_uchar3 (point.s456 * 255.f), gX, rgb);
}
