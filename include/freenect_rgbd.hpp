/*! \file freenect_rgbd.hpp
 *  \brief Declares classes necessary for setting up the freenect library.
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

#ifndef FREENECT_RGBD_HPP
#define FREENECT_RGBD_HPP

#include <mutex>
#include <libfreenect.hpp>

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/cl.hpp>
#else
#include <CL/cl.hpp>
#endif


/*! \brief A class that extends Freenect::FreenectDevice by defining 
 *         the `VideoCallback` and `DepthCallback` functions so we can 
 *         be getting updates with the latest RGB and Depth frames.
 */
class Kinect : public Freenect::FreenectDevice
{
public:
    Kinect (freenect_context *ctx, int idx);
    /*! \brief Delivers the latest RGB frame. */
    void VideoCallback (void *rgb, uint32_t timestamp);
    /*! \brief Delivers the latest Depth frame. */
    void DepthCallback (void *depth, uint32_t timestamp);
    /*! \brief Sets the host buffers for the RGB and Depth frames. */
    void setBuffers (cl::CommandQueue &queue, cl::Buffer &rgb, cl::Buffer &depth);
    /*! \brief Transfers the RGB and Depth frames to the specified OpenCL buffers. */
    bool deliverFrames (cl::CommandQueue &queue, cl::Buffer &rgb, cl::Buffer &depth);

private:
    std::mutex rgbMutex, depthMutex;
    cl_uchar *rgbPtr;  // Aligned to 4KB for pinning in OpenCL
    cl_ushort *depthPtr;  // Aligned to 4KB for pinning in OpenCL
    bool newRGBFrame, newDepthFrame;
    unsigned int width, height;

};

#endif  // FREENECT_RGBD_HPP
