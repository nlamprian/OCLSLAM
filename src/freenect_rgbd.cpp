/*! \file freenect_rgbd.cpp
 *  \brief Defines a class necessary for setting up the freenect library.
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

#include <freenect_rgbd.hpp>


/*! \note The creation of the device is done through the Freenect class.
 *
 *  \param[in] ctx context to open device through (handled by the library).
 *  \param[in] idx index of the device on the bus.
 */
Kinect::Kinect (freenect_context *ctx, int idx) : 
    Freenect::FreenectDevice (ctx, idx), newRGBFrame (false), newDepthFrame (false), 
    width (freenect_find_video_mode (FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).width), 
    height (freenect_find_video_mode (FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).height)
{
    // setVideoFormat (FREENECT_VIDEO_YUV_RGB, FREENECT_RESOLUTION_MEDIUM);
    setDepthFormat (FREENECT_DEPTH_REGISTERED);
}


/*! \note Do not call directly, it's only used by the library.
 *  
 *  \param[in] rgb an array holding the rgb frame.
 *  \param[in] timestamp a time stamp.
 */
void Kinect::VideoCallback (void *rgb, uint32_t timestamp)
{
    std::lock_guard<std::mutex> lock (rgbMutex);
    
    std::copy ((cl_uchar *) rgb, (cl_uchar *) rgb + getVideoBufferSize (), rgbPtr);
    newRGBFrame = true;
}


/*! \note Do not call directly, it's only used by the library.
 *  
 *  \param[in] depth an array holding the depth frame.
 *  \param[in] timestamp a time stamp.
 */
void Kinect::DepthCallback (void *depth, uint32_t timestamp)
{
    std::lock_guard<std::mutex> lock (depthMutex);
    
    std::copy ((cl_ushort *) depth, (cl_ushort *) depth + getDepthBufferSize () / 2, depthPtr);
    newDepthFrame = true;
}


/*! \details It maps the buffers and works with the returned pointers.
 *  
 *  \param[in] queue command queue associated with the buffers.
 *  \param[in] hBufferRGB OpenCL buffer for the RGB frame.
 *  \param[in] hBufferD OpenCL buffer for the Depth frame.
 */
void Kinect::setBuffers (cl::CommandQueue &queue, cl::Buffer &hBufferRGB, cl::Buffer &hBufferD)
{
    rgbPtr = (cl_uchar *) queue.enqueueMapBuffer (
        hBufferRGB, CL_FALSE, CL_MAP_WRITE, 0, width * height * 3 * sizeof (cl_uchar));
    depthPtr = (cl_ushort *) queue.enqueueMapBuffer (
        hBufferD, CL_FALSE, CL_MAP_WRITE, 0, width * height * sizeof (cl_ushort));
    queue.enqueueUnmapMemObject (hBufferRGB, rgbPtr);
    queue.enqueueUnmapMemObject (hBufferD, depthPtr);
    queue.finish ();
}


/*! \note Transfers the frames from the staging buffers to the provided device buffers.
 *
 *  \param[in] queue command queue that will handle the frame transfers.
 *  \param[out] rgb OpenCL buffer to which to transfer the RGB frame.
 *  \param[out] depth OpenCL buffer to which to transfer the Depth frame.
 *  \return A flag to indicate whether new frames were present and got transfered.
 */
bool Kinect::deliverFrames (cl::CommandQueue &queue, cl::Buffer &rgb, cl::Buffer &depth)
{
    std::lock_guard<std::mutex> lockRGB (rgbMutex);
    std::lock_guard<std::mutex> lockDepth (depthMutex);
    
    if (!newRGBFrame || !newDepthFrame)
        return false;

    // Finish the transfer before unlocking the mutex
    queue.enqueueWriteBuffer (rgb, CL_FALSE, 0, getVideoBufferSize (), (void *) rgbPtr);
    queue.enqueueWriteBuffer (depth, CL_TRUE, 0, getDepthBufferSize (), (void *) depthPtr);
    
    newRGBFrame = false;
    newDepthFrame = false;

    return true;
}
