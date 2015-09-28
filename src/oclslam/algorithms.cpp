/*! \file algorithms.cpp
 *  \brief Defines classes that organize the execution of OpenCL kernels.
 *  \details Each class hides the details of the execution of a kernel. They
 *           initialize the necessary buffers, set up the workspaces, and 
 *           run the kernels.
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
#include <cmath>
#include <CLUtils.hpp>
#include <oclslam/algorithms.hpp>


/*! \note All the classes assume there is a fully configured `clutils::CLEnv` 
 *        environment. This means, there is a known context on which they will 
 *        operate, there is a known command queue which they will use, and all 
 *        the necessary kernel code has been compiled. For more info on **CLUtils**, 
 *        you can check the [online documentation](http://clutils.paign10.me/).
 */
namespace cl_algo
{
namespace oclslam
{

    /*! \param[in] _env opencl environment.
     *  \param[in] _info opencl configuration. Specifies the context, queue, etc, to be used.
     */
    SplitPC8D::SplitPC8D (clutils::CLEnv &_env, clutils::CLEnvInfo<1> _info) : 
        env (_env), info (_info), 
        context (env.getContext (info.pIdx)), 
        queue (env.getQueue (info.ctxIdx, info.qIdx[0])), 
        kernel (env.getProgram (info.pgIdx), "splitPC8D_octomap")
    {
    }


    /*! \details This interface exists to allow CL memory sharing between different kernels.
     *
     *  \param[in] mem enumeration value specifying the requested memory object.
     *  \return A reference to the requested memory object.
     */
    cl::Memory& SplitPC8D::get (SplitPC8D::Memory mem)
    {
        switch (mem)
        {
            case SplitPC8D::Memory::H_IN:
                return hBufferIn;
            case SplitPC8D::Memory::H_OUT_PC3D:
                return hBufferOutPC3D;
            case SplitPC8D::Memory::H_OUT_RGB:
                return hBufferOutRGB;
            case SplitPC8D::Memory::D_IN:
                return dBufferIn;
            case SplitPC8D::Memory::D_OUT_PC3D:
                return dBufferOutPC3D;
            case SplitPC8D::Memory::D_OUT_RGB:
                return dBufferOutRGB;
        }
    }


    /*! \details Sets up memory objects as necessary, and defines the kernel workspaces.
     *  \note If you have assigned a memory object to one member variable of the class 
     *        before the call to `init`, then that memory will be maintained. Otherwise, 
     *        a new memory object will be created.
     *        
     *  \param[in] _n number of points in the point cloud.
     *  \param[in] _staging flag to indicate whether or not to instantiate the staging buffers.
     */
    void SplitPC8D::init (unsigned int _n, Staging _staging)
    {
        n = _n;
        bufferInSize = n * sizeof (cl_float8);
        bufferOutPC3DSize = n * 3 * sizeof (cl_float);
        bufferOutRGBSize = n * 3 * sizeof (cl_uchar);
        staging = _staging;

        try
        {
            if (n == 0)
                throw "The point cloud cannot be empty";
        }
        catch (const char *error)
        {
            std::cerr << "Error[SplitPC8D]: " << error << std::endl;
            exit (EXIT_FAILURE);
        }

        // Set workspace
        global = cl::NDRange (n);

        // Create staging buffers
        bool io = false;
        switch (staging)
        {
            case Staging::NONE:
                hPtrIn = nullptr;
                hPtrOutPC3D = nullptr;
                hPtrOutRGB = nullptr;
                break;

            case Staging::IO:
                io = true;

            case Staging::I:
                if (hBufferIn () == nullptr)
                    hBufferIn = cl::Buffer (context, CL_MEM_ALLOC_HOST_PTR, bufferInSize);

                hPtrIn = (cl_float *) queue.enqueueMapBuffer (
                    hBufferIn, CL_FALSE, CL_MAP_WRITE, 0, bufferInSize);
                queue.enqueueUnmapMemObject (hBufferIn, hPtrIn);

                if (!io)
                {
                    queue.finish ();
                    hPtrOutPC3D = nullptr;
                    hPtrOutRGB = nullptr;
                    break;
                }

            case Staging::O:
                if (hBufferOutPC3D () == nullptr)
                    hBufferOutPC3D = cl::Buffer (context, CL_MEM_ALLOC_HOST_PTR, bufferOutPC3DSize);
                if (hBufferOutRGB () == nullptr)
                    hBufferOutRGB = cl::Buffer (context, CL_MEM_ALLOC_HOST_PTR, bufferOutRGBSize);

                hPtrOutPC3D = (cl_float *) queue.enqueueMapBuffer (
                    hBufferOutPC3D, CL_FALSE, CL_MAP_READ, 0, bufferOutPC3DSize);
                hPtrOutRGB = (cl_uchar *) queue.enqueueMapBuffer (
                    hBufferOutRGB, CL_FALSE, CL_MAP_READ, 0, bufferOutRGBSize);
                queue.enqueueUnmapMemObject (hBufferOutPC3D, hPtrOutPC3D);
                queue.enqueueUnmapMemObject (hBufferOutRGB, hPtrOutRGB);
                queue.finish ();

                if (!io) hPtrIn = nullptr;
                break;
        }
        
        // Create device buffers
        if (dBufferIn () == nullptr)
            dBufferIn = cl::Buffer (context, CL_MEM_READ_ONLY, bufferInSize);
        if (dBufferOutPC3D () == nullptr)
            dBufferOutPC3D = cl::Buffer (context, CL_MEM_WRITE_ONLY, bufferOutPC3DSize);
        if (dBufferOutRGB () == nullptr)
            dBufferOutRGB = cl::Buffer (context, CL_MEM_WRITE_ONLY, bufferOutRGBSize);

        // Set kernel arguments
        kernel.setArg (0, dBufferIn);
        kernel.setArg (1, dBufferOutPC3D);
        kernel.setArg (2, dBufferOutRGB);
    }


    /*! \details The transfer happens from a staging buffer on the host to the 
     *           associated (specified) device buffer.
     *  
     *  \param[in] mem enumeration value specifying an input device buffer.
     *  \param[in] ptr a pointer to an array holding input data. If not NULL, the 
     *                 data from `ptr` will be copied to the associated staging buffer.
     *  \param[in] block a flag to indicate whether to perform a blocking 
     *                   or a non-blocking operation.
     *  \param[in] events a wait-list of events.
     *  \param[out] event event associated with the write operation to the device buffer.
     */
    void SplitPC8D::write (SplitPC8D::Memory mem, void *ptr, bool block, 
                           const std::vector<cl::Event> *events, cl::Event *event)
    {
        if (staging == Staging::I || staging == Staging::IO)
        {
            switch (mem)
            {
                case SplitPC8D::Memory::D_IN:
                    if (ptr != nullptr)
                        std::copy ((cl_float8 *) ptr, (cl_float8 *) ptr + n, (cl_float8 *) hPtrIn);
                    queue.enqueueWriteBuffer (dBufferIn, block, 0, bufferInSize, hPtrIn, events, event);
                    break;
                default:
                    break;
            }
        }
    }


    /*! \details The transfer happens from a device buffer to the associated 
     *           (specified) staging buffer on the host.
     *  
     *  \param[in] mem enumeration value specifying an output staging buffer.
     *  \param[in] block a flag to indicate whether to perform a blocking 
     *                   or a non-blocking operation.
     *  \param[in] events a wait-list of events.
     *  \param[out] event event associated with the read operation to the staging buffer.
     */
    void* SplitPC8D::read (SplitPC8D::Memory mem, bool block, 
                           const std::vector<cl::Event> *events, cl::Event *event)
    {
        if (staging == Staging::O || staging == Staging::IO)
        {
            switch (mem)
            {
                case SplitPC8D::Memory::H_OUT_PC3D:
                    queue.enqueueReadBuffer (dBufferOutPC3D, block, 0, bufferOutPC3DSize, hPtrOutPC3D, events, event);
                    return hPtrOutPC3D;
                case SplitPC8D::Memory::H_OUT_RGB:
                    queue.enqueueReadBuffer (dBufferOutRGB, block, 0, bufferOutRGBSize, hPtrOutRGB, events, event);
                    return hPtrOutRGB;
                default:
                    return nullptr;
            }
        }
        return nullptr;
    }


    /*! \details The function call is non-blocking.
     *
     *  \param[in] events a wait-list of events.
     *  \param[out] event event associated with the last kernel execution.
     */
    void SplitPC8D::run (const std::vector<cl::Event> *events, cl::Event *event)
    {
        queue.enqueueNDRangeKernel (kernel, cl::NullRange, global, cl::NullRange, events, event);
    }

}
}
