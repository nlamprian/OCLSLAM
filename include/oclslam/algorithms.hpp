/*! \file algorithms.hpp
 *  \brief Declares classes that organize the execution of OpenCL kernels.
 *  \details Each class hides the details of kernel execution. They
 *           initialize the necessary buffers, set up the workspaces, 
 *           and run the kernels.
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

#ifndef OCLSLAM_ALGORITHMS_HPP
#define OCLSLAM_ALGORITHMS_HPP

#include <CLUtils.hpp>
#include <oclslam/common.hpp>
#include <RBC/data_types.hpp>
#include <RBC/algorithms.hpp>
#include <eigen3/Eigen/Dense>


/*! \brief Offers classes which set up kernel execution parameters and 
 *         provide interfaces for the handling of memory objects.
 */
namespace cl_algo
{
/*! \brief Offers classes associated with the `%OCLSLAM` pipeline. */
namespace oclslam
{

    /*! \brief Interface class for the `splitPC8D_octomap` kernel.
     *  \details `splitPC8D_octomap` splits an 8-D point cloud into 3-D coordinates (in meters) 
     *           and 8-bit RGB values for use with OctoMap data structures.
     *           For more details, look at the kernel's documentation.
     *  \note The `splitPC8D_octomap` kernel is available in `kernels/slam_kernels.cl`.
     *  \note The class creates its own buffers. If you would like to provide 
     *        your own buffers, call `get` to get references to the placeholders 
     *        within the class and assign them to your buffers. You will have to 
     *        do this strictly before the call to `init`. You can also call `get` 
     *        (after the call to `init`) to get a reference to a buffer within 
     *        the class and assign it to another kernel class instance further 
     *        down in your task pipeline.
     *  
     *        The following input/output `OpenCL` memory objects are created by a `SplitPC8D` instance:<br>
     *        | Name | Type | Placement | I/O | Use | Properties | Size |
     *        | ---  |:---: |   :---:   |:---:|:---:|   :---:    |:---: |
     *        | H_IN      | Buffer | Host   | I | Staging     | CL_MEM_READ_WRITE | \f$  width*height*sizeof\ (cl\_float8)\f$ |
     *        | H_OUT_PC3D| Buffer | Host   | O | Staging     | CL_MEM_READ_WRITE | \f$3*width*height*sizeof\ (cl\_float) \f$ |
     *        | H_OUT_RGB | Buffer | Host   | O | Staging     | CL_MEM_READ_WRITE | \f$3*width*height*sizeof\ (cl\_uchar) \f$ |
     *        | D_IN      | Buffer | Device | I | Processing  | CL_MEM_READ_ONLY  | \f$  width*height*sizeof\ (cl\_float8)\f$ |
     *        | D_OUT_PC3D| Buffer | Device | O | Processing  | CL_MEM_WRITE_ONLY | \f$3*width*height*sizeof\ (cl\_float) \f$ |
     *        | D_OUT_RGB | Buffer | Device | O | Processing  | CL_MEM_WRITE_ONLY | \f$3*width*height*sizeof\ (cl\_uchar) \f$ |
     */
    class SplitPC8D
    {
    public:
        /*! \brief Enumerates the memory objects handled by the class.
         *  \note `H_*` names refer to staging buffers on the host.
         *  \note `D_*` names refer to buffers on the device.
         */
        enum class Memory : uint8_t
        {
            H_IN,        /*!< Input staging buffer for the 8-D point cloud. */
            H_OUT_PC3D,  /*!< Output staging buffer for the 3-D coordinates. */
            H_OUT_RGB,   /*!< Output staging buffer for the RGB values. */
            D_IN,        /*!< Input buffer for the 8-D point cloud. */
            D_OUT_PC3D,  /*!< Output buffer for the 4-D coordinates. */
            D_OUT_RGB    /*!< Output buffer for the RGB values. */
        };

        /*! \brief Configures an OpenCL environment as specified by `_info`. */
        SplitPC8D (clutils::CLEnv &_env, clutils::CLEnvInfo<1> _info);
        /*! \brief Returns a reference to an internal memory object. */
        cl::Memory& get (SplitPC8D::Memory mem);
        /*! \brief Configures kernel execution parameters. */
        void init (unsigned int _n, Staging _staging = Staging::IO);
        /*! \brief Performs a data transfer to a device buffer. */
        void write (SplitPC8D::Memory mem = SplitPC8D::Memory::D_IN, void *ptr = nullptr, bool block = CL_FALSE, 
                    const std::vector<cl::Event> *events = nullptr, cl::Event *event = nullptr);
        /*! \brief Performs a data transfer to a staging buffer. */
        void* read (SplitPC8D::Memory mem = SplitPC8D::Memory::H_OUT_PC3D, bool block = CL_TRUE, 
                    const std::vector<cl::Event> *events = nullptr, cl::Event *event = nullptr);
        /*! \brief Executes the necessary kernels. */
        void run (const std::vector<cl::Event> *events = nullptr, cl::Event *event = nullptr);

        cl_float *hPtrIn;       /*!< Mapping of the input staging buffer for the 8-D point cloud. */
        cl_float *hPtrOutPC3D;  /*!< Mapping of the output staging buffer for the 3-D coordinates. */
        cl_uchar *hPtrOutRGB;   /*!< Mapping of the output staging buffer for the RGB values. */

    private:
        clutils::CLEnv &env;
        clutils::CLEnvInfo<1> info;
        cl::Context context;
        cl::CommandQueue queue;
        cl::Kernel kernel;
        cl::NDRange global;
        Staging staging;
        unsigned int n;
        unsigned int bufferInSize, bufferOutPC3DSize, bufferOutRGBSize;
        cl::Buffer hBufferIn, hBufferOutPC3D, hBufferOutRGB;
        cl::Buffer dBufferIn, dBufferOutPC3D, dBufferOutRGB;

    public:
        /*! \brief Executes the necessary kernels.
         *  \details This `run` instance is used for profiling.
         *  
         *  \param[in] timer `GPUTimer` that does the profiling of the kernel executions.
         *  \param[in] events a wait-list of events.
         *  \return Τhe total execution time measured by the timer.
         */
        template <typename period>
        double run (clutils::GPUTimer<period> &timer, const std::vector<cl::Event> *events = nullptr)
        {
            queue.enqueueNDRangeKernel (kernel, cl::NullRange, global, cl::NullRange, events, &timer.event ());
            queue.flush (); timer.wait ();

            return timer.duration ();
        }

    };

}
}

#endif  // OCLSLAM_ALGORITHMS_HPP
