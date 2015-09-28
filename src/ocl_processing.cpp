/*! \file ocl_processing.cpp
 *  \brief Defines the classes for setting up the OpenCL `%OCLSLAM` pipeline.
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

#include <thread>
#include <mutex>
#include <chrono>
#include <ocl_processing.hpp>


// OpenGL buffer parameters
extern GLuint glPC4DBuffer, glRGBABuffer;

std::mutex glMtx;  // Controls access to OpenGL buffers
std::mutex mapMtx;  // Controls access to the map


const std::vector<std::string> kernel_files_gf = { "kernels/GF/imageSupport_kernels.cl", 
                                                   "kernels/GF/scan_kernels.cl", 
                                                   "kernels/GF/transpose_kernels.cl", 
                                                   "kernels/GF/boxFilter_kernels.cl",
                                                   "kernels/GF/math_kernels.cl", 
                                                   "kernels/GF/guidedFilter_kernels.cl" };

const std::vector<std::string> kernel_files_rbc = { "kernels/RBC/reduce_kernels.cl", 
                                                    "kernels/RBC/scan_kernels.cl", 
                                                    "kernels/RBC/rbc_kernels.cl" };

const std::vector<std::string> kernel_files_icp = { "kernels/ICP/reduce_kernels.cl", 
                                                    "kernels/ICP/icp_kernels.cl" };

const std::vector<std::string> kernel_files_slam = { "kernels/oclslam/slam_kernels.cl" };


/*! \param[in] width width (in pixels) of the associated point clouds.
 *  \param[in] height height (in pixels) of the associated point clouds.
 *  \param[in] numPC maximum number of point clouds that the OpenGL buffers will hold.
 */
CLEnvGL::CLEnvGL (int width, int height, int numPC) : 
    CLEnv (), width (width), height (height), numPC (numPC)
{
    addContext (0, true);
    addQueueGL (0);
    addQueueGL (0);
    addProgram (0, kernel_files_gf);
    addProgram (0, kernel_files_rbc);
    addProgram (0, kernel_files_icp);
    addProgram (0, kernel_files_slam);
}


/*! \note Do not call directly. `initGLMemObjects` is called by `addContext`
 *        when creating the GL-shared CL context.
 */
void CLEnvGL::initGLMemObjects ()
{
    glGenBuffers (1, &glPC4DBuffer);
    glBindBuffer (GL_ARRAY_BUFFER, glPC4DBuffer);
    glBufferData (GL_ARRAY_BUFFER, numPC * width * height * sizeof (cl_float4), NULL, GL_DYNAMIC_DRAW);
    glGenBuffers (1, &glRGBABuffer);
    glBindBuffer (GL_ARRAY_BUFFER, glRGBABuffer);
    glBufferData (GL_ARRAY_BUFFER, numPC * width * height * sizeof (cl_float4), NULL, GL_DYNAMIC_DRAW);
    glBindBuffer (GL_ARRAY_BUFFER, 0);
}


/*! \details Initializes the OpenCL environment, the OpenGL buffers, 
 *           and the classes for the `SLAM` pipeline.
 *  
 *  \param[in] kinect initialized Kinect device.
 *  \param[in] map OctoMap structure for building the map.
 */
template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
OCLSLAM<CR, CW>::OCLSLAM (Kinect *kinect, octomap::OcTree &map) : 
    timeStep (0), map (map), gfRGBRadius (5), gfRGBEps (0.02f), gfDRadius (10), 
    gfDEps (0.01f), gfDScaling (1e-3f), focalLength (595.f), a (2e2f), c (1e-6f), 
    max_iterations (40), angle_threshold (0.001), translation_threshold (0.01), 
    slamStatus (false), gfRGBStatus (true), gfDStatus (true), rgbNorm (1), 
    maxPCGL (200), width (640), height (480), n (640 * 480), m (16384), r (256), 
    env (width, height, maxPCGL), infoGF (0, 0, 0, { 0, 1 }, 0), infoRBC (0, 0, 0, { 0 }, 1), 
    infoICP (0, 0, 0, { 0 }, 2), infoSLAM (0, 0, 0, { 0 }, 3), context (env.getContext (0)), 
    queue0 (env.getQueue (0, 0)), queue1 (env.getQueue (0, 1)), kinect (kinect), 
    gfRGB (env, infoGF), gfD (env, infoGF), sepRGB (env, infoGF.getCLEnvInfo (0)), 
    convD (env, infoGF.getCLEnvInfo (0)), to8D (env, infoGF.getCLEnvInfo (0)), lm (env, infoICP), 
    icp (env, infoRBC, infoICP), transform (env, infoICP), sp8D (env, infoGF.getCLEnvInfo (1)), 
    sp8DMap (env, infoSLAM.getCLEnvInfo (0)), waitListGL (1), lICP (0), pc (n) //, cc (n)
{
    // Create input buffers (they will be receiving the Kinect frames)
    hBufferRGB = cl::Buffer (context, CL_MEM_ALLOC_HOST_PTR, n * 3 * sizeof (cl_uchar));
    hBufferD = cl::Buffer (context, CL_MEM_ALLOC_HOST_PTR, n * sizeof (cl_ushort));
    dBufferRGB = cl::Buffer (context, CL_MEM_READ_ONLY, n * 3 * sizeof (cl_uchar));
    dBufferD = cl::Buffer (context, CL_MEM_READ_ONLY, n * sizeof (cl_ushort));

    // Set the buffers in which Kinect will be dropping off its frames
    kinect->setBuffers (queue0, hBufferRGB, hBufferD);

    // Create the host buffer that will hold the global coordinates and orientation
    hBufferTg = cl::Buffer (context, CL_MEM_ALLOC_HOST_PTR, 2 * sizeof (cl_float4));
    hPtrTg = (cl_float *) queue1.enqueueMapBuffer (hBufferTg, CL_FALSE, CL_MAP_WRITE, 0, 2 * sizeof (cl_float4));
    queue1.enqueueUnmapMemObject (hBufferTg, hPtrTg);

    // Create GL-shared buffers
    dBufferGL.emplace_back (context, CL_MEM_WRITE_ONLY, glPC4DBuffer);
    dBufferGL.emplace_back (context, CL_MEM_WRITE_ONLY, glRGBABuffer);
    queue1.enqueueFillBuffer<cl_float> (dBufferGL[0], (cl_float) 0.f, 0, maxPCGL * n * sizeof (cl_float4));
    queue1.enqueueFillBuffer<cl_float> (dBufferGL[1], (cl_float) 0.f, 0, maxPCGL * n * sizeof (cl_float4));

    // Initialize the preprocessing pipeline ==================================

    to8D.get (GF::RGBDTo8D::Memory::D_IN_D) = cl::Buffer (context, CL_MEM_READ_WRITE, n * sizeof (cl_float));
    to8D.get (GF::RGBDTo8D::Memory::D_IN_R) = cl::Buffer (context, CL_MEM_READ_WRITE, n * sizeof (cl_float));
    to8D.get (GF::RGBDTo8D::Memory::D_IN_G) = cl::Buffer (context, CL_MEM_READ_WRITE, n * sizeof (cl_float));
    to8D.get (GF::RGBDTo8D::Memory::D_IN_B) = cl::Buffer (context, CL_MEM_READ_WRITE, n * sizeof (cl_float));
    to8D.get (GF::RGBDTo8D::Memory::D_OUT) = cl::Buffer (context, CL_MEM_READ_WRITE, n * sizeof (cl_float8));
    to8D.init (width, height, focalLength, 1.f, rgbNorm, GF::Staging::NONE);

    // with Guided Image Filtering ========================

    const GF::Kinect::GuidedFilterRGBConfig cGFRGB = GF::Kinect::GuidedFilterRGBConfig::SEPARATED;
    gfRGB.get (GF::Kinect::GuidedFilterRGB<cGFRGB>::Memory::D_IN) = dBufferRGB;
    gfRGB.get (GF::Kinect::GuidedFilterRGB<cGFRGB>::Memory::D_OUT_R) = to8D.get (GF::RGBDTo8D::Memory::D_IN_R);
    gfRGB.get (GF::Kinect::GuidedFilterRGB<cGFRGB>::Memory::D_OUT_G) = to8D.get (GF::RGBDTo8D::Memory::D_IN_G);
    gfRGB.get (GF::Kinect::GuidedFilterRGB<cGFRGB>::Memory::D_OUT_B) = to8D.get (GF::RGBDTo8D::Memory::D_IN_B);
    gfRGB.init (width, height, gfRGBRadius, gfRGBEps, GF::Staging::NONE);

    gfD.get (GF::Kinect::GuidedFilterDepth::Memory::D_IN) = dBufferD;
    gfD.get (GF::Kinect::GuidedFilterDepth::Memory::D_OUT) = to8D.get (GF::RGBDTo8D::Memory::D_IN_D);
    gfD.init (width, height, gfDRadius, gfDEps, gfDScaling, GF::Staging::NONE);

    // ====================================================
    // \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    // without Guided Image Filtering =====================

    sepRGB.get (GF::SeparateRGB<GF::SeparateRGBConfig::UCHAR_FLOAT>::Memory::D_IN) = dBufferRGB;
    sepRGB.get (GF::SeparateRGB<GF::SeparateRGBConfig::UCHAR_FLOAT>::Memory::D_OUT_R) = 
        to8D.get (GF::RGBDTo8D::Memory::D_IN_R);
    sepRGB.get (GF::SeparateRGB<GF::SeparateRGBConfig::UCHAR_FLOAT>::Memory::D_OUT_G) = 
        to8D.get (GF::RGBDTo8D::Memory::D_IN_G);
    sepRGB.get (GF::SeparateRGB<GF::SeparateRGBConfig::UCHAR_FLOAT>::Memory::D_OUT_B) = 
        to8D.get (GF::RGBDTo8D::Memory::D_IN_B);
    sepRGB.init (width, height, GF::Staging::NONE);

    convD.get (GF::Depth<GF::DepthConfig::USHORT_FLOAT>::Memory::D_IN) = dBufferD;
    convD.get (GF::Depth<GF::DepthConfig::USHORT_FLOAT>::Memory::D_OUT) = to8D.get (GF::RGBDTo8D::Memory::D_IN_D);
    convD.init (width, height, 1.f, GF::Staging::NONE);

    // ====================================================

    lm.get (ICP::ICPLMs::Memory::D_IN) = to8D.get (GF::RGBDTo8D::Memory::D_OUT);
    lm.get (ICP::ICPLMs::Memory::D_OUT) = cl::Buffer (context, CL_MEM_READ_WRITE, m * sizeof (cl_float8));
    lm.init (ICP::Staging::NONE);

    // ========================================================================
    // ------------------------------------------------------------------------
    // Initialize the ICP pipeline ============================================

    icp.get (ICP::ICP<CR, CW>::Memory::D_IN_F) = cl::Buffer (context, CL_MEM_READ_WRITE, m * sizeof (cl_float8));
    icp.get (ICP::ICP<CR, CW>::Memory::D_IN_M) = lm.get (ICP::ICPLMs::Memory::D_OUT);
    icp.init (m, r, a, c, max_iterations, angle_threshold, translation_threshold, ICP::Staging::NONE);

    // ========================================================================
    // ------------------------------------------------------------------------
    // Initialize the postprocessing pipeline =================================

    transform.get (ICP::ICPTransform<ICP::ICPTransformConfig::QUATERNION>::Memory::D_IN_M) = 
        to8D.get (GF::RGBDTo8D::Memory::D_OUT);
    transform.get (ICP::ICPTransform<ICP::ICPTransformConfig::QUATERNION>::Memory::D_OUT) = 
        cl::Buffer (context, CL_MEM_READ_WRITE, n * sizeof (cl_float8));
    transform.init (n, ICP::Staging::NONE);

    sp8D.get (GF::SplitPC8D::Memory::D_IN) = 
        transform.get (ICP::ICPTransform<ICP::ICPTransformConfig::QUATERNION>::Memory::D_OUT);
    sp8D.get (GF::SplitPC8D::Memory::D_OUT_PC4D) = dBufferGL[0];
    sp8D.get (GF::SplitPC8D::Memory::D_OUT_RGBA) = dBufferGL[1];
    sp8D.init (n, (maxPCGL - 1) * n, GF::Staging::NONE);

    sp8DMap.get (oclslam::SplitPC8D::Memory::D_IN) = 
        transform.get (ICP::ICPTransform<ICP::ICPTransformConfig::QUATERNION>::Memory::D_OUT);
    sp8DMap.init (n, oclslam::Staging::O);
    queue0.finish ();
    queue1.finish ();
    // ========================================================================

    // Initialize global coordinates and orientation
    R_g = Eigen::Matrix3f::Identity ();
    q_g = Eigen::Quaternionf (R_g);
    t_g.setZero ();
    s_g = 1.f;
    
    // Start Kinect
    kinect->startVideo ();
    kinect->startDepth ();

    // Initialize the SLAM function wrapper
    slam = [this] { std::thread ([this] { init (); }).detach (); };
    slamFuncHashCode = slam.target_type ().hash_code ();
}


template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
OCLSLAM<CR, CW>::~OCLSLAM ()
{
    kinect->stopVideo ();
    kinect->stopDepth ();
}


/*! \details It sets up the first point cloud. After that, `registerPointCloud` 
 *           can be called to perform the registration process.
 */
template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
void OCLSLAM<CR, CW>::init ()
{
    if (slam.target_type ().hash_code () != slamFuncHashCode) return;

    // Host-Device Transfer ===============================================

    kinect->deliverFrames (queue0, dBufferRGB, dBufferD);
    // queue0.enqueueFillBuffer<cl_uchar> (dBufferRGB, (cl_uchar) 255, 0, 3 * n * sizeof (cl_uchar));
    // queue0.enqueueFillBuffer<cl_ushort> (dBufferD, (cl_ushort) 2000, 0, n * sizeof (cl_ushort));

    // ====================================================================
    // --------------------------------------------------------------------
    // Preprocessing ======================================================

    if (gfRGBStatus) gfRGB.run (); else sepRGB.run ();
    if (gfDStatus) gfD.run (); else convD.run ();
    to8D.run ();
    lm.run ();

    queue0.enqueueCopyBuffer ((cl::Buffer &) to8D.get (GF::RGBDTo8D::Memory::D_OUT), 
        (cl::Buffer &) transform.get (ICP::ICPTransform<ICP::ICPTransformConfig::QUATERNION>::Memory::D_OUT), 
        0, 0, n * sizeof (cl_float8), nullptr, &eventGL); waitListGL[0] = eventGL;

    // ====================================================================
    // --------------------------------------------------------------------
    // Postprocessing =====================================================

    sp8DMap.run ();
    sp8DMap.read (oclslam::SplitPC8D::Memory::H_OUT_PC3D, CL_FALSE);
    // sp8DMap.read (oclslam::SplitPC8D::Memory::H_OUT_RGB, CL_FALSE);

    queue0.flush ();

    // ====================================================================
    // --------------------------------------------------------------------
    // OpenGL Rendering ===================================================

    if (timeStep < maxPCGL)
    {
        glMtx.lock ();  // Prevent the OpenGL renderer from reading the buffers

        glFinish ();  // Wait for OpenGL pending operations on the buffers to finish

        // Take ownership of the OpenGL buffers
        queue1.enqueueAcquireGLObjects ((std::vector<cl::Memory> *) &dBufferGL);

        sp8D.setOffset (0);
        sp8D.run (&waitListGL);

        // Give up ownership of the OpenGL buffers
        queue1.enqueueReleaseGLObjects ((std::vector<cl::Memory> *) &dBufferGL);

        queue1.finish ();

        timeStep++;  // Count the current point cloud

        glMtx.unlock ();
    }

    // ====================================================================

    display ();

    // --------------------------------------------------------------------
    // Mapping ============================================================

    queue0.finish ();

    global_pos = octomap::point3d (0.0, 0.0, 0.0);
    std::thread ([this] { _mapping (); }).detach ();

    // ====================================================================

    // Update SLAM function wrapper
    slam = [this] { std::thread ([this] {
            while (slamStatus) registerPointCloud ();
        }).detach ();
    };

    if (slamStatus) slam ();
}


/*! \details Runs `ICP` on the current point cloud, delivers the result to 
 *           OpenGL for visualization, and to OctoMap, in order to update the map.
 */
template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
void OCLSLAM<CR, CW>::registerPointCloud ()
{
    // Host-Device Transfer ===============================================

    kinect->deliverFrames (queue0, dBufferRGB, dBufferD);
    // queue0.enqueueFillBuffer<cl_uchar> (dBufferRGB, (cl_uchar) 100, 0, 3 * n * sizeof (cl_uchar));
    // queue0.enqueueFillBuffer<cl_ushort> (dBufferD, (cl_ushort) 1700, 0, n * sizeof (cl_ushort));
    
    // ====================================================================
    // --------------------------------------------------------------------
    // Preprocessing ======================================================

    if (gfRGBStatus) gfRGB.run (); else sepRGB.run ();
    if (gfDStatus) gfD.run (); else convD.run ();
    to8D.run ();
    queue0.enqueueCopyBuffer ((cl::Buffer &) icp.get (ICP::ICP<CR, CW>::Memory::D_IN_M), 
        (cl::Buffer &) icp.get (ICP::ICP<CR, CW>::Memory::D_IN_F), 0, 0, m * sizeof (cl_float8));
    lm.run ();
    icp.buildRBC ();

    // ====================================================================
    // --------------------------------------------------------------------
    // ICP ================================================================

    timerICP.start ();
    icp.run ();
    lICP = timerICP.stop ();

    // Update global coordinates and orientation ===========

    R_g = icp.R * R_g;
    q_g = Eigen::Quaternionf (R_g);
    t_g = icp.s * icp.R * t_g + icp.t;
    s_g = icp.s * s_g;
    
    Eigen::Map<Eigen::Vector4f> (hPtrTg, 4) = q_g.coeffs ();  // Quaternion
    Eigen::Map<Eigen::Vector4f> (hPtrTg + 4, 4) = t_g.homogeneous ();  // Translation
    hPtrTg[7] = s_g;  // Scale

    queue0.enqueueWriteBuffer ((cl::Buffer &) transform.get (
        ICP::ICPTransform<ICP::ICPTransformConfig::QUATERNION>::Memory::D_IN_T), 
        CL_FALSE, 0, 2 * sizeof (cl_float4), hPtrTg);

    // ======================================================
    
    transform.run (nullptr, &eventGL); waitListGL[0] = eventGL;

    // ====================================================================
    // --------------------------------------------------------------------
    // Postprocessing =====================================================

    sp8DMap.run ();
    sp8DMap.read (oclslam::SplitPC8D::Memory::H_OUT_PC3D, CL_FALSE);
    // sp8DMap.read (oclslam::SplitPC8D::Memory::H_OUT_RGB, CL_FALSE);

    queue0.flush ();

    // ====================================================================
    // --------------------------------------------------------------------
    // OpenGL Rendering ===================================================

    if (timeStep < maxPCGL)
    {
        glMtx.lock ();  // Prevent the OpenGL renderer from reading the buffers

        glFinish ();  // Wait for OpenGL pending operations on the buffers to finish

        // Take ownership of the OpenGL buffers
        queue1.enqueueAcquireGLObjects ((std::vector<cl::Memory> *) &dBufferGL);

        sp8D.setOffset (timeStep * n);
        sp8D.run (&waitListGL);

        // Give up ownership of the OpenGL buffers
        queue1.enqueueReleaseGLObjects ((std::vector<cl::Memory> *) &dBufferGL);

        queue1.finish ();

        timeStep++;  // Count the current point cloud

        glMtx.unlock ();
    }

    // ====================================================================

    display ();

    // --------------------------------------------------------------------
    // Mapping ============================================================

    queue0.finish ();

    // Locking the mutex at this point allows the algorithm to start processing 
    // the next point cloud, but prevents it from going further ahead
    std::lock_guard<std::mutex> lock (mapMtx);

    global_pos = octomap::point3d (t_g[0] * 0.001, t_g[1] * 0.001, t_g[2] * 0.001);
    std::thread ([this] { _mapping (); }).detach ();

    // ====================================================================
}


/*! \brief Retrieves a point cloud and inserts it into the map. */
template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
void OCLSLAM<CR, CW>::_mapping ()
{
    std::lock_guard<std::mutex> lock (mapMtx);

    /*! \todo Remove invalid points at the beginning of the pipeline, 
     *        and resize `pc` to the resulting size. */
    // pc.resize (<n>);

    std::copy (sp8DMap.hPtrOutPC3D, sp8DMap.hPtrOutPC3D + 3 * n, (cl_float *) pc.data ());
    // std::copy (sp8DMap.hPtrOutRGB, sp8DMap.hPtrOutRGB + 3 * n, (cl_uchar *) cc.data ());
    
    map.insertPointCloud (pc, global_pos, -1, false, true);
    // map.insertPointCloud (pc, global_pos, -1, true, true); 
    // for (int i = 0; i < n; ++i)
    // {
    //     octomap::OcTreeKey key = map.coordToKey (pc[i]);
    //     octomap::ColorOcTreeNode *node = map.search (key);
    //     node->setColor (cc[i]);
    // }
    // map.updateInnerOccupancy ();
}


/*! \param[in] filename name for the map file `[.ot]`. */
template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
void OCLSLAM<CR, CW>::write (std::string filename)
{
    std::lock_guard<std::mutex> lock (mapMtx);
    std::thread ([&] { map.write (filename.c_str ()); }).detach ();
    std::cout << "Map saved in file " << filename << std::endl;
}


/*! \param[in] filename name for the map file `[.bt]`. */
template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
void OCLSLAM<CR, CW>::writeBinary (std::string filename)
{
    std::lock_guard<std::mutex> lock (mapMtx);
    std::thread ([&] { map.writeBinary (filename.c_str ()); }).detach ();
    std::cout << "Map saved in file " << filename << std::endl;
}


template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
void OCLSLAM<CR, CW>::display ()
{
    double angle = 180.0 / M_PI * 2.0 * std::atan2 (q_g.vec ().norm (), q_g.w ());  // in degrees
    Eigen::Vector3f axis = ((angle == 0.0) ? Eigen::Vector3f::Zero () : q_g.vec ().normalized ());
    std::cout << "    Time step             :    " << timeStep << std::endl;
    std::cout << "    Latency               :    " << timer.stop () << " [ms]" << std::endl;
    std::cout << "    ICP iterations        :    " << icp.k << std::endl;
    std::cout << "    ICP latency           :    " << lICP << " [ms]" << std::endl;
    std::cout << "    Localization               " << std::endl;
    std::cout << "    - Translation vector  :    " << t_g.transpose () << " [mm]" << std::endl;
    std::cout << "    - Rotation axis       :    " << axis.transpose () << std::endl;
    std::cout << "    - Rotation angle      :    " << angle << " [degrees]" << std::endl;
    std::cout << "===========================    " << std::endl;
    timer.start ();
}


template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
void OCLSLAM<CR, CW>::setSLAMStatus (bool flag)
{
    slamStatus = flag;

    if (slamStatus)
        std::thread ([&] { slam (); }).detach ();
}


template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
void OCLSLAM<CR, CW>::toggleSLAMStatus ()
{
    slamStatus = !slamStatus;

    if (slamStatus)
        std::thread ([&] { slam (); }).detach ();
}


/*! \brief Instantiation that uses the Eigen library to estimate the rotation, and considers regular residual errors. */
template class OCLSLAM<ICP::ICPStepConfigT::EIGEN, ICP::ICPStepConfigW::REGULAR>;
/*! \brief Instantiation that uses the Eigen library to estimate the rotation, and considers weighted residual errors. */
template class OCLSLAM<ICP::ICPStepConfigT::EIGEN, ICP::ICPStepConfigW::WEIGHTED>;
/*! \brief Instantiation that uses the Power Method to estimate the rotation, and considers regular residual errors. */
template class OCLSLAM<ICP::ICPStepConfigT::POWER_METHOD, ICP::ICPStepConfigW::REGULAR>;
/*! \brief Instantiation that uses the Power Method to estimate the rotation, and considers weighted residual errors. */
template class OCLSLAM<ICP::ICPStepConfigT::POWER_METHOD, ICP::ICPStepConfigW::WEIGHTED>;
