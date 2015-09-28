/*! \file ocl_processing.hpp
 *  \brief Declares the classes for setting up the OpenCL `%OCLSLAM` pipeline.
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

#ifndef OCL_PROCESSING_HPP
#define OCL_PROCESSING_HPP

#include <functional>
#include <mutex>
#include <GL/glew.h>  // Add before CLUtils.hpp
#include <CLUtils.hpp>
#include <GuidedFilter/algorithms.hpp>
#include <ICP/algorithms.hpp>
#include <eigen3/Eigen/Dense>
#include <freenect_rgbd.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
// #include <octomap/ColorOcTree.h>
#include <oclslam/pointcloud.hpp>
#include <oclslam/algorithms.hpp>

using namespace cl_algo;


/*! \brief Creates an OpenCL environment with CL-GL interoperability. */
class CLEnvGL : public clutils::CLEnv
{
public:
    /*! \brief Initializes the OpenCL environment. */
    CLEnvGL (int width, int height, int numPC);

private:
    /*! \brief Initializes the OpenGL memory buffers. */
    void initGLMemObjects ();

    int width, height, numPC;

};


/*! \brief Interface class for the `SLAM` pipeline.
 *  \details Retrieves data, registers point clouds, and builds a map.
 *  \note In order to handle memory consumption, there is a limit 
 *        on the number of point clouds displayed on the screen. 
 *        When the limit is reached though, the SLAM process keeps 
 *        running on the background and the map gets updated as normal.
 *  
 *  \tparam CR configures the class with different methods of rotation computation.
 *  \tparam CW configures the class for performing either regular or weighted computation.
 */
template <ICP::ICPStepConfigT CR, ICP::ICPStepConfigW CW>
class OCLSLAM
{
public:
    /*! \brief Constructor. */
    OCLSLAM (Kinect *kinect, octomap::OcTree &map);
    /*! \brief Destructor. */
    ~OCLSLAM ();
    /*! \brief Initializes the SLAM pipeline. */
    void init ();
    /*! \brief Registers a point cloud. */
    void registerPointCloud ();
    /*! \brief Stores an occupancy map on disk. */
    void write (std::string filename = std::string ("map.ot"));
    /*! \brief Stores an binary map on disk. */
    void writeBinary (std::string filename = std::string ("map.bt"));
    /*! \brief Prints on the console results about the current 
     *         registration and localization. */
    void display ();
    /*! \brief Gets the status of the automated SLAM process. */
    bool getSLAMStatus () { return slamStatus; }
    /*! \brief Sets the status of the automated SLAM process. */
    void setSLAMStatus (bool flag);
    /*! \brief Toggles the status of the automated SLAM process. */
    void toggleSLAMStatus ();
    /*! \brief Gets the status of the RGB Guided Filter. */
    bool getGFRGBStatus () { return gfRGBStatus; }
    /*! \brief Sets the status of the RGB Guided Filter. */
    void setGFRGBStatus (bool flag) { gfRGBStatus = flag; }
    /*! \brief Toggles the status of the RGB Guided Filter. */
    void toggleGFRGBStatus () { gfRGBStatus = !gfRGBStatus; }
    /*! \brief Gets the status of the Depth Guided Filter. */
    bool getGFDStatus () { return gfDRadius; }
    /*! \brief Sets the status of the Depth Guided Filter. */
    void setGFDStatus (bool flag) { gfDStatus = flag; }
    /*! \brief Toggles the status of the Depth Guided Filter. */
    void toggleGFDStatus () { gfDStatus = !gfDStatus; }
    /*! \brief Gets the status of the RGB normalization. */
    int getRGBNormalization () { return rgbNorm; }
    /*! \brief Sets the status of the RGB normalization. */
    void setRGBNormalization (int flag) { rgbNorm = flag; to8D.setRGBNorm (rgbNorm); }
    /*! \brief Toggles the status of the RGB normalization. */
    void toggleRGBNormalization () { rgbNorm = !rgbNorm; to8D.setRGBNorm (rgbNorm); }
    /*! \brief Gets the window radius \f$r\f$ for the guided filter performed on the RGB frame. */
    int getGFRGBRadius () { return gfRGB.getRadius (); }
    /*! \brief Sets the window radius \f$r\f$ for the guided filter performed on the RGB frame. */
    void setGFRGBRadius (int radius) { gfRGBRadius = radius; gfRGB.setRadius (radius); }
    /*! \brief Gets the variability threshold \f$\epsilon\f$ for the guided filter performed on the RGB frame. */
    float getGFRGBEps () { return gfRGB.getEps (); }
    /*! \brief Sets the variability threshold \f$\epsilon\f$ for the guided filter performed on the RGB frame. */
    void setGFRGBEps (float eps) { gfRGBEps = eps; gfRGB.setEps (eps); }
    /*! \brief Gets the window radius \f$r\f$ for the guided filter performed on the Depth frame. */
    int getGFDRadius () { return gfD.getRadius (); }
    /*! \brief Sets the window radius \f$r\f$ for the guided filter performed on the Depth frame. */
    void setGFDRadius (int radius) { gfDRadius = radius; gfD.setRadius (radius); }
    /*! \brief Gets the variability threshold \f$\epsilon\f$ for the guided filter performed on the Depth frame. */
    float getGFDEps () { return gfD.getEps (); }
    /*! \brief Sets the variability threshold \f$\epsilon\f$ for the guided filter performed on the Depth frame. */
    void setGFDEps (float eps) { gfDEps = eps; gfD.setEps (eps); }
    /*! \brief Gets the scaling applied to the depth frame for processing with the guided filter. */
    float getGFDScaling () { return gfD.getDScaling (); }
    /*! \brief Sets the scaling applied to the depth frame for processing with the guided filter. */
    void setGFDScaling (float scaling) { gfDScaling = scaling; gfD.setDScaling (scaling); }
    /*! \brief Gets the sensor's focal length. */
    float getSensorFocalLength () { return to8D.getFocalLength (); }
    /*! \brief Sets the sensor's focal length. */
    void setSensorFocalLength (float f) { focalLength = f; to8D.setFocalLength (f); }
    /*! \brief Gets the parameter \f$ \alpha \f$ used in the distance function for the RBC data structure. */
    float getRBCAlpha () { return icp.getAlpha (); }
    /*! \brief Sets the parameter \f$ \alpha \f$ used in the distance function for the RBC data structure. */
    void setRBCAlpha (float _a) { a = _a; icp.setAlpha (_a); }
    /*! \brief Gets the scaling applying to the deviations when computing matrix `S` 
     *         in the ICP algorithm, for managing floating point arithmetic issues. */
    float getICPSScaling () { return icp.getScaling (); }
    /*! \brief Sets the scaling applying to the deviations when computing matrix `S` 
     *         in the ICP algorithm, for managing floating point arithmetic issues. */
    void setICPSScaling (float _c) { c = _c; icp.setScaling (_c); }
    /*! \brief Gets the maximum number of iterations considered for an ICP registration. */
    unsigned int getICPMaxIterations () { return icp.getMaxIterations (); }
    /*! \brief Sets the maximum number of iterations considered for an ICP registration. */
    void setICPMaxIterations (unsigned int maxIter) { max_iterations = maxIter; icp.setMaxIterations (maxIter); }
    /*! \brief Gets the angle threshold (in degrees) for the convergence check of the ICP. */
    double getICPAngleThreshold () { return icp.getAngleThreshold (); }
    /*! \brief Sets the angle threshold (in degrees) for the convergence check of the ICP. */
    void setICPAngleThreshold (double at) { angle_threshold = at; icp.setAngleThreshold (at); }
    /*! \brief Gets the translation threshold (in mm) for the convergence check of the ICP. */
    double getICPTranslationThreshold () { return icp.getTranslationThreshold (); }
    /*! \brief Sets the translation threshold (in mm) for the convergence check of the ICP. */
    void setICPTranslationThreshold (double tt) { translation_threshold = tt; icp.setTranslationThreshold (tt); }

    /*! \brief Performs the SLAM process.
     *  \details Initially, it points to `init` for registering the first point cloud, and 
     *           after that it gets assigned with a function that runs repeatedly `registerPointCloud`.
     */
    std::function<void ()> slam;

    volatile int timeStep;  /*!< Counts the discrete time steps, as in the number of point clouds registered. */

    // Global localization parameters
    Eigen::Matrix3f R_g;     /*!< Represents the orietation with respect to the global coordinate frame, 
                              *   given in rotation matrix representation, \f$ R_g \f$. */
    Eigen::Quaternionf q_g;  /*!< Represents the orietation with respect to the global coordinate frame, 
                              *   given in quaternion representation, \f$ \dot{q}_g = \left[ 
                              *   \begin{matrix} q_x & q_y & q_z & q_w \end{matrix} \right]^T \f$. */
    Eigen::Vector3f t_g;     /*!< Represents the translation (in mm) with respect to the global coordinate frame, 
                              *   given as a vector in 3-D, \f$ t_g \f$. */
    cl_float s_g;            /*!< Represents the scale of the current point cloud with respect to the first one, 
                              *   given as a scalar, \f$ s_g \f$. It's used when transforming the current point 
                              *   cloud before mapping. */

private:
    void _mapping ();

    // Internal parameters
    int gfRGBRadius;
    float gfRGBEps;
    int gfDRadius;
    float gfDEps;
    float gfDScaling;
    float focalLength;
    float a;
    float c;
    unsigned int max_iterations;
    double angle_threshold;
    double translation_threshold;

    // External parameters
    volatile bool slamStatus;
    volatile bool gfRGBStatus;
    volatile bool gfDStatus;
    volatile int rgbNorm;

    size_t slamFuncHashCode;
    int maxPCGL;  // Limits in the number of point clouds held in memory for visualization
    unsigned int width, height;
    unsigned int n;  // Number of points in a point cloud
    unsigned int m;  // Number of landmarks
    unsigned int r;  // Number of representatives

    CLEnvGL env;
    clutils::CLEnvInfo<2> infoGF;
    clutils::CLEnvInfo<1> infoRBC, infoICP, infoSLAM;
    cl::Context &context;
    cl::CommandQueue &queue0, &queue1;
    
    Kinect *kinect;

    cl_float *hPtrTg;
    cl::Buffer hBufferTg;
    cl::Buffer hBufferRGB, hBufferD;
    cl::Buffer dBufferRGB, dBufferD;
    std::vector<cl::BufferGL> dBufferGL;

    GF::Kinect::GuidedFilterRGB<GF::Kinect::GuidedFilterRGBConfig::SEPARATED> gfRGB;
    GF::Kinect::GuidedFilterDepth gfD;
    GF::SeparateRGB<GF::SeparateRGBConfig::UCHAR_FLOAT> sepRGB;
    GF::Depth<GF::DepthConfig::USHORT_FLOAT> convD;
    GF::RGBDTo8D to8D;
    ICP::ICPLMs lm;
    ICP::ICP<CR, CW> icp;
    ICP::ICPTransform<ICP::ICPTransformConfig::QUATERNION> transform;
    GF::SplitPC8D sp8D;
    oclslam::SplitPC8D sp8DMap;

    cl::Event eventGL;
    std::vector<cl::Event> waitListGL;
    clutils::CPUTimer<double, std::milli> timer;
    clutils::CPUTimer<double, std::milli> timerICP;
    volatile double lICP;

    // Map parameters
    octomap::point3d global_pos;  // Global position in meters
    oclslam::PointCloud pc;
    // std::vector<octomap::ColorOcTreeNode::Color> cc;
    octomap::OcTree &map;
    // octomap::ColorOcTree &map;
};

#endif  // OCL_PROCESSING_HPP
