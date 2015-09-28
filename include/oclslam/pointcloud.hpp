/*! \file pointcloud.hpp
 *  \brief Declares classes that enhance the default `octomap::Pointcloud`.
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

#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <octomap/octomap.h>


namespace cl_algo
{
namespace oclslam
{

    /*! \brief Enhances `octomap::Pointcloud`.
     *  \details Defines an API for manipulating directly the enclosed `octomap::point3d` vector.
     */
    class PointCloud : public octomap::Pointcloud
    {
    public:
        PointCloud () {}
        PointCloud (const octomap::Pointcloud &other) : octomap::Pointcloud (other) {}
        PointCloud (octomap::Pointcloud *other) : octomap::Pointcloud (other) {}
        PointCloud (PointCloud *other) : octomap::Pointcloud (other) {}
        PointCloud (size_t n) { points.resize (n); }
        inline void resize (size_t n) { points.resize (n); }
        inline octomap::point3d *data () { return points.data (); }
        inline void emplace_back (float x, float y, float z) { points.emplace_back (x, y, z); }
    };

}
}

#endif  // POINTCLOUD_HPP
